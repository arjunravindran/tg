/*
 * tests/test_serializer.c
 *
 * Unit tests for src/serializer.c (write_file / read_file).
 *
 * Covers:
 *   - Empty snapshot list roundtrip (issue #01 regression)
 *   - Single and multiple snapshot field fidelity
 *   - Double serialization bit-exactness (issue #02 regression)
 *   - Error handling on empty / truncated files
 *   - Unnamed snapshot (NULL name) roundtrip
 */

#include "../src/tg.h"
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

/* -----------------------------------------------------------------------
 * Stubs for symbols defined in interface.c and audio.c that are referenced
 * by computer.c / serializer.c but are never called from test paths.
 * ----------------------------------------------------------------------- */

int preset_bph[] = PRESET_BPH;

void error(char *format, ...)
{
    va_list args;
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
    fputc('\n', stderr);
}

void set_audio_light(bool light)               { (void)light; }
uint64_t get_timestamp(int light)              { (void)light; return 0; }

int analyze_pa_data(struct processing_data *pd, int bph, double la,
                    uint64_t events_from)
{
    (void)pd; (void)bph; (void)la; (void)events_from;
    return 0;
}

int analyze_pa_data_cal(struct processing_data *pd,
                        struct calibration_data *cd)
{
    (void)pd; (void)cd;
    return 0;
}

/* -----------------------------------------------------------------------
 * Minimal test framework
 * ----------------------------------------------------------------------- */

static int tests_run    = 0;
static int tests_failed = 0;

/*
 * On failure: print location, increment counter, and return from the
 * calling test function.  Return is void because all test functions are
 * void; the RUN_TEST macro records whether tests_failed changed.
 */
#define ASSERT(cond, msg) do {                                              \
    if (!(cond)) {                                                          \
        fprintf(stderr, "  FAIL at %s:%d: %s\n", __FILE__, __LINE__, msg); \
        tests_failed++;                                                     \
        return;                                                             \
    }                                                                       \
} while(0)

#define RUN_TEST(fn) do {                           \
    tests_run++;                                    \
    int _before = tests_failed;                     \
    fn();                                           \
    if (tests_failed == _before)                    \
        fprintf(stdout, "PASS: " #fn "\n");         \
    else                                            \
        fprintf(stdout, "FAIL: " #fn "\n");         \
} while(0)

/* -----------------------------------------------------------------------
 * Snapshot construction / destruction helpers
 * ----------------------------------------------------------------------- */

/*
 * Allocate a snapshot that passes every validation check in scan_snapshot.
 * The pb is malloc'd (not set up via setup_buffers), so it has no FFTW
 * plans — use free_test_snapshot() to release it rather than
 * snapshot_destroy().
 */
static struct snapshot *make_test_snapshot(void)
{
    struct snapshot *s = calloc(1, sizeof(*s));
    s->pb = calloc(1, sizeof(*s->pb));

    /* pb fields */
    s->pb->sample_rate  = 44100;
    s->pb->period       = 100.0;
    s->pb->sample_count = 200;      /* must be >= ceil(period) = 100 */
    s->pb->waveform     = calloc(200, sizeof(float));
    s->pb->waveform_max = 1.0;
    s->pb->tic          = 50;
    s->pb->toc          = 150;
    s->pb->tic_pulse    = 0.5;
    s->pb->toc_pulse    = 0.5;
    /* all FFTW plan pointers remain NULL via calloc */

    /* snapshot fields — all within validated ranges */
    s->timestamp   = 1000000000ULL;
    s->nominal_sr  = 44100;
    s->bph         = 21600;           /* valid preset */
    s->la          = 52.0;            /* [MIN_LA=10, MAX_LA=90] */
    s->cal         = 0;               /* [MIN_CAL, MAX_CAL] */
    s->sample_rate = 44100.0;         /* > 0 */
    s->guessed_bph = 21600;           /* [MIN_BPH, MAX_BPH] */
    s->rate        = 0.0;             /* [-9999, 9999] */
    s->be          = 0.0;             /* [0, 99.9] */
    s->amp         = 0.0;             /* [0, 360] */
    s->trace_zoom  = 1.0;

    /* empty events and amplitude history */
    s->events_count = 0;
    s->amps_count   = 0;

    return s;
}

/*
 * Release a snapshot built by make_test_snapshot().
 * Calls free() directly instead of snapshot_destroy() because the pb was
 * not initialised via setup_buffers() and has no FFTW resources.
 */
static void free_test_snapshot(struct snapshot *s)
{
    if (s->pb) {
        free(s->pb->waveform);
        free(s->pb);
    }
    free(s->events);
    free(s->events_tictoc);
    free(s->amps);
    free(s->amps_time);
    free(s);
}

/* -----------------------------------------------------------------------
 * Tests
 * ----------------------------------------------------------------------- */

/*
 * write_file with cnt=0 must produce a file that read_file accepts.
 *
 * Regression test for issue #01: read_file currently returns an error for
 * files with an empty snapshot list even though write_file writes them as
 * structurally valid.
 */
static void test_empty_write_read(void)
{
    FILE *f = tmpfile();
    ASSERT(f != NULL, "tmpfile() failed");

    ASSERT(write_file(f, NULL, NULL, 0) == 0, "write_file cnt=0 failed");
    rewind(f);

    struct snapshot **s    = NULL;
    char            **names = NULL;
    uint64_t          cnt   = 99;   /* sentinel — must be overwritten */

    int r = read_file(f, &s, &names, &cnt);
    fclose(f);

    ASSERT(r   == 0, "read_file rejected a valid empty-snapshot file");
    ASSERT(cnt == 0, "expected cnt=0");

    free(s);
    free(names);
}

/*
 * A single snapshot with a name survives a write->read cycle with all
 * tracked fields intact.
 */
static void test_single_snapshot_roundtrip(void)
{
    struct snapshot *orig    = make_test_snapshot();
    struct snapshot *arr[1]  = { orig };
    char            *nms[1]  = { "hamilton" };

    FILE *f = tmpfile();
    ASSERT(f != NULL, "tmpfile() failed");
    ASSERT(write_file(f, arr, nms, 1) == 0, "write_file failed");
    rewind(f);

    struct snapshot **s_out   = NULL;
    char            **n_out   = NULL;
    uint64_t          cnt     = 0;

    ASSERT(read_file(f, &s_out, &n_out, &cnt) == 0, "read_file failed");
    fclose(f);
    ASSERT(cnt == 1, "expected cnt=1");

    struct snapshot *s = s_out[0];
    ASSERT(strcmp(n_out[0], "hamilton") == 0, "name mismatch");
    ASSERT(s->nominal_sr       == orig->nominal_sr,        "nominal_sr");
    ASSERT(s->bph              == orig->bph,               "bph");
    ASSERT(s->la               == orig->la,                "la");
    ASSERT(s->cal              == orig->cal,               "cal");
    ASSERT(s->timestamp        == orig->timestamp,         "timestamp");
    ASSERT(s->pb->period       == orig->pb->period,        "period");
    ASSERT(s->pb->sample_count == orig->pb->sample_count,  "sample_count");
    ASSERT(s->pb->tic          == orig->pb->tic,           "tic");
    ASSERT(s->pb->toc          == orig->pb->toc,           "toc");
    ASSERT(s->sample_rate      == orig->sample_rate,       "sample_rate");
    ASSERT(s->guessed_bph      == orig->guessed_bph,       "guessed_bph");
    ASSERT(s->rate             == orig->rate,              "rate");
    ASSERT(s->be               == orig->be,               "be");

    snapshot_destroy(s_out[0]);
    free(n_out[0]);
    free(s_out);
    free(n_out);
    free_test_snapshot(orig);
}

/*
 * Multiple snapshots survive a write->read cycle preserving order and
 * per-snapshot content.
 */
static void test_multiple_snapshots_roundtrip(void)
{
    enum { N = 3 };
    struct snapshot *snaps[N];
    char            *name_ptrs[N];
    char             names[N][32];

    for (int i = 0; i < N; i++) {
        snaps[i]            = make_test_snapshot();
        snaps[i]->bph       = 21600 + i * 3600;
        snaps[i]->timestamp = 1000000000ULL + (uint64_t)i * 1000;
        snprintf(names[i], sizeof(names[i]), "watch-%d", i);
        name_ptrs[i] = names[i];
    }

    FILE *f = tmpfile();
    ASSERT(f != NULL, "tmpfile() failed");
    ASSERT(write_file(f, snaps, name_ptrs, N) == 0, "write_file failed");
    rewind(f);

    struct snapshot **s_out = NULL;
    char            **n_out = NULL;
    uint64_t          cnt   = 0;

    ASSERT(read_file(f, &s_out, &n_out, &cnt) == 0, "read_file failed");
    fclose(f);
    ASSERT((int)cnt == N, "cnt mismatch");

    for (int i = 0; i < N; i++) {
        ASSERT(s_out[i]->bph       == snaps[i]->bph,       "bph");
        ASSERT(s_out[i]->timestamp == snaps[i]->timestamp,  "timestamp");
        ASSERT(strcmp(n_out[i], names[i]) == 0,             "name");
        snapshot_destroy(s_out[i]);
        free(n_out[i]);
    }
    free(s_out);
    free(n_out);

    for (int i = 0; i < N; i++)
        free_test_snapshot(snaps[i]);
}

/*
 * double values survive the serialize/deserialize cycle bit-for-bit.
 *
 * Regression test for issue #02: the Windows write_hex_double /
 * parse_hex_double path must preserve the exact IEEE 754 bit pattern.
 * Uses pb->tic_pulse as the carrier field (no range validation).
 */
static void test_double_roundtrip(void)
{
    static const struct { double val; const char *desc; } cases[] = {
        {  0.0,          "zero"       },
        {  1.0,          "one"        },
        { -1.5,          "neg frac"   },
        {  52.0,         "typical la" },
        {  0.5,          "half"       },
        {  M_PI,         "pi"         },
        {  1e-10,        "small pos"  },
        { -1e-10,        "small neg"  },
        {  44100.0,      "sample rate"},
        {  0.000976563,  "2^-10"      },
    };
    int n = (int)(sizeof(cases) / sizeof(cases[0]));

    for (int i = 0; i < n; i++) {
        struct snapshot *orig = make_test_snapshot();
        orig->pb->tic_pulse   = cases[i].val;

        FILE *f = tmpfile();
        ASSERT(f != NULL, "tmpfile() failed");

        struct snapshot *arr[1] = { orig };
        char            *nms[1] = { NULL };
        ASSERT(write_file(f, arr, nms, 1) == 0, "write_file failed");
        rewind(f);

        struct snapshot **s_out = NULL;
        char            **n_out = NULL;
        uint64_t          cnt   = 0;
        int r = read_file(f, &s_out, &n_out, &cnt);
        fclose(f);
        free_test_snapshot(orig);

        ASSERT(r == 0 && cnt == 1, "read_file failed");

        uint64_t want, got;
        memcpy(&want, &cases[i].val,            sizeof(want));
        memcpy(&got,  &s_out[0]->pb->tic_pulse, sizeof(got));

        snapshot_destroy(s_out[0]);
        free(n_out[0]);
        free(s_out);
        free(n_out);

        char msg[80];
        snprintf(msg, sizeof(msg), "double mismatch: %s (%.17g)", cases[i].desc, cases[i].val);
        ASSERT(want == got, msg);
    }
}

/* read_file on an empty file must return a non-zero error code. */
static void test_read_empty_file(void)
{
    FILE *f = tmpfile();
    ASSERT(f != NULL, "tmpfile() failed");

    struct snapshot **s    = NULL;
    char            **names = NULL;
    uint64_t          cnt   = 0;
    int r = read_file(f, &s, &names, &cnt);
    fclose(f);

    ASSERT(r != 0,    "expected error on empty file");
    ASSERT(cnt == 0,  "expected cnt=0 on error");
    ASSERT(s == NULL, "expected NULL snapshots on error");
}

/*
 * read_file on a structurally valid header that is truncated before the
 * snapshot body must return a non-zero error code without crashing.
 */
static void test_read_truncated_file(void)
{
    FILE *f = tmpfile();
    ASSERT(f != NULL, "tmpfile() failed");

    /* Valid header + snapshot-list array of length 1, but no snapshot body */
    fprintf(f,
        "Ltg-timer-version;\n"
        "S5;0.8.0;\n"
        "Ldata;\n"
        "T;\n"
        "Lsnapshot-list;\n"
        "A1;\n");   /* claims 1 snapshot but provides none */
    rewind(f);

    struct snapshot **s    = NULL;
    char            **names = NULL;
    uint64_t          cnt   = 0;
    int r = read_file(f, &s, &names, &cnt);
    fclose(f);

    ASSERT(r != 0,    "expected error on truncated file");
    ASSERT(cnt == 0,  "expected cnt=0 on error");
    ASSERT(s == NULL, "expected NULL snapshots on error");
}

/*
 * A snapshot written with a NULL name is read back with a NULL name.
 */
static void test_unnamed_snapshot(void)
{
    struct snapshot *orig   = make_test_snapshot();
    struct snapshot *arr[1] = { orig };
    char            *nms[1] = { NULL };

    FILE *f = tmpfile();
    ASSERT(f != NULL, "tmpfile() failed");
    ASSERT(write_file(f, arr, nms, 1) == 0, "write_file failed");
    rewind(f);

    struct snapshot **s_out = NULL;
    char            **n_out = NULL;
    uint64_t          cnt   = 0;
    ASSERT(read_file(f, &s_out, &n_out, &cnt) == 0, "read_file failed");
    fclose(f);
    ASSERT(cnt == 1, "expected cnt=1");

    ASSERT(n_out[0] == NULL, "expected NULL name for unnamed snapshot");

    snapshot_destroy(s_out[0]);
    free(n_out[0]);   /* free(NULL) is safe */
    free(s_out);
    free(n_out);
    free_test_snapshot(orig);
}

/* -----------------------------------------------------------------------
 * computer.c tests — snapshot lifecycle and compute_results
 *
 * These exercise logic in computer.c without touching audio or threads.
 * ----------------------------------------------------------------------- */

/*
 * snapshot_clone produces an independent copy; snapshot_destroy on the clone
 * must not corrupt the original, and free_test_snapshot on the original must
 * not double-free anything.
 *
 * Also exercises pb_clone / pb_destroy_clone (algo.c).
 */
static void test_snapshot_clone_destroy(void)
{
    struct snapshot *orig  = make_test_snapshot();
    orig->bph       = 21600;
    orig->timestamp = 999888777ULL;

    struct snapshot *clone = snapshot_clone(orig);
    ASSERT(clone != NULL, "snapshot_clone returned NULL");
    ASSERT(clone != orig, "clone is same pointer as orig");
    ASSERT(clone->pb != NULL, "clone->pb is NULL");
    ASSERT(clone->pb != orig->pb, "clone->pb aliases orig->pb");

    /* field values survive the clone */
    ASSERT(clone->bph       == orig->bph,       "bph");
    ASSERT(clone->timestamp == orig->timestamp,  "timestamp");
    ASSERT(clone->la        == orig->la,         "la");
    ASSERT(clone->nominal_sr == orig->nominal_sr, "nominal_sr");

    /* pb fields: sample_count set by pb_clone from ceil(period) */
    ASSERT(clone->pb->sample_count == (int)ceil(orig->pb->period), "pb sample_count");
    ASSERT(clone->pb->period       == orig->pb->period,            "pb period");

    /* waveform is a distinct allocation with matching content */
    ASSERT(clone->pb->waveform != NULL,             "clone waveform allocated");
    ASSERT(clone->pb->waveform != orig->pb->waveform, "clone waveform is copy");

    snapshot_destroy(clone);    /* frees pb via pb_destroy_clone */
    free_test_snapshot(orig);   /* frees pb directly */
}

/*
 * compute_results with pb=NULL: sample_rate and guessed_bph are set correctly.
 *
 * Regression: calloc fix for issue #03 ensures the cleared snapshot fields
 * don't cause UB if allocation fails mid-way through start_computer.
 */
static void test_compute_results_no_pb(void)
{
    struct snapshot s = {0};
    s.nominal_sr = 44100;
    s.cal        = 0;
    s.bph        = 21600;
    s.la         = 52.0;
    s.pb         = NULL;

    compute_results(&s);

    ASSERT(s.sample_rate  == 44100.0, "sample_rate with cal=0");
    ASSERT(s.guessed_bph  == 21600,   "guessed_bph equals bph when bph set");

    /* with bph=0 and no pb, guessed_bph should fall back to DEFAULT_BPH */
    s.bph = 0;
    compute_results(&s);
    ASSERT(s.guessed_bph == DEFAULT_BPH, "guessed_bph falls back to DEFAULT_BPH");
}

/*
 * compute_results with a pb whose period matches bph exactly: rate == 0.
 *
 * For 21600 bph at 44100 Hz the perfect period is
 *   7200 * sample_rate / bph  =  7200 * 44100 / 21600  =  14700 samples.
 */
static void test_compute_results_perfect_rate(void)
{
    struct processing_buffers pb = {0};
    pb.period = 14700.0;   /* perfect period for 21600 bph at 44100 Hz */
    pb.be     = 0.0;
    pb.amp    = 0.0;

    struct snapshot s = {0};
    s.pb         = &pb;
    s.nominal_sr = 44100;
    s.cal        = 0;
    s.bph        = 21600;
    s.la         = 52.0;

    compute_results(&s);

    ASSERT(s.sample_rate == 44100.0, "sample_rate with cal=0");
    ASSERT(s.guessed_bph == 21600,   "guessed_bph");
    ASSERT(fabs(s.rate)  < 1e-6,     "rate is 0 for perfect period");
    ASSERT(s.be          == 0.0,     "be is 0");
    ASSERT(s.amp         == 0.0,     "amp=0 because pb->amp=0 (out of [135,360])");
}

/*
 * compute_results: sample_rate adjusts proportionally to the cal field.
 *
 * cal=864 means +86.4 s/d correction factor = +0.1 % gain.
 * Expected sample_rate = 44100 * (1 + 864 / (10 * 86400))
 *                      = 44100 * 1.001 = 44144.1 Hz.
 */
static void test_compute_results_calibration(void)
{
    struct snapshot s = {0};
    s.nominal_sr = 44100;
    s.cal        = 864;
    s.bph        = 21600;
    s.la         = 52.0;
    s.pb         = NULL;

    compute_results(&s);

    double expected_sr = 44100.0 * (1.0 + 864.0 / (10.0 * 3600 * 24));
    ASSERT(fabs(s.sample_rate - expected_sr) < 1e-6,
           "sample_rate scales with positive cal");

    /* negative cal: watch is losing time */
    s.cal = -864;
    compute_results(&s);

    expected_sr = 44100.0 * (1.0 + (-864.0) / (10.0 * 3600 * 24));
    ASSERT(fabs(s.sample_rate - expected_sr) < 1e-6,
           "sample_rate scales with negative cal");
}

/* -----------------------------------------------------------------------
 * main
 * ----------------------------------------------------------------------- */

int main(void)
{
    fprintf(stdout, "=== tg-timer serializer tests ===\n");

    RUN_TEST(test_empty_write_read);
    RUN_TEST(test_single_snapshot_roundtrip);
    RUN_TEST(test_multiple_snapshots_roundtrip);
    RUN_TEST(test_double_roundtrip);
    RUN_TEST(test_read_empty_file);
    RUN_TEST(test_read_truncated_file);
    RUN_TEST(test_unnamed_snapshot);
    RUN_TEST(test_snapshot_clone_destroy);
    RUN_TEST(test_compute_results_no_pb);
    RUN_TEST(test_compute_results_perfect_rate);
    RUN_TEST(test_compute_results_calibration);

    fprintf(stdout, "\n%d/%d tests passed\n",
            tests_run - tests_failed, tests_run);
    return tests_failed ? 1 : 0;
}
