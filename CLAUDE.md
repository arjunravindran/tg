# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build (Windows / MSYS2)

The autotools build is broken on this host. Use `GNUmakefile` exclusively.

```sh
make -j4              # build tg-timer.exe
make check            # build and run tests/test_serializer.c
make clean            # remove objects and executables
```

`GNUmakefile` hardcodes the MSYS2 toolchain at `/c/msys64/mingw64`. Do not use `./autogen.sh` or `./configure` on this machine.

Debug build: add `-DDEBUG` to `CFLAGS` in `GNUmakefile`; this enables `debug()` macro output and an extra waveform overlay in the UI.

## Architecture

The app is a GTK3 desktop application. Data flows: **audio → algo → computer → output_panel/interface**.

### src/algo.c
Core DSP. Operates on `struct processing_buffers`. Key pipeline:
- `process()` — top-level entry point; calls the chain below
- `compute_period()` — FFT-based BPH detection; Classic uses simple harmonic mean, Improved uses k²-weighted mean + FWHM sigma
- `compute_phase()` / `compute_waveform()` — locate tic/toc impulses; Improved does sub-sample linear interpolation, Classic rounds to integer
- `compute_amplitude()` — pulse envelope via `smooth_classic()` (asymmetric EMA) in both modes; beat error from threshold crossing times
- `compute_cal()` — calibration fit; Improved adds two-pass outlier rejection

`processing_buffers.algo_classic` (0 = Improved, 1 = Classic) gates all algorithm branches.

### src/audio.c
PortAudio callback fills the global ring buffer `pa_buffers[PA_BUFF_SIZE]`. Light mode (half sample rate) applies a 5-tap FIR anti-alias filter in the callback. `analyze_pa_data()` calls `process()` then signals the computer thread.

### src/computer.c
Dedicated pthread. Owns `struct computer` which holds `actv`/`curr` snapshot pair and EMA state (`amp_history`, `rate_history`, `be_history`). After each `process()` call, the thread applies EMA smoothing (α ≈ 0.4 for beat error) before publishing to `actv`. Resets all EMA state on algorithm change, clear-trace, or warm-up detection. `snapshot_clone()` deep-copies a snapshot for the UI thread.

### src/output_panel.c
GTK drawing area callbacks render all visualisations using Cairo:
- `paperstrip_draw_event()` — classic paper-strip timegrapher view
- `balance_wheel_draw_event()` — balance-wheel-centric view (newer); geometry uses `scale = size/2.70` to keep degree labels within bounds; coordinate macros `PX(r,d)` / `PY(r,d)` are defined after the early-return guard blocks
- `handle_view_toggle()` — switches visibility between `classic_panel_box` and `balance_wheel_area`

### src/interface.c
GTK application shell. Builds the toolbar (BPH combo, audio device, sample rate, lift angle, calibration, algorithm selector). `recompute()` kills and restarts the computer thread when settings change; it compares `w->algo_classic` vs `w->computer->algo_classic` to detect algorithm switches.

### src/config.c
Reads/writes `tg-timer.ini` via GKeyFile. Fields are declared with the `CONFIG_FIELDS(OP)` macro in `tg.h`.

### src/serializer.c
Binary file format for saving/loading snapshots. Tested by `tests/test_serializer.c`.

## Key Data Structures (src/tg.h)

- `struct processing_buffers` — all DSP state for one analysis frame, including FFTW plans and result fields (`period`, `be`, `amp`, `tic_pulse`, `toc_pulse`)
- `struct snapshot` — immutable point-in-time view published to the UI; contains a cloned `processing_buffers` plus amplitude history ring buffers and calibration results
- `struct computer` — threading state; bridges audio thread and UI thread
- `struct output_panel` — per-tab widget tree; `wheel_view` int selects which visualisation is visible

## Algorithm Selector

`algo_classic = 0` → Improved (default); `algo_classic = 1` → Classic (pre-0.8).  
The flag is mirrored on `processing_buffers`, `calibration_data`, `processing_data`, `computer`, and `main_window`. Changing it triggers a full computer restart.

## Windows-Specific Notes

- All runtime DLLs (libgtk, libcairo, libportaudio, etc.) live in the repo root alongside `tg-timer.exe`
- `TEMP`/`TMP` env vars must be set to a Windows-style path (done in `GNUmakefile`) or FFTW wisdom file I/O fails
- `#ifdef __CYGWIN__ #define _WIN32 #endif` in `tg.h` — Cygwin builds advertise as Win32
