# GNUmakefile — Windows/MSYS2 build for tg-timer
# Replaces the broken autotools Makefile on this host.
# Requires: MSYS2 + mingw-w64-x86_64 toolchain
#
# Targets:
#   make              → tg-timer.exe
#   make check        → build and run test-serializer
#   make clean        → remove object files and executables
#   make distclean    → also remove this generated state

SHELL   := /c/msys64/usr/bin/bash
export PATH := /c/msys64/usr/bin:/c/msys64/mingw64/bin:$(PATH)
export TEMP := /c/Users/arjun/AppData/Local/Temp
export TMP  := /c/Users/arjun/AppData/Local/Temp

MH      := /c/msys64/mingw64

CC      := gcc
WINDRES := windres

VERSION  := 0.8.0
PACKAGE  := tg-timer

DEFS := \
  -DPACKAGE_NAME=\"Tg\" \
  -DPACKAGE_TARNAME=\"$(PACKAGE)\" \
  -DPACKAGE_VERSION=\"$(VERSION)\" \
  -DPACKAGE=\"$(PACKAGE)\" \
  -DPROGRAM_NAME=\"Tg\" \
  -DVERSION=\"$(VERSION)\" \
  -DHAVE_LIBPTHREAD=1 \
  -DHAVE_LIBM=1 \
  -DLIBDEFLATE_DLL

INCS := \
  -I. \
  -I$(MH)/include/gtk-3.0 \
  -I$(MH)/include/pango-1.0 \
  -I$(MH)/include/harfbuzz \
  -I$(MH)/include/cairo \
  -I$(MH)/include/freetype2 \
  -I$(MH)/include/pixman-1 \
  -I$(MH)/include/gdk-pixbuf-2.0 \
  -I$(MH)/include/libpng16 \
  -I$(MH)/include/webp \
  -I$(MH)/include/atk-1.0 \
  -I$(MH)/include/fribidi \
  -I$(MH)/include/glib-2.0 \
  -I$(MH)/lib/glib-2.0/include

CFLAGS  := $(DEFS) $(INCS) -g -O2 -Wall -Wextra

LIBDIRS := -L$(MH)/lib

GTK_LIBS := \
  -lgtk-3 -lgdk-3 -lgdi32 -limm32 -lshell32 -lole32 \
  -Wl,-luuid -lwinmm -ldwmapi -lsetupapi -lcfgmgr32 \
  -lpangowin32-1.0 -lpangocairo-1.0 -lpango-1.0 -latk-1.0 \
  -lcairo-gobject -lcairo -lgdk_pixbuf-2.0 \
  -lgio-2.0 -lgobject-2.0 -lglib-2.0 -lintl

LIBS := $(LIBDIRS) $(GTK_LIBS) -lgthread-2.0 -lportaudio -lfftw3f -lpthread -lm

# ── main executable ──────────────────────────────────────────────────────────

SRCS := \
  src/algo.c \
  src/audio.c \
  src/computer.c \
  src/config.c \
  src/interface.c \
  src/output_panel.c \
  src/serializer.c

OBJS := $(SRCS:.c=.o)
RC_OBJ := icons/tg-timer.o

all: tg-timer.exe

tg-timer.exe: $(OBJS) $(RC_OBJ)
	$(CC) -mwindows -o $@ $^ $(LIBS)

# windres is broken on this host (popen of gcc fails); fall back to an
# empty object so the link still succeeds, just without the embedded icon.
$(RC_OBJ): icons/tg-timer.rc
	$(WINDRES) $< -O coff -o $@ || \
	{ echo "windres failed -- building without embedded icon"; \
	  $(CC) -x c -c /dev/null -o $@; }

%.o: %.c src/tg.h
	$(CC) $(CFLAGS) -c -o $@ $<

# ── test suite ───────────────────────────────────────────────────────────────

TEST_SRCS := \
  tests/test_serializer.c \
  src/serializer.c \
  src/computer.c \
  src/algo.c

TEST_OBJS := $(TEST_SRCS:.c=.o)

TEST_LIBS := $(LIBDIRS) $(GTK_LIBS) -lgthread-2.0 -lfftw3f -lpthread -lm

test-serializer.exe: $(TEST_OBJS)
	$(CC) -o $@ $^ $(TEST_LIBS)

check: test-serializer.exe
	./test-serializer.exe

# ── housekeeping ─────────────────────────────────────────────────────────────

clean:
	rm -f $(OBJS) $(RC_OBJ) $(TEST_OBJS) tg-timer.exe test-serializer.exe

distclean: clean

.PHONY: all check clean distclean
