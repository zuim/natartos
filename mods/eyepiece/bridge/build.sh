#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-or-later
# Build eyepiece_bridge for the goggles (aarch64, bionic, API 28 = Android 9).
#
#   ./build.sh [OUTDIR]        Android NDK build (default OUTDIR: ./out)
#   ./build.sh host [OUTDIR]   x86-64 Linux build for protocol tests on the PC
#                              (eyepiece_bridge_host; device features fail soft)
#
# Needs the Android NDK (r25+): $ANDROID_NDK_HOME, else the newest
# ~/Android/Sdk/ndk/*. Exits with status 3 when no NDK is found.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRCS="eyepiece_bridge.c swdec.c hwdec.c dispout.c yuv2fb.c fbout.c fbtext.c headtrack.c rndis.c"
CFLAGS="-O2 -Wall -Wextra -Wno-unused-parameter -std=gnu11"

MODE=ndk
if [ "${1:-}" = "host" ]; then MODE=host; shift; fi
OUT="${1:-$HERE/out}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
cd "$HERE"

if [ "$MODE" = host ]; then
  gcc $CFLAGS -g -o "$OUT/eyepiece_bridge_host" $SRCS -ldl -lm -lpthread
  echo "built $OUT/eyepiece_bridge_host"
  exit 0
fi

NDK="${ANDROID_NDK_HOME:-}"
if [ -z "$NDK" ]; then
  NDK=$(ls -d "$HOME"/Android/Sdk/ndk/*/ 2>/dev/null | sort -V | tail -1)
fi
CC="${NDK%/}/toolchains/llvm/prebuilt/linux-x86_64/bin/aarch64-linux-android28-clang"
if [ -z "$NDK" ] || [ ! -x "$CC" ]; then
  echo "Android NDK clang not found (set ANDROID_NDK_HOME)" >&2
  exit 3
fi

"$CC" $CFLAGS -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -o "$OUT/eyepiece_bridge" $SRCS -ldl -lm
"${CC%/*}/llvm-strip" "$OUT/eyepiece_bridge"
chmod 755 "$OUT/eyepiece_bridge"
echo "built $OUT/eyepiece_bridge ($(stat -c%s "$OUT/eyepiece_bridge") bytes)"
