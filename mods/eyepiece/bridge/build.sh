#!/bin/bash
# SPDX-License-Identifier: AGPL-3.0-or-later
# Build eyepiece_bridge for the goggles (aarch64, bionic, API 28 = Android 9).
#
#   ./build.sh [OUTDIR]        Android NDK bridge build (default OUTDIR: ./out)
#   ./build.sh host [OUTDIR]   x86-64 Linux build for protocol tests on the PC
#                              (eyepiece_bridge_host; device features fail soft)
#   ./build.sh alpha-test [OUTDIR]  Android NDK build of the GFX1 alpha probe
#
# Needs the canonical schema in $EYEPIECE_PROTO, protoc, protoc-gen-nanopb,
# the nanopb source in $NANOPB_DIR,
# and the Android NDK (r25+): $ANDROID_NDK_HOME, else the newest
# ~/Android/Sdk/ndk/*. Exits with status 3 when no NDK is found.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
SRCS="eyepiece_bridge.c wire.c hwdec.c dispout.c yuv2bgra.c headtrack.c usb_bulk.c"
CFLAGS="-O2 -Wall -Wextra -Wno-unused-parameter -std=gnu11"

MODE=ndk
if [ "${1:-}" = "host" ]; then MODE=host; shift; fi
if [ "${1:-}" = "alpha-test" ]; then MODE=alpha-test; shift; fi
OUT="${1:-$HERE/out}"
mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
cd "$HERE"

prepare_protocol() {
  if [ -z "${NANOPB_DIR:-}" ] || [ ! -f "$NANOPB_DIR/pb_encode.c" ]; then
    echo "nanopb source not found (set NANOPB_DIR)" >&2
    exit 1
  fi
  case "${EYEPIECE_PROTO:-}" in
    /*) ;;
    *) echo "set EYEPIECE_PROTO to the absolute path of eyepiece-rpc.proto" >&2; exit 1 ;;
  esac
  if [ ! -f "$EYEPIECE_PROTO" ]; then
    echo "canonical schema not found: $EYEPIECE_PROTO" >&2
    exit 1
  fi
  if [ "$(basename "$EYEPIECE_PROTO")" != "eyepiece-rpc.proto" ]; then
    echo "schema input must be named eyepiece-rpc.proto" >&2
    exit 1
  fi
  EYEPIECE_PROTO="$(cd "$(dirname "$EYEPIECE_PROTO")" && pwd)/$(basename "$EYEPIECE_PROTO")"
  PROTO_DIR="$(mktemp -d "${TMPDIR:-/tmp}/eyepiece-proto.XXXXXXXX")"
  trap 'rm -f "$PROTO_DIR/eyepiece-rpc.pb.c" "$PROTO_DIR/eyepiece-rpc.pb.h"; rmdir "$PROTO_DIR"' EXIT
  PYTHONPATH="${NANOPB_PYTHONPATH:-}${PYTHONPATH:+:$PYTHONPATH}" \
    protoc -I "$(dirname "$EYEPIECE_PROTO")" --nanopb_out="-f$HERE/nanopb.options:$PROTO_DIR" "$EYEPIECE_PROTO"
  SRCS="$SRCS $PROTO_DIR/eyepiece-rpc.pb.c $NANOPB_DIR/pb_common.c $NANOPB_DIR/pb_encode.c $NANOPB_DIR/pb_decode.c"
  CFLAGS="$CFLAGS -I$PROTO_DIR -I$NANOPB_DIR"
}

if [ "$MODE" = host ]; then
  prepare_protocol
  HOST_CC="${HOST_CC:-gcc}"
  "$HOST_CC" $CFLAGS -g -o "$OUT/eyepiece_bridge_host" $SRCS -ldl -lm -lpthread
  echo "built $OUT/eyepiece_bridge_host"
  exit 0
fi

NDK="${ANDROID_NDK_HOME:-}"
if [ -z "$NDK" ]; then
  NDK=$(ls -d "$HOME"/Android/Sdk/ndk/*/ 2>/dev/null | sort -V | tail -1)
fi
case "$(uname -s)-$(uname -m)" in
  Darwin-arm64|Darwin-aarch64)
    NDK_HOST="darwin-arm64"
    if [ -n "$NDK" ] && [ ! -x "${NDK%/}/toolchains/llvm/prebuilt/$NDK_HOST/bin/aarch64-linux-android28-clang" ]; then
      NDK_HOST="darwin-x86_64"
    fi
    ;;
  Darwin-x86_64) NDK_HOST="darwin-x86_64" ;;
  Linux-x86_64) NDK_HOST="linux-x86_64" ;;
  Linux-aarch64|Linux-arm64) NDK_HOST="linux-aarch64" ;;
  *) echo "unsupported NDK host: $(uname -s)-$(uname -m)" >&2; exit 3 ;;
esac
CC="${NDK%/}/toolchains/llvm/prebuilt/$NDK_HOST/bin/aarch64-linux-android28-clang"
if [ -z "$NDK" ] || [ ! -x "$CC" ]; then
  echo "Android NDK clang not found (set ANDROID_NDK_HOME)" >&2
  exit 3
fi

if [ "$MODE" = "alpha-test" ]; then
  "$CC" $CFLAGS -o "$OUT/gfx1_alpha_test" gfx1_alpha_test.c -ldl
  "${CC%/*}/llvm-strip" "$OUT/gfx1_alpha_test"
  chmod 755 "$OUT/gfx1_alpha_test"
  echo "built $OUT/gfx1_alpha_test ($(wc -c < "$OUT/gfx1_alpha_test") bytes)"
  exit 0
fi

prepare_protocol
"$CC" $CFLAGS -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -o "$OUT/eyepiece_bridge" $SRCS -ldl -lm
"${CC%/*}/llvm-strip" "$OUT/eyepiece_bridge"
chmod 755 "$OUT/eyepiece_bridge"
echo "built $OUT/eyepiece_bridge ($(stat -c%s "$OUT/eyepiece_bridge") bytes)"
