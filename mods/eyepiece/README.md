# Eyepiece (eyepiece)

Use the DJI Goggles 3 as a PC display over the USB cable, with head tracking
sent back to the PC. No drone and no Wi-Fi are needed.

```
PC ──adb forward tcp:42420──► eyepiece_bridge ──► VC8000D decode ──► VIDEO0 plane ──► display
PC ◄──── head pose / frame stats ──── eyepiece_bridge ◄── MCU head attitude (100 Hz)
```

The mod adds an **Eyepiece** entry to the More menu. It starts
`/vendor/natartos/apps/eyepiece/eyepiece_bridge -b`. The bridge receives
H.264/HEVC from the PC over an adb-forwarded TCP socket, decodes it in
hardware, shows it on the display, and streams the head orientation (yaw,
pitch, roll) back on the same connection. The bridge reads head tracking from
the goggles' MCU attitude stream itself. The mod hooks nothing in
`dji_glasses`.

Status: tested on firmware 01.00.1600 (menu launch, clean Back exit, hardware
decode, head tracking).

## Requirements

- The **overlay** mod. The bridge uses its `fb_shared` injection for the
  waiting screen and to make the GUI transparent over the video.
- Root ADB on the goggles, as set up by [toolya](https://toolya.dev)'s root
  session (`adb forward` carries the stream).
- The PC side, **eyepiece-pc**. It is a separate tool (not published yet) that
  captures and encodes the desktop, speaks the protocol below, and can forward
  the head pose to opentrack (UDP 4242). The toolya web UI also has an
  "Eyepiece stream" action.

## Usage

1. On the goggles: **More → Eyepiece**. A waiting screen is shown until a PC
   connects.
2. On the PC: `adb forward tcp:42420 tcp:42420`, then start eyepiece-pc.
3. **Back** (a short click) exits the bridge and returns to the goggles UI.
   **Long-hold Back** (≥ 1.5 s, the overlay's app exit) also stops it.

A PC disconnect does not end a menu-launched session. The waiting screen comes
back and another client can connect. Log: `/data/local/tmp/eyepiece.log`.

You can also start the bridge by hand from `adb shell`
(`/vendor/natartos/apps/eyepiece/eyepiece_bridge`, add `-b` for the Back exit).

## Wire protocol

There is one TCP connection on 127.0.0.1:42420. Video goes PC → goggles, and
pose and stats come back on the same socket. Every message is an 8-byte
little-endian header (`'E' 'P' type flags u32 len`) followed by the payload.
Protocol version 1.

| Dir | Type | Payload |
|-----|------|---------|
| PC → goggles | `0x01` VIDEO | pts, codec + Annex-B access unit (flags bit0 = keyframe) |
| PC → goggles | `0x02` PING | PC timestamp |
| PC → goggles | `0x03` CTRL | op: 1 = recenter, 2 = quit |
| PC → goggles | `0x04` STREAM | width, height, codec, fps (sent before video) |
| goggles → PC | `0x81` POSE | yaw/pitch/roll in degrees, device timestamp |
| goggles → PC | `0x82` PONG | PC + device timestamps |
| goggles → PC | `0x83` LOG | text |
| goggles → PC | `0x84` FRAME | per-frame timing / queue / drop stats |
| goggles → PC | `0x85` INFO | version, framebuffer size, decoder, head-tracking state |

[bridge/proto.h](bridge/proto.h) is the authoritative definition.

## Safety

- To get head tracking, the bridge starts the goggles' MCU attitude stream
  (bus command `0x15/0x31` to the MCU). It sends only two fixed payloads,
  start and stop. It stops the stream on every exit path: normal exit, the
  bridge dying (the helper process gets a parent-death signal) and the helper
  dying (the bridge runs a stopper).
- Recovery: if the stream is ever left running, run
  `/vendor/natartos/apps/eyepiece/eyepiece_bridge -Z`. It sends only the stop
  and exits.
- At runtime the bridge writes only `/data/local/tmp/{fb_shared,fb_override,eyepiece.log}`.
  It never writes `/system`, `/vendor` or any partition. If it crashes, it
  removes `fb_override` so the stock UI comes back.
- SELinux must be permissive, as set by the toolya root session. The bridge
  has not been tested with SELinux enforcing.

## Bridge options

```
-p PORT      TCP port on 127.0.0.1 (default 42420)
-A           listen on all interfaces instead of loopback
-s WxH       output framebuffer size (default 1920x1080)
-t N / -c N  decoder slice threads / colour-convert threads (default 4)
-f           fast decode (skip loop filter, software decoder)
-V           no video (head tracking only)
-H           no head tracking
-Z           only send the MCU head-attitude stop, then exit (recovery)
-b           exit on Back key (menu launches)
-1           exit after the first client disconnects
-D sw|hw     decoder: hw = VC8000D (default), sw = device libavcodec
-F N         hw: number of frame buffers (default 12)
-S DIR       hw: cache sync direction before reading frames
-O plane[:M] zero-copy scan-out on the VIDEO0 plane of VOP mask M (default plane:3)
-O fb        convert into fb_shared instead (overlay path)
-T           with -O plane: keep the GUI visible over the video
-M           log display timing every 3 s
-Q VFP       experimental: display qsync (variable refresh)
-L R,F,V     hw: decoder start-config knobs
-N           null decoder (protocol/latency test)
-l FILE      log file (default /data/local/tmp/eyepiece.log, '-' = none)
```

## Build

The bridge is an Android (bionic) binary built with the Android NDK (r25+,
API 28):

```sh
mods/eyepiece/bridge/build.sh [OUTDIR]       # NDK: $ANDROID_NDK_HOME or newest ~/Android/Sdk/ndk/*
mods/eyepiece/bridge/build.sh host [OUTDIR]  # x86-64 Linux build for protocol tests on the PC
```

`scripts/build.sh` builds it into `dist/apps/eyepiece/eyepiece_bridge` when an
NDK is found and skips it with a message otherwise. CI builds it with the NDK
preinstalled on the runner and ships it as a release asset.
