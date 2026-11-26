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
- At runtime the bridge writes only `/data/local/tmp/{fb_gui_hide,eyepiece.lock,eyepiece.log}`
  (with an overlay that predates `fb_gui_hide`: `fb_shared` and `fb_override` instead).
  It never writes `/system`, `/vendor` or any partition. While it holds a lock on
  `fb_gui_hide` the overlay hides the GUI plane; if the bridge crashes, the lock is
  released and the overlay shows the stock UI again.
- SELinux must be permissive, as set by the toolya root session. The bridge
  has not been tested with SELinux enforcing.

## Bridge options

Experimental work in progress: `-B` adds a separately named `ffs.eyepiece` vendor
interface without reusing DJI bulk services. TCP listeners remain available.
The gadget switch temporarily disconnects USB; normal shutdown attempts to restore
the previous binding, and startup checks a recovery snapshot in
`/data/local/tmp/eyepiece.bulk-state`. Endpoint pumps use bounded 64 KiB staging buffers
and blocking FunctionFS endpoint reads/writes; endpoint files do not implement `.poll`.
The socketpair uses nonblocking polling. A stop flag, SIGUSR1 wakeups, and thread joins
bound teardown. The Android bridge builds with NDK r30 for API 28; on Apple Silicon, the
Nix-provided x86_64 host tools run under Rosetta.

This diagnostic build remains experimental. Descriptor binding was tested alongside
RNDIS/ADB and ACM/ADB at High-Speed on a Mac. After the blocking endpoint-I/O fix, no-video
bulk PING/PONG passed at about 0.4 ms RTT; one 118,438-byte OUT stream and sustained tests
through a host-observed 8.5 Mbps passed without USB errors or dropped frames. Each test
restored ACM + ADB and cleared the recovery snapshot. A three-frame 1080p test at 1 fps
and 500 kbps decoded on the VC8000D and pushed a frame to VOP0 plane 4. A 30-second
1080p/20 Mbps test-pattern soak then ran at about 20 fps on the prior build: 600 frames
pushed, zero display errors or drops, and 72,922,424 USB bytes relayed across 1,252 OUT
reads. The bounded-write update (500 ms internal socketpair send deadline, no routine
per-second rate logs) is now installed. A three-frame visible smoke test passed. A
10-second 1080p/20 Mbps run then relayed 24,133,160 bytes across 424 OUT reads at about
20 fps with zero drops; routine rate logs were absent. Cleanup restored ACM + ADB and
cleared the snapshot. The 30-second soak has not been repeated on this update; forced IN
backpressure and real desktop capture remain untested. `/vendor` is read-only; `/system`
was not touched. Do not treat bulk as ready for normal use. A power-cycle returns the
runtime gadget configuration to stock if USB recovery fails.

Latency A/B measured x264 `ultrafast` about 5 ms lower in steady capture-to-shown p50 than
`superfast` at 1080p/20 fps and roughly 19.5 Mbps, with zero drops. The preset has not been
made the default because decoded image quality has not been compared.

```text
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
