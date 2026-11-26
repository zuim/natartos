# Eyepiece (`zuim_eyepiece`)

Use the DJI Goggles 3 as a PC display over the USB cable, with head tracking sent back to the PC. The mod
adds **More → Eyepiece**, which starts `eyepiece_bridge` (`bridge/`), and hooks the DJI GUI so the stream
shows through. The PC side is [eyepiece-pc](https://github.com/zuim/eyepiece-pc).

The documentation lives with the PC tool, because the wire protocol and the schema it is built from belong
to that repository:

- [Goggles mod and bridge](https://github.com/zuim/eyepiece-pc/blob/main/docs/goggles/bridge.md): starting,
  display modes, bridge options, safety, internals, build and deploy
- [Bridge protocol](https://github.com/zuim/eyepiece-pc/blob/main/docs/protocol/bridge.md)
- [Display composition](https://github.com/zuim/eyepiece-pc/blob/main/docs/goggles/display-composition.md)

Building the bridge needs the matching `contract/eyepiece-rpc.proto` from eyepiece-pc in `EYEPIECE_PROTO`;
see the build section of the bridge page.

Status: tested on firmware 01.00.1600. Experimental; never test while flying.
