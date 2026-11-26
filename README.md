# natartos

A per-session mod framework for your goggles, built on top of
[toolya](https://github.com/grom-labs/toolya)'s root.

**Verified devices:** DJI Goggles 3.

> **Status: v0.1.0 — per-session only.** Mods work but do not survive a reboot.
> After every power cycle you re-enable ADB (`toolya adb --yes`) and restart
> mods (`toolya natartos start`). Reboot persistence is on the
> [roadmap](#roadmap). See [If you bricked it](#if-you-bricked-it) for why.

## What it does

natartos installs a native loader (`LD_PRELOAD`) and opt-in mods into
`/vendor` on your goggles. The loader hooks `dji_glasses` at runtime, loading
your chosen mods. Mods are per-session — they activate when you start and
last until the goggles reboot or power off.

## Safety

- **Crash guard** — if `dji_glasses` crashes 5 times within 120 seconds, the
  inject script gives up and the process restarts stock. No reboot needed.
- **Kill-switch** — `toolya natartos safe on` disables mods for the current
  session.
- **No bootloader / system writes** — only `/vendor` (remounted rw) and `/data`
  are touched. `/system` is never modified.

See [SECURITY.md](SECURITY.md) for the trust model.

## Mods

| Id | Name | Type | Default | Status |
|----|------|------|---------|--------|
| `overlay` | Display Overlay | hook | yes | tested |
| `popen` | Popen Redirect | hook | yes | tested |
| `menu` | Menu Framework | preload | yes | tested |
| `doom` | Doom | app | yes | tested |
| `stream` | PC Stream | app | yes | tested |
| `background` | Custom Background | theme | no | tested |
| `zuim_eyepiece` | Eyepiece (PC display + head tracking over USB) | app | no | tested on fw 01.00.1600; requires `overlay` |
| `audio` | Custom Audio | theme | no | untested |
| `mute` | Audio Control | tweak | no | untested |
| `input` | Key Injection | hook | no | experimental |
| `splash` | Custom Splash | theme | no | in progress |

Mods are opt-in — you choose which to install. Each mod is a standalone `.so`
loaded by the loader. To write your own, see [MOD_SPEC.md](MOD_SPEC.md) and the
starter [mods/template](mods/template).

### Menu architecture

The `menu` mod is a pure framework — it provides the `natartos_menu_register()`
API that other mods call to add items to the More menu. App mods (`doom`,
`stream`) each register their own entry during the loader's `mod_register` phase.
The menu mod never hardcodes app entries.

### Doom

Doom needs a WAD game file on the goggles. toolya downloads it for you:

```
toolya natartos install doom           # downloads Freedoom Phase 1, pushes to goggles
toolya natartos install doom --wad /path/to/my.wad   # use your own WAD
```

Or from the web UI: open the **Doom** section and click **Download WAD**.

Doom uses the 5D joystick for movement and the center press for fire.

To **exit Doom**:
1. Long-hold **BACK** (≥ 500ms) to open the Doom menu
2. Navigate to "Quit Game" with the joystick
3. On the confirm dialog, long-hold **BACK** again (center/5D confirm does not work here)
4. Doom exits and the overlay nudges the 5D focus so the hidden GUI repaints;
   the More menu stays open — press **BACK** to return home. If the screen is
   still frozen, press **BACK** once or wiggle the joystick **left-right**.
   Do **not** press the 5D center — Doom is still selected in the menu and
   center will relaunch it

## Install

natartos is installed through **toolya** — either the CLI or the web UI at
[toolya.dev](https://toolya.dev):

```
toolya adb --yes                 # enable ADB (required after every reboot)
toolya natartos install          # latest release, interactive mod selection
toolya natartos start            # start mods (kills stock dji_glasses, restarts with loader)
toolya natartos status           # check installed version + active mods
toolya natartos safe on          # kill-switch: disable mods this session
toolya natartos uninstall        # remove everything, restore backups
```

The web UI at toolya.dev provides the same install flow over WebUSB+ADB.

Every artifact is **SHA-256 verified** against the release manifest before any
device write.

### After a reboot

Mods do not persist across reboots yet. After powering on:

1. `toolya adb --yes` — re-enable ADB via DUML
2. `toolya natartos start` — start mods

Installed files on `/vendor` survive reboots — you only need to re-start,
not reinstall.

## Uninstall

```
toolya natartos uninstall
```

Removes all natartos files from `/vendor` and restores backups.

## Build

Build the distributable with Docker:

```sh
bash scripts/build.sh
```

The script writes the loader, mods, helper binaries, and manifest to `dist/`.
It builds `eyepiece_bridge` on the host if an Android NDK is available via
`ANDROID_NDK_HOME` or `~/Android/Sdk/ndk/`; otherwise, it skips the bridge.

Alternatively, build with the Nix flake on macOS or Linux:

```sh
nix build
```

The flake accepts Google's Android SDK license and includes the NDK, so the
result includes `eyepiece_bridge`. For local builds without Docker, run
`nix develop` followed by `bash scripts/build.sh`.

## Documentation

- [MOD_SPEC.md](MOD_SPEC.md) — mod descriptor and how to write a mod
- [mods/template](mods/template) — starter template for your own mod
- [mods/menu/RE_MENU.md](mods/menu/RE_MENU.md) — menu injection reverse-engineering notes
- [SECURITY.md](SECURITY.md) — threat model and trust boundaries

## Roadmap

| Version | What | Status |
|---------|------|--------|
| **v0.1.0** | Per-session mods, web install + start + mod picker, Doom, stream, background swap | current |
| v0.2.0 | Stabilization — polish existing mods, improve stream quality, better error handling | planned |
| v0.3.0 | Reboot persistence (without `/system` modification) — community contributions welcome | research |

## dm_noverity

`toolya natartos noverity` temporarily disables dm-verity on `/vendor` for the
current session. This replaces the dm-verity device-mapper target with dm-linear
via ioctl — the kernel bypass is per-session only and reverts automatically on
reboot.

**What it enables:** writing to previously verity-protected paths like
`/vendor/lib64/`. Without it, writes appear to succeed (sha256 matches on read)
but `dlopen` loads the original content from the block device.

**Safety:**
- Per-session only — reboot restores dm-verity automatically
- Does **not** touch `/system` — only `/vendor`
- natartos itself does not require noverity (the loader lives in
  `/vendor/natartos/`, which is not verity-protected)

**Danger:**
- **Do not write to `/system`** after disabling verity — dm-verity mode 2
  (`restart_on_corruption`) will brick the device on reboot
- **Do not zero verity metadata** — this makes the partition unmountable

## If you bricked it

The goggles run dm-verity in mode 2 (`restart_on_corruption`) on `/system`: any
write to `/system` that isn't followed by a hash-tree recalculation bricks the
device on reboot (fast bootloop, USB does not enumerate).

**There is currently no known way to unbrick a device in this state.** If you find
one, please open an issue.

natartos itself **does not write to `/system`** — only `/vendor` (rw) and `/data`.
Don't modify `/system` unless you can recalculate the verity hash tree.

## Questions / help

Join the **[Discord](https://discord.gg/dDPb7CnmBb)** — questions, ideas, show what you built.

Bugs and feature requests: **[Issues](https://github.com/grom-labs/natartos/issues)**.

## License

AGPL-3.0 © 2026 grom-labs. See [LICENSE](LICENSE) and [NOTICE](NOTICE).

Not affiliated with, authorized, or endorsed by DJI.
