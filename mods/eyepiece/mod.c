// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * mod_zuim_eyepiece.so — natartos side of Eyepiece (the goggles as a USB PC
 * display with head tracking).
 *
 * Adds the "Eyepiece" entry to the More menu; it launches
 * /vendor/natartos/apps/eyepiece/eyepiece_bridge, which receives H.264/HEVC
 * from the PC over adb-forwarded TCP, shows it, and streams the head pose
 * back. Head tracking comes from the goggles' MCU and is handled entirely by
 * the bridge (no hook in dji_glasses). No libc: raw syscalls only, like the
 * other natartos mods.
 */
#define NTLOG_PREFIX "eyepiece"
#include "natartos_common.h"

int natartos_mod_register(void *(*loader_dlsym)(void *, const char *)) {
    natartos_menu_register_fn reg =
        (natartos_menu_register_fn)loader_dlsym(RTLD_DEFAULT, "natartos_menu_register");
    if (!reg) { NTLOG("menu_register not found"); return 0; }
    natartos_menu_entry e = {
        .id        = "zuim_eyepiece",
        .name      = "Eyepiece",
        .subtitle  = "PC display + head tracking over USB",
        .icon_path = "/vendor/natartos/icons/natartos_icon_stream.rgba",
        .command   = "nohup /vendor/natartos/apps/eyepiece/eyepiece_bridge -b >/dev/null 2>&1 &",
        .priority  = 210,
    };
    reg(&e);
    NTLOG("mod_zuim_eyepiece: menu entry registered");
    return 1;
}
