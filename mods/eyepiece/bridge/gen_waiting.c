// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * gen_waiting.c — host tool for gen_waiting.sh: renders the bridge's "waiting
 * for the PC" screen (same 5x7 font as fbtext.c) as one raw 1920x1080 BGRA
 * frame on stdout. Argument: 1 = with the "press Back" hint.
 */
#include <stdio.h>
#include <stdlib.h>
#include "bridge.h"

int main(int argc, char **argv) {
    const int w = 1920, h = 1080, cy = h / 2;
    int back = argc > 1 && atoi(argv[1]);
    static const char *L1 = "EYEPIECE - WAITING FOR THE PC";
    static const char *L2 = "ON THE PC: EYEPIECE-PC  OR  TOOLYA WEB > EYEPIECE STREAM";
    const char *L3 = back ? "PRESS BACK TO EXIT" : "";
    uint32_t *px = calloc((size_t)w * h, 4);
    if (!px) return 1;
    for (size_t i = 0; i < (size_t)w * h; i++) px[i] = 0xff000000u;
    uint8_t *s = (uint8_t *)px;
    /* big enough to read through the lenses: 40x56 px title, 25x35 px hints */
    fbtext_draw(s, w, h, (w - fbtext_width(L1, 8)) / 2, cy - 140, 8, L1, 0xffffffffu);
    fbtext_draw(s, w, h, (w - fbtext_width(L2, 5)) / 2, cy + 10, 5, L2, 0xffc8c8c8u);
    fbtext_draw(s, w, h, (w - fbtext_width(L3, 5)) / 2, cy + 90, 5, L3, 0xffc8c8c8u);
    fwrite(px, 4, (size_t)w * h, stdout);
    return 0;
}
