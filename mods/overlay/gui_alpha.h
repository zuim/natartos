// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef NATARTOS_GUI_ALPHA_H
#define NATARTOS_GUI_ALPHA_H

#include <stdint.h>

#ifndef GUI_ALPHA_PATH
#define GUI_ALPHA_PATH "/data/local/tmp/eyepiece.gui-alpha"
#endif

typedef struct {
    uint64_t request;
    uint64_t acknowledged;
    uint32_t alpha;
    uint32_t status;
} gui_alpha_request;

static inline int gui_alpha_effective(int hidden, int requested, uint32_t alpha) {
    return hidden ? 0 : requested ? (int)alpha : 255;
}

static inline int gui_alpha_plane_order(const char *text, int len, int plane) {
    int vop = 0, selected = 0;
    for (int i = 0; i < len;) {
        int start = i;
        while (i < len && text[i] != '\n') i++;
        int end = i++;
        while (start < end && (text[start] == ' ' || text[start] == '\t')) start++;
        if (end - start >= 5 && text[start] == 'V' && text[start + 1] == 'O' &&
            text[start + 2] == 'P' && text[start + 3] == ' ') {
            vop = text[start + 4] == '0';
            selected = 0;
        } else if (vop && end - start >= 8 && text[start] == 'p' &&
                   text[start + 1] == 'l' && text[start + 2] == 'a' &&
                   text[start + 3] == 'n' && text[start + 4] == 'e' &&
                   text[start + 5] == ' ') {
            selected = text[start + 6] == '0' + plane && text[start + 7] == ' ';
        } else if (selected) {
            for (int j = start; j + 7 < end; j++) {
                if (text[j] == 'o' && text[j + 1] == 'r' && text[j + 2] == 'd' &&
                    text[j + 3] == 'e' && text[j + 4] == 'r' && text[j + 5] == ':' &&
                    text[j + 6] == ' ' && text[j + 7] >= '0' && text[j + 7] <= '5' &&
                    (j + 8 == end || text[j + 8] == ',' || text[j + 8] == ' '))
                    return text[j + 7] - '0';
            }
        }
    }
    return -1;
}

static inline int gui_alpha_blend_order(const char *text, int len, int plane) {
    int order = gui_alpha_plane_order(text, len, plane);
    /* GFX1 acquisition can leave both graphics planes at order 1. */
    if (plane == 5 && order == 1 && gui_alpha_plane_order(text, len, 6) == 1)
        return 2;
    return order;
}

#endif
