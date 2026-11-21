// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * fbout.c — producer side of natartos' fb_shared protocol (same layout as
 * natartos/mods/stream/fb_receiver.c):
 *
 *   /data/local/tmp/fb_shared = [64-byte header][slot0 BGRA][slot1 BGRA]
 *   header u32 @0 = frame counter; the overlay's inject thread copies slot
 *   (counter & 1) into the scanout DMA-bufs whenever the counter changes.
 *   /data/local/tmp/fb_override present = "an app owns the display".
 *
 * The producer always writes the slot the overlay is NOT reading, then
 * bumps the counter (release) — no tearing, no locks.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include "bridge.h"

#define SHM_HEADER 64
#define SHM_PATH   tmp_path("fb_shared")
#define FLAG_PATH  tmp_path("fb_override")
#define STOP_PATH  tmp_path("fb_stop")

int fbout_open(fbout_t *fb, int w, int h) {
    memset(fb, 0, sizeof(*fb));
    fb->w = w; fb->h = h;
    fb->slot = (size_t)w * h * 4;
    fb->size = SHM_HEADER + 2 * fb->slot;
    unlink(STOP_PATH);
    fb->fd = open(SHM_PATH, O_RDWR | O_CREAT, 0666);
    if (fb->fd < 0) { vlog("fbout: open %s: %s", SHM_PATH, strerror(errno)); return -1; }
    if (ftruncate(fb->fd, (off_t)fb->size) < 0) { vlog("fbout: ftruncate: %s", strerror(errno)); return -1; }
    fb->map = mmap(NULL, fb->size, PROT_READ | PROT_WRITE, MAP_SHARED, fb->fd, 0);
    if (fb->map == MAP_FAILED) { fb->map = NULL; vlog("fbout: mmap: %s", strerror(errno)); return -1; }
    fb->counter = (volatile uint32_t *)fb->map;
    memset(fb->map, 0, SHM_HEADER);
    fbout_clear(fb);
    int ff = open(FLAG_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (ff >= 0) close(ff);
    vlog("fbout: %dx%d fb_shared ready, fb_override set", w, h);
    return 0;
}

uint8_t *fbout_back(fbout_t *fb) {
    uint32_t c = __atomic_load_n(fb->counter, __ATOMIC_RELAXED);
    return fb->map + SHM_HEADER + (size_t)((c + 1) & 1) * fb->slot;
}

void fbout_publish(fbout_t *fb) {
    __atomic_add_fetch(fb->counter, 1, __ATOMIC_RELEASE);
}

void fbout_fill(fbout_t *fb, uint32_t px) {
    uint32_t *p = (uint32_t *)(fb->map + SHM_HEADER);
    size_t n = 2 * fb->slot / 4;
    for (size_t i = 0; i < n; i++) p[i] = px;
}

/* Opaque black in both slots (letterbox bars, start-up).                  */
void fbout_clear(fbout_t *fb) { fbout_fill(fb, 0xff000000u); }

/* The overlay's long-hold-Back handler drops fb_stop to tear the app down. */
int fbout_stop_requested(void) {
    return access(STOP_PATH, F_OK) == 0;
}

void fbout_close(fbout_t *fb) {
    unlink(FLAG_PATH);
    if (fb->map) munmap(fb->map, fb->size);
    if (fb->fd >= 0) close(fb->fd);
    fb->map = NULL; fb->fd = -1;
}
