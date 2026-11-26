// SPDX-License-Identifier: AGPL-3.0-or-later
#define dlopen fake_dlopen
#define dlsym fake_dlsym
#include "../dispout.c"
#undef dlopen
#undef dlsym
#include <assert.h>
#include <stdarg.h>

static int fail_acquire, fail_callback, fail_move, releases, display_closes, device_closes;
static disp_blend_t blend;

void vlog(const char *fmt, ...) { (void)fmt; }
uint64_t now_us(void) { return 1; }
int hwdec_hal_init(void) { return 0; }
static int dev_open(const char *name, void *cfg, void **out) {
    (void)name; (void)cfg; *out = (void *)1; return 0;
}
static int dev_close(void *dev) { (void)dev; device_closes++; return 0; }
static int display_open(void *dev, void **out, uint8_t vop) {
    (void)dev; assert(vop == 0); *out = (void *)2; return 0;
}
static int display_close(void *dev, void **out) {
    (void)dev; *out = NULL; display_closes++; return 0;
}
static int acquire(void *disp, uint8_t mode, int8_t *plane) {
    (void)disp; assert(mode == 6 && *plane == 6); return fail_acquire ? -1 : 0;
}
static int release(void *disp, int8_t plane) {
    (void)disp; assert(plane == 6); releases++; return 0;
}
static int register_cb(void *disp, int8_t plane, pop_cb_t cb, void *user) {
    (void)disp; (void)user; assert(plane == 6 && cb); return fail_callback ? -1 : 0;
}
static int push(void *disp, int8_t plane, void *fb) {
    (void)disp; (void)plane; (void)fb; return 0;
}
static int blending(void *disp, int8_t plane, void *cfg) {
    (void)disp; assert(plane == 6); memcpy(&blend, cfg, sizeof(blend)); return fail_move ? -1 : 0;
}
static void released(void *user, void *fb) { (void)user; (void)fb; }
void *fake_dlopen(const char *name, int flags) { (void)name; (void)flags; return (void *)1; }
void *fake_dlsym(void *handle, const char *name) {
    (void)handle;
    if (!strcmp(name, "duss_hal_device_open")) return (void *)dev_open;
    if (!strcmp(name, "duss_hal_device_close")) return (void *)dev_close;
    if (!strcmp(name, "duss_hal_display_open")) return (void *)display_open;
    if (!strcmp(name, "duss_hal_display_close")) return (void *)display_close;
    if (!strcmp(name, "duss_hal_display_aquire_plane")) return (void *)acquire;
    if (!strcmp(name, "duss_hal_display_release_plane")) return (void *)release;
    if (!strcmp(name, "duss_hal_display_register_frame_cycle_callback")) return (void *)register_cb;
    if (!strcmp(name, "duss_hal_display_push_frame")) return (void *)push;
    if (!strcmp(name, "duss_hal_display_plane_blending_set")) return (void *)blending;
    return NULL;
}

int main(void) {
    fail_acquire = 1;
    assert(!dispout_open_pane(640, 480, 100, 50, 0, 255, released, NULL));
    assert(releases == 0 && display_closes == 1 && device_closes == 1);
    fail_acquire = 0; fail_callback = 1;
    assert(!dispout_open_pane(640, 480, 100, 50, 0, 255, released, NULL));
    assert(releases == 1 && display_closes == 2 && device_closes == 2);
    fail_callback = 0;
    dispout_t *out = dispout_open_pane(640, 480, 100, 50, 0, 255, released, NULL);
    assert(out);
    assert(blend.enable == 1 && blend.glb_alpha_en == 0 && blend.pre_blend == 0);
    assert(blend.order == 1 && blend.hoffset == 100 && blend.voffset == 50);
    assert(dispout_move_pane(out, 1280, 600) == 0);
    assert(blend.enable == 1 && blend.glb_alpha_en == 0 && blend.order == 1);
    assert(blend.hoffset == 1280 && blend.voffset == 600);
    fail_move = 1;
    assert(dispout_move_pane(out, 0, 0) != 0);
    fail_move = 0;
    assert(dispout_close_checked(out) == 0);
    assert(releases == 2 && display_closes == 3 && device_closes == 3);
    out = dispout_open_pane(640, 480, 100, 50, 1, 128, released, NULL);
    assert(out && blend.glb_alpha_en == 1 && blend.glb_alpha_val == 128);
    assert(dispout_move_pane(out, 200, 150) == 0);
    assert(blend.hoffset == 200 && blend.voffset == 150 && blend.glb_alpha_val == 128);
    assert(dispout_set_pane_alpha(out, 64) == 0);
    assert(blend.hoffset == 200 && blend.voffset == 150 && blend.glb_alpha_val == 64);
    fail_move = 1;
    assert(dispout_set_pane_alpha(out, 255) != 0);
    assert(dispout_move_pane(out, 0, 0) != 0);
    fail_move = 0;
    assert(dispout_move_pane(out, 300, 250) == 0);
    assert(blend.hoffset == 300 && blend.voffset == 250 && blend.glb_alpha_val == 64);
    assert(dispout_close_checked(out) == 0);
    return 0;
}
