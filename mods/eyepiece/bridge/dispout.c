// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * dispout.c — scan-out of decoded NV12 frames on VIDEO0 and converted BGRA
 * panes on GFX1, using the display HAL from /system/bin/test_disp.
 *
 *   duss_hal_device_open("/dev/dji_display", &0, &dev)
 *   duss_hal_display_open(dev, &disp, vop)              per VOP (0 = left, 1 = right eye?)
 *   duss_hal_display_aquire_plane(disp, 6, &plane)      plane in/out, 4 = DUSS_DISP_PLANE_VIDEO0
 *   duss_hal_display_register_frame_cycle_callback(disp, plane, pop_cb, user)
 *   duss_hal_display_push_frame(disp, plane, fb)        pop_cb(inst, plane, fb, user) when the
 *                                                       display is done with fb → back to decoder
 *
 * Normal ordering is VIDEO0, GFX1, GFX0. Only a plane's owner may change its blending.
 */
#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/wait.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "bridge.h"

#define ACQUIRE_MODE 6            /* what test_disp passes for an explicit plane id */
#define MAX_VOP 2

typedef int (*pop_cb_t)(void *inst, int8_t plane, void *fb, void *user);

static struct {
    int (*device_open)(const char *name, void *cfg, void **h);
    int (*device_close)(void *h);
    int (*display_open)(void *dev, void **disp, uint8_t vop);
    int (*display_close)(void *dev, void **disp);
    int (*aquire_plane)(void *disp, uint8_t mode, int8_t *plane);
    int (*release_plane)(void *disp, int8_t plane);
    int (*register_frame_cycle_callback)(void *disp, int8_t plane, pop_cb_t cb, void *user);
    int (*push_frame)(void *disp, int8_t plane, void *fb);
    int (*blending_set)(void *disp, int8_t plane, void *cfg);
    int (*timing_set)(void *disp, void *cfg);
    int (*crop_set)(void *disp, int8_t plane, void *cfg);
} D;

/* EXPERIMENTAL stereo (EYEPIECE_STEREO=<timing>, e.g. 1901): one side-by-side frame
 * (W×H, W = 2 eyes) goes to VOP0 (left eye) and VOP1 (right eye), each VIDEO0 cropped to
 * its half; VOP0 is switched to the 3D timing like the stock player's DisplaySink
 * (duss_hal_display_timing_set {res, fmt, bitdepth}) and back to {0,0,0} on close —
 * DisplaySink::deinit does the same ("Set display to 2D timing before display close"). */
typedef struct __attribute__((packed)) { uint32_t res, fmt, bitdepth; } disp_timing_t;
/* test_disp "crop $plane $en $hoff $voff $hsize $vsize" → duss_hal_display_plane_crop_set */
typedef struct __attribute__((packed)) { uint8_t en, _pad[3]; int32_t hoff, voff, hsize, vsize; } disp_crop_t;
static void *g_timing_disp;    /* VOP0 while in the 3D timing (crash handler) */

/* duss_hal_display_plane_blending_set config (test_disp "blending" command,
 * /system/bin/test_disp). Without it the VIDEO0 plane stays "user enabled,
 * actually disabled / no input frame" (/proc/display/plane) and every pushed
 * frame is silently dropped (black screen). */
typedef struct __attribute__((packed)) {
    uint8_t  enable;          /* 0x00 */
    uint8_t  glb_alpha_en;    /* 0x01 */
    uint8_t  pre_blend;       /* 0x02 */
    uint8_t  _pad0;
    uint32_t glb_alpha_val;   /* 0x04 0..255 */
    uint8_t  alg;             /* 0x08 0: up_alp*up + (1-up_alp)*base */
    uint8_t  order;           /* 0x09 0..5; GUI: gfx1 = 1 (arhome), gfx0 = 2 (dji_glasses) */
    uint8_t  _pad1[2];
    uint32_t pos;             /* 0x0c 0 = free position (h/v offset) */
    int32_t  hoffset;         /* 0x10 */
    int32_t  voffset;         /* 0x14 */
} disp_blend_t;               /* 0x18 */

/* No plane scaling: /proc/display/feature lists no "scale" capability for any
 * plane on this display controller (VIDEO0: crop, blend, dimming, …), and
 * duss_hal_display_plane_scale_set returns -1001. A stream smaller than the
 * panel is shown 1:1, centred with the blending offset.                    */

#define PANEL_W 1920
#define PANEL_H 1080

struct dispout;
typedef struct { struct dispout *o; int v; } vop_ctx_t;

struct dispout {
    void *dev;
    vop_ctx_t vctx[MAX_VOP];
    int stereo;
    void *disp[MAX_VOP];
    int8_t plane[MAX_VOP];
    int nvop;
    dispout_release_cb release;
    void *user;
    pthread_mutex_t mu;
    pthread_mutex_t push_mu;
    disp_blend_t pane_blend;
    /* A frame is free after every VOP that received it has released it. */
    struct { void *fb; uint32_t seq; uint8_t pend; } held[32];
    uint32_t pushed, popped, errors, reclaimed;
};

static int load(void) {
    void *h = dlopen("libduml_hal.so", RTLD_NOW | RTLD_GLOBAL);
    if (!h) { vlog("dispout: dlopen: %s", dlerror()); return -1; }
#define L(f, n) do { *(void **)&D.f = dlsym(h, n); if (!D.f) { vlog("dispout: missing %s", n); return -1; } } while (0)
    L(device_open, "duss_hal_device_open");
    L(device_close, "duss_hal_device_close");
    L(display_open, "duss_hal_display_open");
    L(display_close, "duss_hal_display_close");
    L(aquire_plane, "duss_hal_display_aquire_plane");
    L(release_plane, "duss_hal_display_release_plane");
    L(register_frame_cycle_callback, "duss_hal_display_register_frame_cycle_callback");
    L(push_frame, "duss_hal_display_push_frame");
    L(blending_set, "duss_hal_display_plane_blending_set");
    *(void **)&D.timing_set = dlsym(h, "duss_hal_display_timing_set");
    *(void **)&D.crop_set = dlsym(h, "duss_hal_display_plane_crop_set");
#undef L
    return 0;
}

/* The display drops frames when they arrive faster than it scans out (queue
 * of 6; e.g. 100 fps into the 100 Hz panel with jitter) and does NOT return
 * dropped frames through the frame-cycle callback. Each drop then leaks one
 * decoder output buffer until the decoder starves (12 fps, "no free stream
 * buffer"). The queue is FIFO, so when frame N comes back every older frame
 * still held for the same VOP is a drop: reclaim it. */
static int on_pop(void *inst, int8_t plane, void *fb, void *user) {
    (void)inst; (void)plane;
    vop_ctx_t *vc = user;
    dispout_t *o = vc->o;
    uint8_t bit = (uint8_t)(1u << vc->v);
    if (!fb) return 0;
    void *rel[33]; int nrel = 0;
    pthread_mutex_lock(&o->mu);
    o->popped++;
    int me = -1;
    for (int i = 0; i < 32; i++) if (o->held[i].fb == fb && (o->held[i].pend & bit)) { me = i; break; }
    if (me >= 0) {
        /* per-VOP FIFO: every older frame still pending on this VOP was dropped by it */
        uint32_t seq = o->held[me].seq;
        for (int i = 0; i < 32; i++) {
            if (!o->held[i].fb || !(o->held[i].pend & bit)) continue;
            if (i != me && (int32_t)(o->held[i].seq - seq) >= 0) continue;
            o->held[i].pend &= (uint8_t)~bit;
            if (i != me) o->reclaimed++;
            if (!o->held[i].pend) { rel[nrel++] = o->held[i].fb; o->held[i].fb = NULL; }
        }
    }
    pthread_mutex_unlock(&o->mu);
    for (int i = 0; i < nrel; i++) o->release(o->user, rel[i]);
    return 0;
}

static dispout_t *open_display(int vop_mask, int src_w, int src_h, int stereo_res,
                               int pane, int opaque, uint8_t alpha, int x, int y,
                               dispout_release_cb release, void *user) {
    if (load() < 0) return NULL;
    dispout_t *o = calloc(1, sizeof(*o));
    if (!o) return NULL;
    pthread_mutex_init(&o->mu, NULL);
    pthread_mutex_init(&o->push_mu, NULL);
    o->release = release; o->user = user;
    const char *st = pane ? NULL : getenv("EYEPIECE_STEREO");   /* experiment override */
    if (st) stereo_res = atoi(st);
    o->stereo = st != NULL || stereo_res > 0;
    if (o->stereo) {
        vop_mask = 3;
        vlog("dispout: stereo: %dx%d side by side → VOP0 left / VOP1 right, "
             "%d×%d per eye, timing %d", src_w, src_h, src_w / 2, src_h, stereo_res);
    }
    /* stream smaller than the panel: 1:1, centred (no plane scaler) */
    int need_center = !pane && !o->stereo && src_w > 0 && src_h > 0 && src_w <= PANEL_W && src_h <= PANEL_H &&
                      (src_w != PANEL_W || src_h != PANEL_H);
    uint32_t zero = 0;
    int r = D.device_open("/dev/dji_display", &zero, &o->dev);
    if (r != 0) { vlog("dispout: open /dev/dji_display: %d", r); free(o); return NULL; }
    for (int vop = 0; vop < MAX_VOP; vop++) {
        if (!(vop_mask & (1 << vop))) continue;
        void *disp = NULL;
        int plane_id = pane ? 6 : 4;
        int8_t plane = (int8_t)plane_id;
        if ((r = D.display_open(o->dev, &disp, (uint8_t)vop)) != 0) {
            vlog("dispout: display_open(vop %d): %d", vop, r); continue;
        }
        if ((r = D.aquire_plane(disp, ACQUIRE_MODE, &plane)) != 0) {
            vlog("dispout: acquire plane %d on vop %d: %d (in use?)", plane_id, vop, r);
            r = D.display_close(o->dev, &disp);
            if (r != 0) vlog("dispout: display_close(vop %d): %d", vop, r);
            continue;
        }
        if (pane && plane != plane_id) {
            vlog("dispout: requested plane %d, acquired %d", plane_id, plane);
            r = D.release_plane(disp, plane);
            if (r != 0) vlog("dispout: release unexpected plane %d: %d", plane, r);
            r = D.display_close(o->dev, &disp);
            if (r != 0) vlog("dispout: display_close(vop %d): %d", vop, r);
            continue;
        }
        disp_blend_t b = { .enable = 1, .glb_alpha_en = !pane || opaque,
                           .glb_alpha_val = pane && opaque ? alpha : 255,
                           .alg = 0, .order = pane ? 1 : 0, .pos = 0 };
        if (need_center) { b.hoffset = ((PANEL_W - src_w) / 2) & ~1; b.voffset = ((PANEL_H - src_h) / 2) & ~1; }
        if (pane) { b.hoffset = x; b.voffset = y; }
        if ((r = D.blending_set(disp, plane, &b)) != 0) {
            vlog("dispout: blending_set vop %d plane %d: %d", vop, plane, r);
            int release_r = D.release_plane(disp, plane);
            if (release_r != 0)
                vlog("dispout: release plane %d on vop %d: %d", plane, vop, release_r);
            r = D.display_close(o->dev, &disp);
            if (r != 0) vlog("dispout: display_close(vop %d): %d", vop, r);
            continue;
        }
        if (o->stereo) {
            disp_crop_t c = { .en = 1, .hoff = vop * (src_w / 2), .voff = 0, .hsize = src_w / 2, .vsize = src_h };
            r = D.crop_set ? D.crop_set(disp, plane, &c) : -1;
            vlog("dispout: vop %d crop +%d+0 %dx%d → %d", vop, c.hoff, c.hsize, c.vsize, r);
        }
        if (need_center)
            vlog("dispout: vop %d: %dx%d stream shown 1:1 centred at +%d+%d", vop, src_w, src_h,
                 b.hoffset, b.voffset);
        o->vctx[o->nvop].o = o; o->vctx[o->nvop].v = o->nvop;
        if ((r = D.register_frame_cycle_callback(disp, plane, on_pop, &o->vctx[o->nvop])) != 0) {
            vlog("dispout: frame-cycle callback vop %d: %d", vop, r);
            if (pane) {
                r = D.release_plane(disp, plane);
                if (r != 0) vlog("dispout: release plane %d on vop %d: %d", plane, vop, r);
                r = D.display_close(o->dev, &disp);
                if (r != 0) vlog("dispout: display_close(vop %d): %d", vop, r);
                continue;
            }
        }
        o->disp[o->nvop] = disp;
        o->plane[o->nvop] = plane;
        if (pane) o->pane_blend = b;
        o->nvop++;
        vlog("dispout: vop %d plane %d acquired (order %u, alpha %u)",
             vop, plane, b.order, b.glb_alpha_val);
    }
    if (!o->nvop) {
        r = D.device_close(o->dev);
        if (r != 0) vlog("dispout: device_close: %d", r);
        free(o);
        return NULL;
    }
    if (o->stereo && stereo_res && o->nvop == 2 && D.timing_set) {
        disp_timing_t t = { (uint32_t)stereo_res, 0, 0 };
        r = D.timing_set(o->disp[0], &t);
        vlog("dispout: vop 0 timing_set(%d) → %d", stereo_res, r);
        if (r == 0) g_timing_disp = o->disp[0];
    }
    return o;
}

dispout_t *dispout_open(int vop_mask, int src_w, int src_h, int stereo_res,
                        dispout_release_cb release, void *user) {
    return open_display(vop_mask, src_w, src_h, stereo_res, 0, 0, 255, 0, 0, release, user);
}

dispout_t *dispout_open_pane(int w, int h, int x, int y, int opaque, uint8_t alpha,
                            dispout_release_cb release, void *user) {
    return open_display(1, w, h, 0, 1, opaque, alpha, x, y, release, user);
}

int dispout_move_pane(dispout_t *o, int x, int y) {
    if (!o || o->nvop != 1 || o->plane[0] != 6) return -1;
    pthread_mutex_lock(&o->push_mu);
    disp_blend_t b = o->pane_blend;
    b.hoffset = x; b.voffset = y;
    int r = D.blending_set(o->disp[0], o->plane[0], &b);
    if (r == 0) o->pane_blend = b;
    pthread_mutex_unlock(&o->push_mu);
    if (r != 0) vlog("dispout: move GFX1 to +%d+%d failed: %d", x, y, r);
    return r;
}

int dispout_set_pane_alpha(dispout_t *o, uint8_t alpha) {
    if (!o || o->nvop != 1 || o->plane[0] != 6) return -1;
    pthread_mutex_lock(&o->push_mu);
    disp_blend_t b = o->pane_blend;
    int r = -1;
    if (b.glb_alpha_en) {
        b.glb_alpha_val = alpha;
        r = D.blending_set(o->disp[0], o->plane[0], &b);
        if (r == 0) o->pane_blend = b;
    }
    pthread_mutex_unlock(&o->push_mu);
    if (r != 0) vlog("dispout: set GFX1 global alpha %u failed: %d", alpha, r);
    return r;
}

/* 0 = queued on ≥1 VOP (frame now owned by the display until popped) */
int dispout_push(dispout_t *o, void *fb) {
    pthread_mutex_lock(&o->push_mu);
    pthread_mutex_lock(&o->mu);
    int slot = -1;
    for (int i = 0; i < 32; i++) if (!o->held[i].fb) { slot = i; break; }
    if (slot >= 0) { o->held[slot].fb = fb; o->held[slot].seq = o->pushed;
                     o->held[slot].pend = (uint8_t)((1u << o->nvop) - 1); }
    pthread_mutex_unlock(&o->mu);
    if (slot < 0) { o->errors++; pthread_mutex_unlock(&o->push_mu); return -1; }
    /* display timestamp (duss_frame_buffer +0x30, µs). The decoder copies the
     * PC's pts there; EYEPIECE_DISP_TS = keep | zero | now (experiment knob).  */
    static int ts_mode = -1;
    if (ts_mode < 0) {
        const char *e = getenv("EYEPIECE_DISP_TS");
        ts_mode = !e ? 0 : !strcmp(e, "zero") ? 1 : !strcmp(e, "now") ? 2 : 0;
        vlog("dispout: timestamp mode %s", ts_mode == 1 ? "zero" : ts_mode == 2 ? "now" : "keep");
    }
    if (ts_mode == 1) *(uint64_t *)((uint8_t *)fb + 0x30) = 0;
    else if (ts_mode == 2) {
        struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
        *(uint64_t *)((uint8_t *)fb + 0x30) = (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
    }
    int ok = 0, rejected = 1;
    for (int i = 0; i < o->nvop; i++) {
        int r = D.push_frame(o->disp[i], o->plane[i], fb);
        if (r == 0) { ok++; continue; }
        if (r != -1001) rejected = 0;
        if (o->errors++ < 20) vlog("dispout: push_frame vop#%d: %d", i, r);
        void *released = NULL;
        pthread_mutex_lock(&o->mu);                  /* this VOP won't pop it */
        o->held[slot].pend &= (uint8_t)~(1u << i);
        /* Accepted VOPs may already have popped while this push failed. */
        if (ok && !o->held[slot].pend) {
            released = o->held[slot].fb;
            o->held[slot].fb = NULL;
        }
        pthread_mutex_unlock(&o->mu);
        if (released) o->release(o->user, released);
    }
    if (!ok) {
        pthread_mutex_lock(&o->mu); o->held[slot].fb = NULL; pthread_mutex_unlock(&o->mu);
        pthread_mutex_unlock(&o->push_mu);
        return rejected ? DISPOUT_PUSH_REJECTED : DISPOUT_PUSH_ERROR;
    }
    o->pushed++;
    if (o->pushed == 1) vlog("dispout: first frame pushed to %d VOP(s)", ok);
    if (o->pushed % 600 == 0)
        vlog("dispout: pushed %u, returned %u, reclaimed (display drops) %u, errors %u",
             o->pushed, o->popped, o->reclaimed, o->errors);
    pthread_mutex_unlock(&o->push_mu);
    return 0;
}

/* A bridge killed with SIGKILL in stereo leaves VOP0 in the 3D timing (TESTLOG 114):
 * at startup, reset it if /proc/display/timing shows the 3D timing on VOP 0. */
static void reset_3d_child(void) {
    if (hwdec_hal_init() < 0 || load() < 0 || !D.timing_set) _exit(2);
    void *dev = NULL, *disp = NULL; uint32_t zero = 0;
    if (D.device_open("/dev/dji_display", &zero, &dev) != 0) _exit(3);
    if (D.display_open(dev, &disp, 0) != 0) _exit(4);
    disp_timing_t t = { 0, 0, 0 };
    _exit(D.timing_set(disp, &t) == 0 ? 0 : 5);   /* _exit: no DJI destructors */
}

void dispout_reset_stale_3d(void) {
    FILE *f = fopen("/proc/display/timing", "r");
    if (!f) return;
    char l[256]; int stale = 0;
    while (fgets(l, sizeof(l), f))
        if (!strncmp(l, "VOP 0:", 6)) { stale = strstr(l, " 1901 ") != NULL; break; }
    fclose(f);
    if (!stale) return;
    /* in a child: the duss HAL starts threads, and the bridge forks its head-tracking
     * helper later, which must happen before any thread exists */
    pid_t p = fork();
    if (p == 0) reset_3d_child();
    int st = -1;
    if (p > 0) waitpid(p, &st, 0);
    vlog("dispout: VOP0 was left in the 3D timing (killed stereo session) → 2D: %s (%d)",
         p > 0 && WIFEXITED(st) && WEXITSTATUS(st) == 0 ? "ok" : "failed", st);
}

/* Back to the 2D timing (exit and crash handler; idempotent). */
void dispout_stereo_off(void) {
    void *d = g_timing_disp;
    if (!d || !D.timing_set) return;
    g_timing_disp = NULL;
    disp_timing_t t = { 0, 0, 0 };
    D.timing_set(d, &t);
}

int dispout_close_checked(dispout_t *o) {
    if (!o) return 0;
    int failed = 0;
    if (g_timing_disp) { dispout_stereo_off(); vlog("dispout: vop 0 timing back to 2D"); }
    for (int i = 0; i < o->nvop; i++) {
        int release_r = D.release_plane(o->disp[i], o->plane[i]);
        if (release_r != 0)
            vlog("dispout: release plane %d on vop #%d: %d", o->plane[i], i, release_r);
        if (release_r != 0) failed = 1;
        int r = D.display_close(o->dev, &o->disp[i]);
        if (r != 0) vlog("dispout: display_close(vop #%d): %d", i, r);
        if (r != 0) failed = 1;
    }
    int r = D.device_close(o->dev);
    if (r != 0) vlog("dispout: device_close: %d", r);
    if (r != 0) failed = 1;
    vlog("dispout: closed (pushed %u, popped %u, reclaimed %u, errors %u)",
         o->pushed, o->popped, o->reclaimed, o->errors);
    return failed ? -1 : 0;
}
