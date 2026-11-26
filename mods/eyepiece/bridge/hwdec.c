// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * hwdec.c — hardware H.264/HEVC decode on the goggles' Verisilicon VC8000D
 * through DJI's HAL (/dev/vdec0 + /dev/ion0), reproducing the flow of the
 * stock /system/bin/test_vdec, using offsets from static analysis.
 *
 *   submit:  copy AU into a free ION stream buffer → duss_hal_vdec_decstrm(vdec, {0, strm})
 *   decoder: strm_empty_cb(strm) when consumed, frame_full_cb(frame) when a
 *            picture is ready (NV12 in an ION frame buffer)
 *   return:  frames go back with duss_hal_vdec_decstrm(vdec, {frame, 0}) from
 *            our own thread (never from inside a decoder callback)
 *
 * Every struct offset below comes from test_vdec; the log prints enough to
 * check them on the first device run.
 */
#include <dlfcn.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "bridge.h"

/* ---- descriptors (sizes/offsets from test_vdec_allocate_buffer) -------- */
#define STRM_DESC_SZ   0x1c8
#define STRM_MEM       0x20   /* duss_hal_mem handle (void*)                */
#define STRM_W         0x30
#define STRM_H         0x34
#define STRM_LEN       0x40   /* u32 bytes of bitstream                     */
#define STRM_PTS       0x1c0  /* u64                                        */

#define FRM_DESC_SZ    0x330
#define FRM_MEM        0x20
#define FRM_FMT        0x28   /* 2 = DUSS_PIXFMT_YUV8_SP420_YUV (NV12)      */
#define FRM_PTS        0x30   /* u64 output ts                              */
#define FRM_W          0x38
#define FRM_H          0x3c
#define FRM_STRIDE     0x40
#define FRM_VALID_A    0x50   /* both 0 → empty buffer being handed back    */
#define FRM_LUMA_H     0x4c
#define FRM_UV_STRIDE  0x5c
#define FRM_UV_OFF     0x60
#define FRM_VALID_B    0x6c
#define FRM_UV_H       0x68
#define FRM_PLANES     0xb0
#define PIXFMT_NV12    2
#define PIXFMT_RGB888_I32 20002
#define PIXFMT_ARGB8_I32 20015
#define OVERLAY_BUFS 8
#define OVERLAY_REJECT_LIMIT 30

#define U32(p, o) (*(uint32_t *)((uint8_t *)(p) + (o)))
#define U64(p, o) (*(uint64_t *)((uint8_t *)(p) + (o)))
#define PTR(p, o) (*(void **)((uint8_t *)(p) + (o)))

typedef struct { const char *name; void *attach, *detach, *rsv; } hal_dev_t;

static struct {
    int  (*initialize)(hal_dev_t *tbl);
    int  (*device_open)(const char *name, void *cfg, void **h);
    int  (*device_start)(void *h, void *cfg);
    int  (*device_stop)(void *h, void *cfg);
    int  (*device_close)(void *h);
    int  (*mem_alloc)(void *ion, void **mem, uint32_t size, uint32_t align, uint32_t a5, uint32_t a6);
    int  (*mem_map)(void *mem, void **va);
    int  (*mem_sync)(void *mem, uint32_t dir);
    int  (*mem_free)(void *mem);
    int  (*decstrm)(void *vdec, void *req);
} hal;

static hal_dev_t g_devs[4];

static int stop_device(void *device) {
    uint32_t config = 0;
    return hal.device_stop(device, &config);
}

typedef struct __attribute__((packed)) {
    uint32_t codec, one;
    void *strm_empty_cb, *frame_start_cb, *frame_full_cb, *error_cb;
    void *user[4];
    uint8_t pad[0x50 - 72];
} vdec_open_cfg_t;

typedef struct { void **frames, **strms; uint32_t nframes, nstrm; } vdec_pool_t;

/* vdec_start() config (libduml_vcodec vdec_start/vsi_set_config):        */
typedef struct __attribute__((packed)) {
    uint64_t z0, z1;
    vdec_pool_t *pool;         /* 0x10                                       */
    uint32_t scale_w, scale_h; /* 0x18 / 0x1c  0 = no scaling (must be even) */
    uint32_t stride_align;     /* 0x20                                       */
    uint32_t flag24;           /* 0x24  → instance+0x3ec (bool, unknown)     */
    uint8_t  no_reorder;       /* 0x28  ≠0 → "Requesting non-reordering decode"
                                        (instance+0xc8 = 1): output each picture
                                        immediately — what we want, no B-frames */
    uint8_t  pad29[3];
    uint32_t val2c;            /* 0x2c  → instance+0x3e0 (unknown)           */
} vdec_start_cfg_t;

#define MAX_BUF 24
struct hwdec {
    void *ion, *vdec;
    vdec_pool_t pool;
    void *strm[MAX_BUF], *frame[MAX_BUF];
    void *strm_va[MAX_BUF], *frame_va[MAX_BUF];
    int strm_busy[MAX_BUF];
    int nstrm, nframe;
    uint32_t strm_size;
    uint32_t frame_size;
    int w, h, stride;
    atomic_uint disp_drops;
    int sync_dir;                 /* <0 = no cache sync before reading frames */
    hwdec_frame_cb cb; void *user;
    dispout_t *disp;              /* zero-copy scan-out, or NULL               */
    int pane_output, pane_opaque, pane_w, pane_h, pane_stride, cvt_threads;
    int pane_direct;              /* opaque pane: decoder NV12 scanned out on GFX1, no conversion */
    int (*prepare_output)(void *user);
    int (*skip_output)(void *user);
    void (*output_error)(void *user);
    uint8_t *staging, *retained_staging;
    uint8_t pane_alpha;
    yuv_img_t retained_img;
    uint64_t retained_pts;
    struct { void *desc, *va; int busy; } overlay[OVERLAY_BUFS];
    pthread_t ret_thread;
    int ret_thread_started;
    atomic_int overlay_failed;
    unsigned overlay_rejections;
    uint64_t pane_us[3];          /* convert, cache sync, push (under staging_mu) */
    unsigned pane_timed;
    pthread_mutex_t mu;
    pthread_mutex_t staging_mu;
    pthread_cond_t strm_cv, ret_cv, callbacks_cv;
    int callbacks_active;
    void *ret_q[MAX_BUF * 2]; int ret_n;
    int returning, last_frame_pending;
    uint64_t last_submitted_pts, last_output_pts;
    atomic_int stop;
    atomic_uint frames_out, errors, output_skips;
};

static int load_hal(int with_display) {
    static int done;
    if (done) return done > 0 ? 0 : -1;
    done = -1;
    void *hh = dlopen("libduml_hal.so", RTLD_NOW | RTLD_GLOBAL);
    void *vc = dlopen("libduml_vcodec.so", RTLD_NOW | RTLD_GLOBAL);
    if (!hh || !vc) { vlog("hwdec: dlopen: %s", dlerror()); return -1; }
#define L(lib, f, n) do { *(void **)&hal.f = dlsym(lib, n); \
        if (!hal.f) { vlog("hwdec: missing %s", n); return -1; } } while (0)
    L(hh, initialize, "duss_hal_initialize");
    L(hh, device_open, "duss_hal_device_open");
    L(hh, device_start, "duss_hal_device_start");
    L(hh, device_stop, "duss_hal_device_stop");
    L(hh, device_close, "duss_hal_device_close");
    L(hh, mem_alloc, "duss_hal_mem_alloc");
    L(hh, mem_map, "duss_hal_mem_map");
    L(hh, mem_sync, "duss_hal_mem_sync");
    L(hh, mem_free, "duss_hal_mem_free");
    L(vc, decstrm, "duss_hal_vdec_decstrm");
#undef L
    void *av = dlsym(vc, "duss_hal_attach_vdec"), *dv = dlsym(vc, "duss_hal_detach_vdec");
    void *ai = dlsym(hh, "duss_hal_attach_ion_mem"), *di = dlsym(hh, "duss_hal_detach_ion_mem");
    if (!ai) ai = dlsym(RTLD_DEFAULT, "duss_hal_attach_ion_mem");
    if (!di) di = dlsym(RTLD_DEFAULT, "duss_hal_detach_ion_mem");
    if (!av || !dv || !ai || !di) { vlog("hwdec: attach/detach symbols missing"); return -1; }
    /* same device table test_vdec passes to duss_hal_initialize (NULL-terminated) */
    g_devs[0] = (hal_dev_t){ "/dev/vdec0", av, dv, NULL };
    g_devs[1] = (hal_dev_t){ "/dev/ion0", ai, di, NULL };
    g_devs[2] = (hal_dev_t){ NULL, NULL, NULL, NULL };
    g_devs[3] = (hal_dev_t){ NULL, NULL, NULL, NULL };
    if (with_display) {   /* same entry test_disp registers */
        void *ad = dlsym(hh, "duss_hal_attach_disp"), *dd = dlsym(hh, "duss_hal_detach_disp");
        if (!ad || !dd) { vlog("hwdec: duss_hal_attach_disp missing"); return -1; }
        g_devs[2] = (hal_dev_t){ "/dev/dji_display", ad, dd, NULL };
    }
    int r = hal.initialize(g_devs);
    if (r != 0) { vlog("hwdec: duss_hal_initialize: %d", r); return -1; }
    done = 1;
    return 0;
}

/* duss HAL with the display registered (once per process; dispout's stale-3D reset) */
int hwdec_hal_init(void) { return load_hal(1); }

/* ---- decoder callbacks (run on the decoder's threads) ------------------ */

static int cb_strm_empty(void *user, void *desc) {
    hwdec_t *d = user;
    pthread_mutex_lock(&d->mu);
    for (int i = 0; i < d->nstrm; i++)
        if (d->strm[i] == desc) { d->strm_busy[i] = 0; break; }
    pthread_cond_signal(&d->strm_cv);
    pthread_cond_broadcast(&d->callbacks_cv);
    pthread_mutex_unlock(&d->mu);
    return 0;
}

static int cb_frame_start(void *user, void *desc) { (void)user; (void)desc; return 0; }

static void queue_return(hwdec_t *d, void *desc) {
    pthread_mutex_lock(&d->mu);
    if (d->ret_n < (int)(sizeof(d->ret_q) / sizeof(d->ret_q[0]))) d->ret_q[d->ret_n++] = desc;
    pthread_cond_signal(&d->ret_cv);
    pthread_mutex_unlock(&d->mu);
}

static void overlay_error(hwdec_t *d, const char *message) {
    if (!atomic_exchange(&d->overlay_failed, 1)) {
        vlog("hwdec: overlay: %s", message);
        if (d->output_error) d->output_error(d->user);
    }
}

static void overlay_release(void *user, void *fb) {
    hwdec_t *d = user;
    pthread_mutex_lock(&d->mu);
    for (int i = 0; i < OVERLAY_BUFS; i++) {
        if (d->overlay[i].desc == fb) {
            d->overlay[i].busy = 0;
            pthread_mutex_unlock(&d->mu);
            return;
        }
    }
    pthread_mutex_unlock(&d->mu);
    overlay_error(d, "display returned an unknown buffer");
}

static int pane_dimensions_match(const hwdec_t *d, int width, int height) {
    return width == (d->pane_opaque ? 1 : 2) * d->pane_w && height == d->pane_h;
}

static int overlay_present(hwdec_t *d, const yuv_img_t *img, uint64_t pts) {
    if (!pane_dimensions_match(d, img->w, img->h)) {
        overlay_error(d, "decoded dimensions do not match the pane");
        return -1;
    }
    int slot = -1;
    pthread_mutex_lock(&d->mu);
    for (int i = 0; i < OVERLAY_BUFS; i++) {
        if (!d->overlay[i].busy) { d->overlay[i].busy = 1; slot = i; break; }
    }
    pthread_mutex_unlock(&d->mu);
    if (slot < 0) {
        d->disp_drops++;
        if (d->disp_drops == 1 || d->disp_drops % 600 == 0)
            vlog("hwdec: overlay buffers busy; dropped %u frames", d->disp_drops);
        return 0;
    }
    void *frame = d->overlay[slot].desc;
    uint64_t t0 = now_us();
    int r = d->pane_opaque
        ? yuv_opaque_to_rgbx_staged(img, d->staging, d->frame_size,
                                    d->overlay[slot].va, d->pane_stride, d->cvt_threads)
        : yuv_packed_alpha_to_bgra_staged(img, d->staging, d->frame_size,
                                          d->overlay[slot].va, d->pane_stride, d->cvt_threads, d->pane_alpha);
    if (r != 0) {
        overlay_release(d, frame);
        overlay_error(d, "pane frame conversion failed");
        return -1;
    }
    uint64_t t1 = now_us();
    r = hal.mem_sync(PTR(frame, FRM_MEM), 1);
    if (r != 0) {
        vlog("hwdec: overlay buffer cache sync: %d", r);
        overlay_release(d, frame);
        overlay_error(d, "output buffer cache sync failed");
        return -1;
    }
    U64(frame, FRM_PTS) = pts;
    uint64_t t2 = now_us();
    r = dispout_push(d->disp, frame);
    d->pane_us[0] += t1 - t0; d->pane_us[1] += t2 - t1; d->pane_us[2] += now_us() - t2;
    if (++d->pane_timed == 600) {
        vlog("hwdec: pane stages per frame: convert %llu us, cache sync %llu us, push %llu us",
             (unsigned long long)(d->pane_us[0] / 600), (unsigned long long)(d->pane_us[1] / 600),
             (unsigned long long)(d->pane_us[2] / 600));
        memset(d->pane_us, 0, sizeof(d->pane_us));
        d->pane_timed = 0;
    }
    if (r != DISPOUT_PUSH_OK) {
        overlay_release(d, frame);
        if (r == DISPOUT_PUSH_REJECTED && ++d->overlay_rejections < OVERLAY_REJECT_LIMIT) {
            d->disp_drops++;
            return 0;
        }
        if (r == DISPOUT_PUSH_REJECTED)
            vlog("hwdec: display rejected %u consecutive output frames", d->overlay_rejections);
        overlay_error(d, "display rejected the output frame");
        return -1;
    }
    d->overlay_rejections = 0;
    d->retained_img = *img;
    d->retained_img.y = d->staging;
    d->retained_img.u = d->staging + (size_t)img->ys * img->h;
    d->retained_img.v = d->retained_img.u + 1;
    d->retained_pts = pts;
    uint8_t *previous = d->retained_staging;
    d->retained_staging = d->staging;
    d->staging = previous;
    return 1;
}

static int frame_full(hwdec_t *d, void *desc) {
    if (U32(desc, FRM_VALID_A) == 0 || U32(desc, FRM_VALID_B) == 0) {   /* empty hand-back */
        queue_return(d, desc);
        return 0;
    }
    int idx = -1;
    for (int i = 0; i < d->nframe; i++) if (d->frame[i] == desc) { idx = i; break; }
    if (idx < 0) { vlog("hwdec: unknown frame desc %p", desc); return 0; }
    if (d->sync_dir >= 0) {
        int r = hal.mem_sync(PTR(desc, FRM_MEM), (uint32_t)d->sync_dir);
        if (r != 0 && d->pane_output) {
            vlog("hwdec: decoder frame cache sync: %d", r);
            overlay_error(d, "decoder frame cache sync failed");
            queue_return(d, desc);
            return 0;
        }
    }
    int w = (int)U32(desc, FRM_W), h = (int)U32(desc, FRM_H);
    int stride = (int)U32(desc, FRM_STRIDE), uvoff = (int)U32(desc, FRM_UV_OFF);
    if (d->frames_out == 0)
        vlog("hwdec: first frame fmt=%u %dx%d stride=%d uvoff=%d pts=%llu",
             U32(desc, FRM_FMT), w, h, stride, uvoff, (unsigned long long)U64(desc, FRM_PTS));
    if (w > 0 && h > 0 && stride >= w && (int64_t)uvoff >= (int64_t)stride * h &&
        (!d->pane_output || (U32(desc, FRM_FMT) == PIXFMT_NV12 &&
         (int64_t)uvoff + (int64_t)stride * ((h + 1LL) / 2) <= d->frame_size)) && d->frame_va[idx]) {
        if (d->pane_output && d->skip_output && d->skip_output(d->user)) {
            unsigned skipped = ++d->output_skips;
            if (skipped == 1 || skipped % 600 == 0)
                vlog("hwdec: skipped %u decoded frames under queue pressure", skipped);
            queue_return(d, desc);
            return 0;
        }
        const uint8_t *va = d->frame_va[idx];
        yuv_img_t img = { .y = va, .u = va + uvoff, .v = va + uvoff + 1,
                          .ys = stride, .us = stride, .vs = stride, .cstep = 2,
                          .w = w, .h = h, .full_range = 0 };
        if (d->prepare_output && d->prepare_output(d->user) < 0) {
            queue_return(d, desc);
            return 0;
        }
        if (d->pane_output && d->pane_direct) {
            if (!pane_dimensions_match(d, w, h)) {
                overlay_error(d, "decoded dimensions do not match the pane");
                queue_return(d, desc);
                return 0;
            }
            int r = dispout_push(d->disp, desc);
            if (r == DISPOUT_PUSH_OK) {
                /* the display owns the frame now; on_pop → disp_release → decoder */
                d->overlay_rejections = 0;
                d->cb(d->user, NULL, U64(desc, FRM_PTS), 0);
                d->frames_out++;
                return 0;
            }
            queue_return(d, desc);
            if (r == DISPOUT_PUSH_REJECTED && ++d->overlay_rejections < OVERLAY_REJECT_LIMIT) {
                d->disp_drops++;
                return 0;
            }
            if (r == DISPOUT_PUSH_REJECTED)
                vlog("hwdec: display rejected %u consecutive output frames", d->overlay_rejections);
            overlay_error(d, "display rejected the output frame");
            return 0;
        }
        if (d->pane_output) {
            uint64_t start = now_us();
            /* Decoder ION is uncached; conversion workers stage their own rows. */
            pthread_mutex_lock(&d->staging_mu);
            int presented = overlay_present(d, &img, U64(desc, FRM_PTS));
            pthread_mutex_unlock(&d->staging_mu);
            if (presented > 0) {
                d->cb(d->user, NULL, U64(desc, FRM_PTS), (uint32_t)(now_us() - start));
                d->frames_out++;
            }
            queue_return(d, desc);
            return 0;
        }
        if (d->disp) {
            if (dispout_push(d->disp, desc) == 0) {
                /* the display owns the frame now; on_pop → disp_release → decoder */
                d->cb(d->user, NULL, U64(desc, FRM_PTS), 0);
                d->frames_out++;
                return 0;
            }
            /* A rejected display frame is dropped.
             * Never fall back to CPU conversion here — reading the uncached
             * decoder buffer takes ~430 ms and stalls the whole pipeline.    */
            d->disp_drops++;
            queue_return(d, desc);
            return 0;
        }
        d->cb(d->user, &img, U64(desc, FRM_PTS), 0);
        d->frames_out++;
    } else if (d->pane_output) {
        overlay_error(d, "invalid decoded frame descriptor");
    }
    queue_return(d, desc);
    return 0;
}

static int cb_frame_full(void *user, void *desc) {
    hwdec_t *d = user;
    pthread_mutex_lock(&d->mu);
    if (d->stop) { pthread_mutex_unlock(&d->mu); return 0; }
    d->callbacks_active++;
    pthread_mutex_unlock(&d->mu);
    int valid = U32(desc, FRM_VALID_A) && U32(desc, FRM_VALID_B);
    uint64_t pts = U64(desc, FRM_PTS);
    int r = frame_full(d, desc);
    pthread_mutex_lock(&d->mu);
    if (valid) {
        d->last_output_pts = pts;
        if (d->last_output_pts == d->last_submitted_pts) d->last_frame_pending = 0;
    }
    --d->callbacks_active;
    pthread_cond_broadcast(&d->callbacks_cv);
    pthread_mutex_unlock(&d->mu);
    return r;
}

static int cb_error(void *user, void *desc, uint32_t ev) {
    hwdec_t *d = user;
    d->errors++;
    if (d->errors < 20) vlog("hwdec: decoder error event %u (desc %p)", ev, desc);
    if (d->pane_output) overlay_error(d, "decoder reported an error");
    return 0;
}

static int give_frame(hwdec_t *d, void *frame) {
    void *req[2] = { frame, NULL };
    return hal.decstrm(d->vdec, req);
}

static void *return_thread(void *arg) {
    hwdec_t *d = arg;
    for (;;) {
        pthread_mutex_lock(&d->mu);
        while (!d->ret_n && !d->stop) pthread_cond_wait(&d->ret_cv, &d->mu);
        if (d->stop) { pthread_mutex_unlock(&d->mu); break; }
        void *f = d->ret_q[--d->ret_n];
        d->returning = 1;
        pthread_mutex_unlock(&d->mu);
        int r = give_frame(d, f);
        if (r != 0) vlog("hwdec: return frame: %d", r);
        pthread_mutex_lock(&d->mu);
        d->returning = 0;
        pthread_cond_broadcast(&d->callbacks_cv);
        pthread_mutex_unlock(&d->mu);
    }
    return NULL;
}

static void disp_release(void *user, void *fb) { queue_return((hwdec_t *)user, fb); }

static int align_up(int v, int a) { return (v + a - 1) / a * a; }

static int overlay_open(hwdec_t *d, const hwdec_tune_t *tune) {
    d->pane_output = 1;
    d->pane_opaque = tune->pane_opaque;
    d->pane_alpha = tune->pane_alpha;
    d->pane_w = tune->pane_w;
    d->pane_h = tune->pane_h;
    d->cvt_threads = tune->cvt_threads;
    d->output_error = tune->output_error;
    if (d->pane_w <= 0 || d->pane_h <= 0 || d->pane_w > 1920 || d->pane_h > 1080 ||
        !pane_dimensions_match(d, d->w, d->h)) {
        vlog("hwdec: invalid pane dimensions");
        return -1;
    }
    d->pane_direct = d->pane_opaque && !getenv("EYEPIECE_PANE_RGB");
    if (d->pane_direct) {
        d->disp = dispout_open_pane(d->pane_w, d->pane_h, tune->pane_x, tune->pane_y,
                                    1, d->pane_alpha, disp_release, d);
        if (!d->disp) { vlog("hwdec: GFX1 unavailable (dji_arhome may own it)"); return -1; }
        vlog("hwdec: opaque pane scans out decoder NV12 directly (no colour conversion)");
        return 0;
    }
    d->pane_stride = align_up(d->pane_w * 4, 256);
    d->staging = malloc(d->frame_size);
    d->retained_staging = malloc(d->frame_size);
    if (!d->staging || !d->retained_staging) {
        vlog("hwdec: pane staging allocation failed"); return -1;
    }
    uint32_t size = (uint32_t)(d->pane_stride * d->pane_h);
    for (int i = 0; i < OVERLAY_BUFS; i++) {
        void *f = calloc(1, FRM_DESC_SZ);
        if (!f) { vlog("hwdec: overlay descriptor allocation failed"); return -1; }
        d->overlay[i].desc = f;
        int r = hal.mem_alloc(d->ion, (void **)((uint8_t *)f + FRM_MEM), size, 0x80, 0, 0);
        if (r != 0) { vlog("hwdec: overlay buffer %d allocation: %d", i, r); return -1; }
        r = hal.mem_map(PTR(f, FRM_MEM), &d->overlay[i].va);
        if (r != 0 || !d->overlay[i].va) {
            vlog("hwdec: overlay buffer %d map: %d", i, r); return -1;
        }
        U32(f, FRM_FMT) = d->pane_opaque ? PIXFMT_RGB888_I32 : PIXFMT_ARGB8_I32;
        U32(f, FRM_W) = (uint32_t)d->pane_w;
        U32(f, FRM_H) = (uint32_t)d->pane_h;
        U32(f, FRM_STRIDE) = (uint32_t)d->pane_stride;
        U32(f, FRM_LUMA_H) = (uint32_t)d->pane_h;
        U32(f, FRM_VALID_A) = size;
        U32(f, FRM_PLANES) = 1;
    }
    d->disp = dispout_open_pane(d->pane_w, d->pane_h, tune->pane_x, tune->pane_y,
                                d->pane_opaque, d->pane_alpha, overlay_release, d);
    if (!d->disp) { vlog("hwdec: GFX1 unavailable (dji_arhome may own it)"); return -1; }
    return 0;
}

hwdec_t *hwdec_open(int codec, int w, int h, int nframes, int sync_dir,
                    const hwdec_tune_t *tune, hwdec_frame_cb cb, void *user) {
    if (load_hal(tune && tune->direct_vops) < 0) return NULL;
    hwdec_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    atomic_init(&d->stop, 0);
    atomic_init(&d->overlay_failed, 0);
    atomic_init(&d->disp_drops, 0);
    atomic_init(&d->frames_out, 0);
    atomic_init(&d->errors, 0);
    atomic_init(&d->output_skips, 0);
    pthread_mutex_init(&d->mu, NULL);
    pthread_mutex_init(&d->staging_mu, NULL);
    pthread_cond_init(&d->strm_cv, NULL);
    pthread_cond_init(&d->ret_cv, NULL);
    pthread_cond_init(&d->callbacks_cv, NULL);
    d->cb = cb; d->user = user; d->w = w; d->h = h; d->sync_dir = sync_dir;
    if (tune) {
        d->prepare_output = tune->prepare_output;
        d->skip_output = tune->skip_output;
    }
    d->nstrm = 8;
    d->nframe = nframes < 4 ? 4 : nframes > MAX_BUF ? MAX_BUF : nframes;
    d->strm_size = 1u << 20;

    /* Check VIDEO0 ownership before starting the decoder, whose teardown is
     * unsafe on this firmware. Overlay sessions acquire GFX1 separately. */
    if (tune && tune->direct_vops && !tune->pane_output) {
        d->disp = dispout_open(tune->direct_vops, w, h, tune->stereo,
                               disp_release, d);
        if (!d->disp) {
            vlog("hwdec: direct scan-out unavailable");
            pthread_cond_destroy(&d->ret_cv);
            pthread_cond_destroy(&d->strm_cv);
            pthread_cond_destroy(&d->callbacks_cv);
            pthread_mutex_destroy(&d->staging_mu);
            pthread_mutex_destroy(&d->mu);
            free(d);
            return NULL;
        }
    }

    uint32_t zero = 0;
    int r = hal.device_open("/dev/ion0", &zero, &d->ion);
    if (r == 0) r = hal.device_start(d->ion, &zero);
    if (r != 0) { vlog("hwdec: ion0 open/start: %d", r); return NULL; }

    vdec_open_cfg_t oc;
    memset(&oc, 0, sizeof(oc));
    oc.codec = codec == eyepiece_rpc_v1_Codec_CODEC_HEVC ? 1 : 0;        /* 0 = H264, 1 = H265 */
    oc.one = 1;
    oc.strm_empty_cb = (void *)cb_strm_empty;
    oc.frame_start_cb = (void *)cb_frame_start;
    oc.frame_full_cb = (void *)cb_frame_full;
    oc.error_cb = (void *)cb_error;
    for (int i = 0; i < 4; i++) oc.user[i] = d;
    r = hal.device_open("/dev/vdec0", &oc, &d->vdec);
    if (r != 0) { vlog("hwdec: vdec0 open: %d", r); return NULL; }

    /* stride / size rules from test_vdec instance_task (NV12, align ≥ 64).
     * 256, not 64: the display's VIDEO0 plane rejects NV12 whose stride isn't
     * 256-aligned (kernel: "lcdc_check_frameinfo_v2: stride not aligned to
     * 256: 1920" → config frame fail → black screen). 1920 → stride 2048.   */
    const int align = 256;
    int w16 = align_up(w, 16), h16 = align_up(h, 16);
    d->stride = align_up(w16, align);
    long pg = sysconf(_SC_PAGESIZE);
    uint32_t fsize = (uint32_t)align_up((int)(1.5 * d->stride * h16) + (int)pg, (int)pg);
    d->frame_size = fsize;

    for (int i = 0; i < d->nstrm; i++) {
        void *s = calloc(1, STRM_DESC_SZ);
        if (!s || hal.mem_alloc(d->ion, (void **)((uint8_t *)s + STRM_MEM), d->strm_size, 0x80, 0, 0) != 0) {
            vlog("hwdec: stream buffer %d alloc failed", i); return NULL;
        }
        U32(s, STRM_W) = (uint32_t)w; U32(s, STRM_H) = (uint32_t)h;
        r = hal.mem_map(PTR(s, STRM_MEM), &d->strm_va[i]);
        if (r != 0 || !d->strm_va[i]) {
            vlog("hwdec: stream buffer %d map: %d", i, r); return NULL;
        }
        d->strm[i] = s;
    }
    for (int i = 0; i < d->nframe; i++) {
        void *f = calloc(1, FRM_DESC_SZ);
        if (!f || hal.mem_alloc(d->ion, (void **)((uint8_t *)f + FRM_MEM), fsize, 0x80, 0, 0) != 0) {
            vlog("hwdec: frame buffer %d alloc failed (%u B)", i, fsize); return NULL;
        }
        U32(f, FRM_FMT) = PIXFMT_NV12;
        U32(f, FRM_W) = (uint32_t)w; U32(f, FRM_H) = (uint32_t)h;
        U32(f, FRM_STRIDE) = (uint32_t)d->stride; U32(f, 0x44) = 0;
        U32(f, FRM_LUMA_H) = (uint32_t)h;
        U32(f, FRM_UV_STRIDE) = (uint32_t)d->stride;
        U32(f, FRM_UV_OFF) = (uint32_t)(h * d->stride);
        U32(f, FRM_UV_H) = (uint32_t)(h / 2);
        U32(f, FRM_PLANES) = 2;
        r = hal.mem_map(PTR(f, FRM_MEM), &d->frame_va[i]);
        if (r != 0 || !d->frame_va[i]) {
            vlog("hwdec: frame buffer %d map: %d", i, r); return NULL;
        }
        d->frame[i] = f;
    }
    d->pool = (vdec_pool_t){ d->frame, d->strm, (uint32_t)d->nframe, (uint32_t)d->nstrm };
    vdec_start_cfg_t sc;
    memset(&sc, 0, sizeof(sc));
    sc.pool = &d->pool;
    sc.stride_align = align;                         /* scale 0×0 = no scaling */
    sc.no_reorder = tune ? (uint8_t)tune->no_reorder : 1;
    if (tune) { sc.flag24 = (uint32_t)tune->flag24; sc.val2c = (uint32_t)tune->val2c; }
    r = hal.device_start(d->vdec, &sc);
    if (r != 0) { vlog("hwdec: vdec0 start: %d", r); return NULL; }
    if (tune && tune->pane_output) {
        if (overlay_open(d, tune) != 0) {
            hwdec_stop_overlay(d);
            return NULL;
        }
    }
    for (int i = 0; i < d->nframe; i++)
        if ((r = give_frame(d, d->frame[i])) != 0) {
            vlog("hwdec: give frame %d: %d", i, r);
            if (d->pane_output) hwdec_stop_overlay(d);
            return NULL;
        }
    pthread_t t;
    r = pthread_create(&t, NULL, return_thread, d);
    if (r != 0) {
        vlog("hwdec: return thread creation failed: %d", r);
        if (d->pane_output) hwdec_stop_overlay(d);
        return NULL;
    }
    if (d->pane_output) { d->ret_thread = t; d->ret_thread_started = 1; }
    else pthread_detach(t);
    vlog("hwdec: VC8000D %s %dx%d ready (stride %d, %d×%u B frames, %d×1 MiB streams, sync %d, "
         "no_reorder %u, +0x24=%u, +0x2c=%u)", oc.codec ? "HEVC" : "H264", w, h, d->stride, d->nframe,
         fsize, d->nstrm, sync_dir, sc.no_reorder, sc.flag24, sc.val2c);
    return d;
}

int hwdec_submit(hwdec_t *d, const uint8_t *au, int len, uint64_t pts) {
    if ((uint32_t)len > d->strm_size) { vlog("hwdec: AU too big (%d)", len); return -1; }
    struct timespec dl;
    clock_gettime(CLOCK_REALTIME, &dl);
    dl.tv_nsec += 200 * 1000000L;
    if (dl.tv_nsec >= 1000000000L) { dl.tv_sec++; dl.tv_nsec -= 1000000000L; }
    pthread_mutex_lock(&d->mu);
    int idx = -1;
    for (;;) {
        for (int i = 0; i < d->nstrm; i++) if (!d->strm_busy[i]) { idx = i; break; }
        if (idx >= 0) break;
        if (pthread_cond_timedwait(&d->strm_cv, &d->mu, &dl) != 0) break;
    }
    if (idx >= 0) d->strm_busy[idx] = 1;
    pthread_mutex_unlock(&d->mu);
    if (idx < 0) { vlog("hwdec: no free stream buffer (decoder stalled?)"); return -1; }
    void *s = d->strm[idx];
    memcpy(d->strm_va[idx], au, (size_t)len);
    U32(s, STRM_LEN) = (uint32_t)len;
    U64(s, STRM_PTS) = pts;
    void *req[2] = { NULL, s };
    int r = hal.decstrm(d->vdec, req);
    if (r != 0) {
        vlog("hwdec: decstrm: %d", r);
        pthread_mutex_lock(&d->mu); d->strm_busy[idx] = 0; pthread_mutex_unlock(&d->mu);
        return -1;
    }
    pthread_mutex_lock(&d->mu);
    d->last_submitted_pts = pts;
    d->last_frame_pending = d->last_output_pts != pts;
    pthread_mutex_unlock(&d->mu);
    return 0;
}

int hwdec_move_overlay(hwdec_t *d, int x, int y) {
    if (!d || !d->pane_output || d->stop) return -1;
    return dispout_move_pane(d->disp, x, y);
}

int hwdec_set_overlay_alpha(hwdec_t *d, uint8_t alpha) {
    if (!d || !d->pane_output || d->stop || d->overlay_failed) return -1;
    pthread_mutex_lock(&d->staging_mu);
    uint8_t previous = d->pane_alpha;
    if (previous == alpha) {
        pthread_mutex_unlock(&d->staging_mu);
        return 0;
    }
    if (d->pane_opaque) {
        int r = dispout_set_pane_alpha(d->disp, alpha);
        if (r == 0) d->pane_alpha = alpha;
        pthread_mutex_unlock(&d->staging_mu);
        return r;
    }
    d->pane_alpha = alpha;
    int failed = d->retained_img.y && overlay_present(d, &d->retained_img, d->retained_pts) <= 0;
    if (failed) {
        d->pane_alpha = previous;
        vlog("hwdec: pane opacity update could not present retained frame");
    }
    pthread_mutex_unlock(&d->staging_mu);
    return failed ? -1 : 0;
}

static int overlay_drained(hwdec_t *d) {
    for (int i = 0; i < d->nstrm; i++)
        if (d->strm_busy[i]) return 0;
    return !d->last_frame_pending && !d->callbacks_active && !d->ret_n && !d->returning;
}

int hwdec_drain_overlay(hwdec_t *d) {
    struct timespec deadline;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += 3;
    pthread_mutex_lock(&d->mu);
    while (!overlay_drained(d)) {
        int r = pthread_cond_timedwait(&d->callbacks_cv, &d->mu, &deadline);
        if (r != 0 && !overlay_drained(d)) {
            pthread_mutex_unlock(&d->mu);
            vlog("hwdec: overlay decoder did not drain");
            return -1;
        }
    }
    pthread_mutex_unlock(&d->mu);
    return 0;
}

/* Process exits after an overlay session: retain ION objects until exit rather
 * than freeing buffers that DJI's HAL may still reference. */
static int stop_overlay(hwdec_t *d) {
    if (!d) return 0;
    /* HAL stop holds its device lock while waiting for EOS, so outstanding
     * frames must be returned before stopping the return thread. */
    if (hwdec_drain_overlay(d) != 0) return -1;
    pthread_mutex_lock(&d->mu);
    d->stop = 1;
    pthread_cond_broadcast(&d->ret_cv);
    pthread_mutex_unlock(&d->mu);
    if (d->ret_thread_started) {
        pthread_join(d->ret_thread, NULL);
        d->ret_thread_started = 0;
    }
    vlog("hwdec: stopping overlay decoder");
    int r = stop_device(d->vdec);
    if (r != 0) {
        vlog("hwdec: decoder stop failed: %d; retaining GFX1 until exit", r);
        return -1;
    }
    pthread_mutex_lock(&d->mu);
    while (d->callbacks_active) pthread_cond_wait(&d->callbacks_cv, &d->mu);
    pthread_mutex_unlock(&d->mu);
    vlog("hwdec: releasing overlay display");
    if (dispout_close_checked(d->disp) != 0) {
        vlog("hwdec: display release failed; retaining decoder buffers until exit");
        return -1;
    }
    d->disp = NULL;
    return 0;
}

void hwdec_stop_overlay(hwdec_t *d) {
    (void)stop_overlay(d);
}

int hwdec_reconfigure_close(hwdec_t *d) {
    if (!d) return 0;
    if (!d->pane_output || stop_overlay(d) != 0) return -1;
    int r = hal.device_close(d->vdec);
    if (r != 0) {
        vlog("hwdec: decoder close failed: %d", r);
        return -1;
    }
    for (int i = 0; i < d->nstrm; i++) {
        r = hal.mem_free(PTR(d->strm[i], STRM_MEM));
        if (r != 0) { vlog("hwdec: stream buffer release failed: %d", r); return -1; }
    }
    for (int i = 0; i < d->nframe; i++) {
        r = hal.mem_free(PTR(d->frame[i], FRM_MEM));
        if (r != 0) { vlog("hwdec: frame buffer release failed: %d", r); return -1; }
    }
    for (int i = 0; i < OVERLAY_BUFS; i++) {
        if (!d->overlay[i].desc) continue;
        r = hal.mem_free(PTR(d->overlay[i].desc, FRM_MEM));
        if (r != 0) { vlog("hwdec: overlay buffer release failed: %d", r); return -1; }
    }
    r = stop_device(d->ion);
    if (r != 0) { vlog("hwdec: ion stop failed: %d", r); return -1; }
    r = hal.device_close(d->ion);
    if (r != 0) { vlog("hwdec: ion close failed: %d", r); return -1; }
    free(d->staging);
    free(d->retained_staging);
    d->staging = NULL;
    d->retained_staging = NULL;
    return 0;
}
