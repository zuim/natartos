// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * hwdec.c — hardware H.264/HEVC decode on the goggles' Verisilicon VC8000D
 * through DJI's HAL (/dev/vdec0 + /dev/ion0), reproducing the flow of the
 * stock /system/bin/test_vdec.  EXPERIMENTAL — written from static analysis.
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

#define U32(p, o) (*(uint32_t *)((uint8_t *)(p) + (o)))
#define U64(p, o) (*(uint64_t *)((uint8_t *)(p) + (o)))
#define PTR(p, o) (*(void **)((uint8_t *)(p) + (o)))

typedef struct { const char *name; void *attach, *detach, *rsv; } hal_dev_t;

static struct {
    int  (*initialize)(hal_dev_t *tbl);
    int  (*device_open)(const char *name, void *cfg, void **h);
    int  (*device_start)(void *h, void *cfg);
    int  (*device_stop)(void *h);
    int  (*device_close)(void *h);
    int  (*mem_alloc)(void *ion, void **mem, uint32_t size, uint32_t align, uint32_t a5, uint32_t a6);
    int  (*mem_map)(void *mem, void **va);
    int  (*mem_sync)(void *mem, uint32_t dir);
    int  (*mem_free)(void *mem);
    int  (*decstrm)(void *vdec, void *req);
} hal;

static hal_dev_t g_devs[4];

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
    int w, h, stride;
    unsigned disp_drops;
    int sync_dir;                 /* <0 = no cache sync before reading frames */
    hwdec_frame_cb cb; void *user;
    dispout_t *disp;              /* zero-copy scan-out, or NULL               */
    pthread_mutex_t mu;
    pthread_cond_t strm_cv, ret_cv;
    void *ret_q[MAX_BUF * 2]; int ret_n;
    int stop;
    uint32_t frames_out, errors;
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

/* ---- decoder callbacks (run on the decoder's threads) ------------------ */

static int cb_strm_empty(void *user, void *desc) {
    hwdec_t *d = user;
    pthread_mutex_lock(&d->mu);
    for (int i = 0; i < d->nstrm; i++)
        if (d->strm[i] == desc) { d->strm_busy[i] = 0; break; }
    pthread_cond_signal(&d->strm_cv);
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

static int cb_frame_full(void *user, void *desc) {
    hwdec_t *d = user;
    if (d->stop) return 0;                     /* shutting down: buffers going away */
    if (U32(desc, FRM_VALID_A) == 0 || U32(desc, FRM_VALID_B) == 0) {   /* empty hand-back */
        queue_return(d, desc);
        return 0;
    }
    int idx = -1;
    for (int i = 0; i < d->nframe; i++) if (d->frame[i] == desc) { idx = i; break; }
    if (idx < 0) { vlog("hwdec: unknown frame desc %p", desc); return 0; }
    if (d->sync_dir >= 0) hal.mem_sync(PTR(desc, FRM_MEM), (uint32_t)d->sync_dir);
    int w = (int)U32(desc, FRM_W), h = (int)U32(desc, FRM_H);
    int stride = (int)U32(desc, FRM_STRIDE), uvoff = (int)U32(desc, FRM_UV_OFF);
    if (d->frames_out == 0)
        vlog("hwdec: first frame fmt=%u %dx%d stride=%d uvoff=%d pts=%llu",
             U32(desc, FRM_FMT), w, h, stride, uvoff, (unsigned long long)U64(desc, FRM_PTS));
    if (w > 0 && h > 0 && stride >= w && uvoff >= stride * h && d->frame_va[idx]) {
        const uint8_t *va = d->frame_va[idx];
        yuv_img_t img = { .y = va, .u = va + uvoff, .v = va + uvoff + 1,
                          .ys = stride, .us = stride, .vs = stride, .cstep = 2,
                          .w = w, .h = h, .full_range = 0 };
        if (d->disp) {
            if (dispout_push(d->disp, desc) == 0) {
                /* the display owns the frame now; on_pop → disp_release → decoder */
                d->cb(d->user, NULL, U64(desc, FRM_PTS));
                d->frames_out++;
                return 0;
            }
            /* display queue full (-1001, bursts at 100 fps): drop this frame.
             * Never fall back to CPU conversion here — reading the uncached
             * decoder buffer takes ~430 ms and stalls the whole pipeline.    */
            d->disp_drops++;
            queue_return(d, desc);
            return 0;
        }
        d->cb(d->user, &img, U64(desc, FRM_PTS));
        d->frames_out++;
    }
    queue_return(d, desc);
    return 0;
}

static int cb_error(void *user, void *desc, uint32_t ev) {
    hwdec_t *d = user;
    d->errors++;
    if (d->errors < 20) vlog("hwdec: decoder error event %u (desc %p)", ev, desc);
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
        pthread_mutex_unlock(&d->mu);
        int r = give_frame(d, f);
        if (r != 0) vlog("hwdec: return frame: %d", r);
    }
    return NULL;
}

static void disp_release(void *user, void *fb) { queue_return((hwdec_t *)user, fb); }

static int align_up(int v, int a) { return (v + a - 1) / a * a; }

hwdec_t *hwdec_open(int codec, int w, int h, int nframes, int sync_dir,
                    const hwdec_tune_t *tune, hwdec_frame_cb cb, void *user) {
    if (load_hal(tune && tune->direct_vops) < 0) return NULL;
    hwdec_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    pthread_mutex_init(&d->mu, NULL);
    pthread_cond_init(&d->strm_cv, NULL);
    pthread_cond_init(&d->ret_cv, NULL);
    d->cb = cb; d->user = user; d->w = w; d->h = h; d->sync_dir = sync_dir;
    d->nstrm = 8;
    d->nframe = nframes < 4 ? 4 : nframes > MAX_BUF ? MAX_BUF : nframes;
    d->strm_size = 1u << 20;

    uint32_t zero = 0;
    int r = hal.device_open("/dev/ion0", &zero, &d->ion);
    if (r == 0) r = hal.device_start(d->ion, &zero);
    if (r != 0) { vlog("hwdec: ion0 open/start: %d", r); return NULL; }

    vdec_open_cfg_t oc;
    memset(&oc, 0, sizeof(oc));
    oc.codec = codec == EP_CODEC_HEVC ? 1 : 0;        /* 0 = H264, 1 = H265 */
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

    for (int i = 0; i < d->nstrm; i++) {
        void *s = calloc(1, STRM_DESC_SZ);
        if (!s || hal.mem_alloc(d->ion, (void **)((uint8_t *)s + STRM_MEM), d->strm_size, 0x80, 0, 0) != 0) {
            vlog("hwdec: stream buffer %d alloc failed", i); return NULL;
        }
        U32(s, STRM_W) = (uint32_t)w; U32(s, STRM_H) = (uint32_t)h;
        hal.mem_map(PTR(s, STRM_MEM), &d->strm_va[i]);
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
        hal.mem_map(PTR(f, FRM_MEM), &d->frame_va[i]);
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
    if (tune && tune->direct_vops) {
        d->disp = dispout_open(tune->direct_vops, w, h, disp_release, d);
        if (!d->disp) vlog("hwdec: direct scan-out unavailable → converting into fb_shared");
    }
    for (int i = 0; i < d->nframe; i++)
        if ((r = give_frame(d, d->frame[i])) != 0) { vlog("hwdec: give frame %d: %d", i, r); return NULL; }
    pthread_t t;
    pthread_create(&t, NULL, return_thread, d);
    pthread_detach(t);
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
    return 0;
}

void hwdec_close(hwdec_t *d) {
    if (!d) return;
    pthread_mutex_lock(&d->mu);
    d->stop = 1;
    pthread_cond_broadcast(&d->ret_cv);
    pthread_mutex_unlock(&d->mu);
    /* decoder first (no more output callbacks), then the display (stops
     * reading our buffers), then free: the old order crashed the decoder's
     * output thread on exit (coredump in vdec_output_loop).    */
    hal.device_stop(d->vdec);
    dispout_close(d->disp);
    if (d->disp_drops) vlog("hwdec: %u frames dropped (display queue full)", d->disp_drops);
    hal.device_close(d->vdec);
    for (int i = 0; i < d->nstrm; i++) hal.mem_free(PTR(d->strm[i], STRM_MEM));
    for (int i = 0; i < d->nframe; i++) hal.mem_free(PTR(d->frame[i], FRM_MEM));
    hal.device_stop(d->ion);
    hal.device_close(d->ion);
    /* descriptors intentionally leaked: the driver may still reference them */
}
