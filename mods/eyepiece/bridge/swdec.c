// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * swdec.c — H.264/HEVC software decode with the goggles' own
 * /system/lib64/libavcodec.so (FFmpeg lavc 56.13, CyanogenMod build) loaded
 * via dlopen, plus a threaded YUV420 → BGRA converter that writes straight
 * into the fb_shared slot the natartos overlay injects into the display.
 *
 * We deliberately avoid FFmpeg headers: the ABI of that 2014-era build is
 * pinned by hand below and every assumption is sanity-checked at runtime
 * (see swdec_open), so a mismatch fails cleanly instead of crashing.
 */
#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "bridge.h"

/* ---- libavcodec / libavutil (lavc 56 / lavu 54) ABI subset ------------- */
#define AV_NOPTS_VALUE ((int64_t)0x8000000000000000ULL)
/* AVPacket — DJI's build differs from upstream 2.5 (size@32, sizeof 0x58):
 * buf@0 pts@8 dts@16 data@24 ?@32(8) flags?@40 size@44 … pos@96, sizeof 0x88
 * (from the device's avcodec_decode_video2 / av_new_packet).
 * Only data and size are written by us; the packet buffer is zeroed.      */
#define PKT_OFF_PTS   8
#define PKT_OFF_DATA  24
#define PKT_OFF_SIZE  44
/* AVFrame (2.5): data[8]@0 linesize[8]@64 extended_data@96 width@104 height@108
 *                nb_samples@112 format@116                                  */
#define FRM_OFF_LINESIZE 64
#define FRM_OFF_WIDTH    104
#define FRM_OFF_HEIGHT   108
#define FRM_OFF_FORMAT   116

static struct {
    void  (*register_all)(void);
    void *(*find_decoder_by_name)(const char *);
    void *(*alloc_context3)(const void *);
    int   (*open2)(void *, const void *, void **);
    int   (*decode_video2)(void *, void *, int *, const void *);
    void  (*init_packet)(void *);
    void *(*frame_alloc)(void);
    int   (*opt_set)(void *, const char *, const char *, int);
    int   (*opt_set_int)(void *, const char *, int64_t, int);
    int   (*get_pix_fmt)(const char *);
    void  (*log_set_level)(int);
} av;

struct swdec {
    void *codec, *ctx, *frame;
    unsigned char pkt[256];          /* > sizeof(AVPacket) in any lavc 56   */
    int fmt_yuv420p, fmt_yuvj420p;
    unsigned n_calls, n_err, n_nopic;
};

#define LOAD(lib, field, name) do { \
        *(void **)&av.field = dlsym(lib, name); \
        if (!av.field) { vlog("swdec: missing %s", name); return -1; } \
    } while (0)

static int load_ffmpeg(void) {
    static int loaded = 0;
    if (loaded) return loaded > 0 ? 0 : -1;
    loaded = -1;
    void *avu = dlopen("libavutil.so", RTLD_NOW | RTLD_GLOBAL);
    void *avc = dlopen("libavcodec.so", RTLD_NOW | RTLD_GLOBAL);
    if (!avu || !avc) { vlog("swdec: dlopen failed: %s", dlerror()); return -1; }
    LOAD(avc, register_all, "avcodec_register_all");
    LOAD(avc, find_decoder_by_name, "avcodec_find_decoder_by_name");
    LOAD(avc, alloc_context3, "avcodec_alloc_context3");
    LOAD(avc, open2, "avcodec_open2");
    LOAD(avc, decode_video2, "avcodec_decode_video2");
    LOAD(avc, init_packet, "av_init_packet");
    LOAD(avu, frame_alloc, "av_frame_alloc");
    LOAD(avu, opt_set, "av_opt_set");
    LOAD(avu, opt_set_int, "av_opt_set_int");
    LOAD(avu, get_pix_fmt, "av_get_pix_fmt");
    LOAD(avu, log_set_level, "av_log_set_level");
    /* Don't call avcodec_version(): DJI's libavcodec has a patched codec-ID
     * enum, so its own self-check (av_assert0(CODEC_ID_CLLC == AV_CODEC_ID_CLLC),
     * utils.c:3163) aborts the process. Pin the ABI by symbol set instead:
     * 56.x has decode_video2 (already loaded) and no send_packet (57.37+).  */
    if (dlsym(avc, "avcodec_send_packet")) {
        vlog("swdec: libavcodec >= 57.37 (send_packet API), ABI pinned for 56 — refusing");
        return -1;
    }
    vlog("swdec: libavcodec ABI 56 (no version call: it aborts on this build)");
    av.log_set_level(getenv("EYEPIECE_AVLOG") ? atoi(getenv("EYEPIECE_AVLOG")) : 24 /* AV_LOG_WARNING */);
    av.register_all();
    loaded = 1;
    return 0;
}

swdec_t *swdec_open(int codec, int threads, int fast) {
    if (load_ffmpeg() < 0) return NULL;
    swdec_t *d = calloc(1, sizeof(*d));
    if (!d) return NULL;
    const char *name = codec == EP_CODEC_HEVC ? "hevc" : "h264";
    d->codec = av.find_decoder_by_name(name);
    if (!d->codec) { vlog("swdec: no %s decoder", name); free(d); return NULL; }
    d->ctx = av.alloc_context3(d->codec);
    if (!d->ctx) { free(d); return NULL; }
    /* Low latency: slice threads only (frame threads add N-1 frames of delay),
     * no output reordering.  Slice threading needs a multi-slice encode.    */
    av.opt_set_int(d->ctx, "threads", threads, 0);
    av.opt_set(d->ctx, "thread_type", "slice", 0);
    av.opt_set(d->ctx, "flags", "+low_delay", 0);
    av.opt_set(d->ctx, "flags2", "+fast", 0);
    if (fast) av.opt_set(d->ctx, "skip_loop_filter", "all", 0);
    if (av.open2(d->ctx, d->codec, NULL) < 0) { vlog("swdec: avcodec_open2 failed"); free(d); return NULL; }
    d->frame = av.frame_alloc();
    av.init_packet(d->pkt);
    if (!d->frame || *(int64_t *)(d->pkt + PKT_OFF_PTS) != AV_NOPTS_VALUE) {
        vlog("swdec: AVPacket layout check failed (pts@8 != NOPTS) — ABI mismatch");
        free(d);
        return NULL;
    }
    d->fmt_yuv420p = av.get_pix_fmt("yuv420p");
    d->fmt_yuvj420p = av.get_pix_fmt("yuvj420p");
    vlog("swdec: %s ready, %d slice threads%s", name, threads, fast ? ", fast" : "");
    return d;
}

int swdec_decode(swdec_t *d, const uint8_t *au, int len, fbout_t *fb,
                 int cvt_threads, uint32_t *decode_us, uint32_t *convert_us) {
    uint64_t t0 = now_us();
    *(const uint8_t **)(d->pkt + PKT_OFF_DATA) = au;
    *(int *)(d->pkt + PKT_OFF_SIZE) = len;
    int got = 0;
    int r = av.decode_video2(d->ctx, d->frame, &got, d->pkt);
    uint64_t t1 = now_us();
    *decode_us = (uint32_t)(t1 - t0);
    d->n_calls++;
    if (r < 0) d->n_err++; else if (!got) d->n_nopic++;
    if (d->n_calls <= 5 || (r < 0 && (d->n_err <= 5 || d->n_err % 300 == 0)) ||
        (!got && r >= 0 && d->n_nopic % 300 == 0))
        vlog("swdec: call %u len=%d ret=%d got=%d (%u errors, %u no-picture) %u us",
             d->n_calls, len, r, got, d->n_err, d->n_nopic, *decode_us);
    if (r < 0) return -1;
    if (!got) return 0;

    uint8_t **data = (uint8_t **)d->frame;
    int *ls = (int *)((uint8_t *)d->frame + FRM_OFF_LINESIZE);
    int w = *(int *)((uint8_t *)d->frame + FRM_OFF_WIDTH);
    int h = *(int *)((uint8_t *)d->frame + FRM_OFF_HEIGHT);
    int fmt = *(int *)((uint8_t *)d->frame + FRM_OFF_FORMAT);
    if (w < 16 || h < 16 || w > 4096 || h > 4096 || ls[0] < w || !data[0] || !data[1] || !data[2]) {
        vlog("swdec: AVFrame layout check failed (w=%d h=%d ls=%d)", w, h, ls[0]);
        return -1;
    }
    if (fmt != d->fmt_yuv420p && fmt != d->fmt_yuvj420p) {
        vlog("swdec: unsupported pixel format %d (need yuv420p)", fmt);
        return -1;
    }
    yuv_img_t img = {
        .y = data[0], .u = data[1], .v = data[2],
        .ys = ls[0], .us = ls[1], .vs = ls[2], .cstep = 1,
        .w = w, .h = h, .full_range = (fmt == d->fmt_yuvj420p),
    };
    yuv_to_fb(fb, &img, cvt_threads);
    *convert_us = (uint32_t)(now_us() - t1);
    fbout_publish(fb);
    return 1;
}
