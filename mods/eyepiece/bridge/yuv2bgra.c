// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * YUV 4:2:0 (I420 or NV12) → BGRA for GFX1 panes, row-parallel on a small
 * persistent thread pool. BT.709, limited or full range.
 */
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include "bridge.h"
#if defined(__aarch64__) && defined(__ARM_NEON)
#include <arm_neon.h>
#endif

typedef struct {
    yuv_img_t img;
    uint8_t *dst; int ds;        /* destination (top-left of target rect)     */
    int dw, dh;                  /* target rect                               */
    int row0, row1;
    int alpha_x;                 /* packed grayscale alpha starts here, or 0 */
    uint8_t alpha;
    uint8_t *stage_y, *stage_uv;
} cvt_job_t;

static inline uint8_t clamp8(int v) { return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v; }

#if defined(__aarch64__) && defined(__ARM_NEON)
static inline void packed_neon_four(int16x4_t y, int16x4_t u, int16x4_t v,
                                    int cy, int cbu, int cgu, int cgv, int crv,
                                    int16x4_t *b, int16x4_t *g, int16x4_t *r) {
    int32x4_t Y = vmulq_n_s32(vmovl_s16(y), cy);
    int32x4_t U = vmovl_s16(u), V = vmovl_s16(v);
    *b = vqmovn_s32(vrshrq_n_s32(vmlaq_n_s32(Y, U, cbu), 16));
    *g = vqmovn_s32(vrshrq_n_s32(vmlaq_n_s32(vmlaq_n_s32(Y, U, cgu), V, cgv), 16));
    *r = vqmovn_s32(vrshrq_n_s32(vmlaq_n_s32(Y, V, crv), 16));
}

static void cvt_packed_nv12_neon(const cvt_job_t *j) {
    const yuv_img_t *s = &j->img;
    const int cy = s->full_range ? 65536 : 76309;
    const int cbu = s->full_range ? 121609 : 138438;
    const int cgu = s->full_range ? -12276 : -13975;
    const int cgv = s->full_range ? -30679 : -34925;
    const int crv = s->full_range ? 103206 : 117489;
    const int yoff = s->full_range ? 0 : 16;
    for (int row = j->row0; row < j->row1; row++) {
        const uint8_t *y = s->y + (long)row * s->ys;
        const uint8_t *uv = s->u + (long)(row >> 1) * s->us;
        uint8_t *dst = j->dst + (long)row * j->ds;
        int x = 0;
        for (; x + 8 <= j->dw; x += 8) {
            uint8x8_t chroma = vld1_u8(uv + x);
            uint8x8_t u = vuzp1_u8(chroma, chroma);
            uint8x8_t v = vuzp2_u8(chroma, chroma);
            int16x8_t yy = vreinterpretq_s16_u16(vsubl_u8(vld1_u8(y + x), vdup_n_u8(yoff)));
            int16x8_t uu = vreinterpretq_s16_u16(vsubl_u8(vzip1_u8(u, u), vdup_n_u8(128)));
            int16x8_t vv = vreinterpretq_s16_u16(vsubl_u8(vzip1_u8(v, v), vdup_n_u8(128)));
            int16x4_t b0, g0, r0, b1, g1, r1;
            packed_neon_four(vget_low_s16(yy), vget_low_s16(uu), vget_low_s16(vv),
                             cy, cbu, cgu, cgv, crv, &b0, &g0, &r0);
            packed_neon_four(vget_high_s16(yy), vget_high_s16(uu), vget_high_s16(vv),
                             cy, cbu, cgu, cgv, crv, &b1, &g1, &r1);
            uint8x8_t alpha = vdup_n_u8(255);
            if (j->alpha_x) {
                int16x8_t aa = vreinterpretq_s16_u16(
                    vsubl_u8(vld1_u8(y + j->alpha_x + x), vdup_n_u8(yoff)));
                int16x4_t a0 = vqmovn_s32(vrshrq_n_s32(vmulq_n_s32(vmovl_s16(vget_low_s16(aa)), cy), 16));
                int16x4_t a1 = vqmovn_s32(vrshrq_n_s32(vmulq_n_s32(vmovl_s16(vget_high_s16(aa)), cy), 16));
                alpha = vqmovun_s16(vcombine_s16(a0, a1));
            }
            uint8x8x4_t bgra = {{ vqmovun_s16(vcombine_s16(b0, b1)),
                                  vqmovun_s16(vcombine_s16(g0, g1)),
                                  vqmovun_s16(vcombine_s16(r0, r1)),
                                  alpha }};
            if (j->alpha_x && j->alpha != 255) {
                uint16x8_t a = vmull_u8(bgra.val[3], vdup_n_u8(j->alpha));
                a = vaddq_u16(a, vdupq_n_u16(128));
                bgra.val[3] = vshrn_n_u16(vaddq_u16(a, vshrq_n_u16(a, 8)), 8);
            }
            vst4_u8(dst + 4 * x, bgra);
        }
        for (; x < j->dw; x++) {
            int Y = (y[x] - yoff) * cy;
            int U = uv[(x >> 1) * 2] - 128, V = uv[(x >> 1) * 2 + 1] - 128;
            dst[4 * x] = clamp8((Y + cbu * U + 32768) >> 16);
            dst[4 * x + 1] = clamp8((Y + cgu * U + cgv * V + 32768) >> 16);
            dst[4 * x + 2] = clamp8((Y + crv * V + 32768) >> 16);
            if (j->alpha_x) {
                int a = clamp8(((y[j->alpha_x + x] - yoff) * cy + 32768) >> 16);
                dst[4 * x + 3] = (uint8_t)((a * j->alpha + 127) / 255);
            } else dst[4 * x + 3] = 255;
        }
    }
}
#endif

static void cvt_rows(const cvt_job_t *j) {
    cvt_job_t staged;
    if (j->stage_y) {
        staged = *j;
        const yuv_img_t *src = &j->img;
        if (src->y != j->stage_y) {
            for (int row = j->row0; row < j->row1; row++)
                memcpy(j->stage_y + (long)row * src->ys, src->y + (long)row * src->ys, src->w);
            for (int row = j->row0 / 2; row < j->row1 / 2; row++)
                memcpy(j->stage_uv + (long)row * src->us, src->u + (long)row * src->us, j->dw);
        }
        staged.img.y = j->stage_y;
        staged.img.u = j->stage_uv;
        staged.img.v = j->stage_uv + 1;
        j = &staged;
    }
    const yuv_img_t *s = &j->img;
#if defined(__aarch64__) && defined(__ARM_NEON)
    if ((j->alpha_x || j->stage_y) && s->cstep == 2 && s->v == s->u + 1 && s->us == s->vs &&
        j->dw == (j->alpha_x ? j->alpha_x : s->w) && j->dh == s->h) {
        cvt_packed_nv12_neon(j);
        return;
    }
#endif
    /* fixed point 16.16; limited-range BT.709 unless full_range           */
    const int cy  = s->full_range ? 65536 : 76309;   /* 255/219             */
    const int crv = s->full_range ? 103206 : 117489; /* 1.5748 (*255/224)   */
    const int cgu = s->full_range ? -12276 : -13975; /* -0.1873             */
    const int cgv = s->full_range ? -30679 : -34925; /* -0.4681             */
    const int cbu = s->full_range ? 121609 : 138438; /* 1.8556              */
    const int yoff = s->full_range ? 0 : 16;
    const int cs = s->cstep;                          /* 1 = I420, 2 = NV12 */
    for (int dy = j->row0; dy < j->row1; dy++) {
        int sy = dy;
        const uint8_t *yr = s->y + (long)sy * s->ys;
        const uint8_t *ur = s->u + (long)(sy >> 1) * s->us;
        const uint8_t *vr = s->v + (long)(sy >> 1) * s->vs;
        uint8_t *o = j->dst + (long)dy * j->ds;
        for (int x = 0; x < j->dw; x++) {
            int sx = x;
            int Y = (yr[sx] - yoff) * cy;
            int c = (sx >> 1) * cs;
            int U = ur[c] - 128, V = vr[c] - 128;
            o[0] = clamp8((Y + cbu * U + 32768) >> 16);
            o[1] = clamp8((Y + cgu * U + cgv * V + 32768) >> 16);
            o[2] = clamp8((Y + crv * V + 32768) >> 16);
            if (j->alpha_x) {
                int a = clamp8(((yr[j->alpha_x + sx] - yoff) * cy + 32768) >> 16);
                o[3] = (uint8_t)((a * j->alpha + 127) / 255);
            } else o[3] = 255;
            o += 4;
        }
    }
}

#define CVT_MAX_THREADS 4
static struct {
    pthread_mutex_t call_mu;
    pthread_mutex_t mu;
    pthread_cond_t  go, done;
    cvt_job_t jobs[CVT_MAX_THREADS];
    int gen, pending;
    int workers;
} pool = { .call_mu = PTHREAD_MUTEX_INITIALIZER,
           .mu = PTHREAD_MUTEX_INITIALIZER, .go = PTHREAD_COND_INITIALIZER,
           .done = PTHREAD_COND_INITIALIZER };
static pthread_once_t pool_once = PTHREAD_ONCE_INIT;

static void *cvt_worker(void *arg) {
    int idx = (int)(long)arg, seen = 0;
    for (;;) {
        pthread_mutex_lock(&pool.mu);
        while (pool.gen == seen) pthread_cond_wait(&pool.go, &pool.mu);
        seen = pool.gen;
        cvt_job_t job = pool.jobs[idx];
        pthread_mutex_unlock(&pool.mu);
        cvt_rows(&job);
        pthread_mutex_lock(&pool.mu);
        if (--pool.pending == 0) pthread_cond_signal(&pool.done);
        pthread_mutex_unlock(&pool.mu);
    }
    return NULL;
}

static void cvt_init(void) {
    for (int i = 1; i < CVT_MAX_THREADS; i++) {
        pthread_t t;
        int r = pthread_create(&t, NULL, cvt_worker, (void *)(long)i);
        if (r != 0) {
            vlog("yuv2bgra: worker %d creation failed: %d", i, r);
            break;
        }
        pool.workers++;
        r = pthread_detach(t);
        if (r != 0) vlog("yuv2bgra: worker %d detach failed: %d", i, r);
    }
}

static void cvt_run(const cvt_job_t *base, int nthreads) {
    pthread_once(&pool_once, cvt_init);
    if (nthreads > pool.workers + 1) nthreads = pool.workers + 1;
    if (nthreads < 1) nthreads = 1;
    pthread_mutex_lock(&pool.call_mu);
    /* Staged 4:2:0 jobs must not share a chroma row between workers. */
    int rows = base->dh, row_unit = base->stage_y ? 2 : 1;
    pthread_mutex_lock(&pool.mu);
    for (int i = 0; i < CVT_MAX_THREADS; i++) {
        pool.jobs[i] = *base;
        pool.jobs[i].row0 = i < nthreads ? (int)((int64_t)(rows / row_unit) * i / nthreads) * row_unit : rows;
        pool.jobs[i].row1 = i < nthreads ? (int)((int64_t)(rows / row_unit) * (i + 1) / nthreads) * row_unit : rows;
    }
    pool.pending = pool.workers;
    pool.gen++;
    pthread_cond_broadcast(&pool.go);
    pthread_mutex_unlock(&pool.mu);
    cvt_rows(&pool.jobs[0]);
    pthread_mutex_lock(&pool.mu);
    while (pool.pending) pthread_cond_wait(&pool.done, &pool.mu);
    pthread_mutex_unlock(&pool.mu);
    pthread_mutex_unlock(&pool.call_mu);
}

static int packed_job(const yuv_img_t *img, uint8_t *dst, int dst_stride, cvt_job_t *job) {
    if (!img || !dst || !img->y || !img->u || !img->v ||
        img->w <= 0 || img->w % 4 || img->h <= 0 || img->h % 2 ||
        (img->cstep != 1 && img->cstep != 2) ||
        img->ys < img->w || img->us < img->w / 2 * img->cstep ||
        img->vs < img->w / 2 * img->cstep ||
        dst_stride < 0 || (int64_t)dst_stride < (int64_t)img->w * 2)
        return -1;
    *job = (cvt_job_t){
        .img = *img, .dst = dst, .ds = dst_stride,
        .dw = img->w / 2, .dh = img->h, .alpha_x = img->w / 2,
        .alpha = 255,
    };
    return 0;
}

int yuv_packed_alpha_to_bgra(const yuv_img_t *img, uint8_t *dst, int dst_stride, int threads) {
    cvt_job_t job;
    if (packed_job(img, dst, dst_stride, &job) != 0) return -1;
    cvt_run(&job, threads);
    return 0;
}

static int staged_convert(const yuv_img_t *img, cvt_job_t *job, uint8_t *staging,
                          size_t staging_size, int threads) {
    if (!staging ||
        img->cstep != 2 || img->v != img->u + 1 || img->us != img->vs)
        return -1;
    if ((size_t)img->ys > SIZE_MAX / (size_t)img->h ||
        (size_t)img->us > SIZE_MAX / (size_t)(img->h / 2))
        return -1;
    size_t ysize = (size_t)img->ys * img->h, uvsize = (size_t)img->us * (img->h / 2);
    if (ysize > staging_size || uvsize > staging_size - ysize) return -1;
    job->stage_y = staging;
    job->stage_uv = staging + ysize;
    cvt_run(job, threads);
    return 0;
}

int yuv_packed_alpha_to_bgra_staged(const yuv_img_t *img, uint8_t *staging, size_t staging_size,
                                    uint8_t *dst, int dst_stride, int threads, uint8_t alpha) {
    cvt_job_t job;
    if (packed_job(img, dst, dst_stride, &job) != 0) return -1;
    job.alpha = alpha;
    return staged_convert(img, &job, staging, staging_size, threads);
}

int yuv_opaque_to_rgbx_staged(const yuv_img_t *img, uint8_t *staging, size_t staging_size,
                              uint8_t *dst, int dst_stride, int threads) {
    if (!img || !dst || !img->y || !img->u || !img->v ||
        img->w <= 0 || img->w % 2 || img->h <= 0 || img->h % 2 ||
        img->ys < img->w || img->us < img->w || img->vs < img->w ||
        dst_stride < 0 || (int64_t)dst_stride < (int64_t)img->w * 4)
        return -1;
    cvt_job_t job = { .img = *img, .dst = dst, .ds = dst_stride, .dw = img->w, .dh = img->h, .alpha = 255 };
    return staged_convert(img, &job, staging, staging_size, threads);
}
