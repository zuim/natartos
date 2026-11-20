// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * yuv2fb.c — YUV 4:2:0 (I420 or NV12) → BGRA into the fb_shared back slot,
 * aspect-preserving fit + nearest-neighbour scale, row-parallel on a small
 * persistent thread pool.  BT.709, limited or full range.
 */
#include <pthread.h>
#include <stdint.h>
#include "bridge.h"

typedef struct {
    yuv_img_t img;
    uint8_t *dst; int ds;        /* destination (top-left of target rect)     */
    int dw, dh;                  /* target rect                               */
    int row0, row1;
} cvt_job_t;

static inline uint8_t clamp8(int v) { return v < 0 ? 0 : v > 255 ? 255 : (uint8_t)v; }

static void cvt_rows(const cvt_job_t *j) {
    const yuv_img_t *s = &j->img;
    /* fixed point 16.16; limited-range BT.709 unless full_range           */
    const int cy  = s->full_range ? 65536 : 76309;   /* 255/219             */
    const int crv = s->full_range ? 103206 : 117489; /* 1.5748 (*255/224)   */
    const int cgu = s->full_range ? -12276 : -13975; /* -0.1873             */
    const int cgv = s->full_range ? -30679 : -34925; /* -0.4681             */
    const int cbu = s->full_range ? 121609 : 138438; /* 1.8556              */
    const int yoff = s->full_range ? 0 : 16;
    const int cs = s->cstep;                          /* 1 = I420, 2 = NV12 */
    const int same = (s->w == j->dw && s->h == j->dh);
    for (int dy = j->row0; dy < j->row1; dy++) {
        int sy = same ? dy : (int)((int64_t)dy * s->h / j->dh);
        const uint8_t *yr = s->y + (long)sy * s->ys;
        const uint8_t *ur = s->u + (long)(sy >> 1) * s->us;
        const uint8_t *vr = s->v + (long)(sy >> 1) * s->vs;
        uint8_t *o = j->dst + (long)dy * j->ds;
        for (int x = 0; x < j->dw; x++) {
            int sx = same ? x : (int)((int64_t)x * s->w / j->dw);
            int Y = (yr[sx] - yoff) * cy;
            int c = (sx >> 1) * cs;
            int U = ur[c] - 128, V = vr[c] - 128;
            o[0] = clamp8((Y + cbu * U + 32768) >> 16);
            o[1] = clamp8((Y + cgu * U + cgv * V + 32768) >> 16);
            o[2] = clamp8((Y + crv * V + 32768) >> 16);
            o[3] = 255;
            o += 4;
        }
    }
}

#define CVT_MAX_THREADS 4
static struct {
    pthread_mutex_t mu;
    pthread_cond_t  go, done;
    cvt_job_t jobs[CVT_MAX_THREADS];
    int gen, pending;
    int started;
} pool = { .mu = PTHREAD_MUTEX_INITIALIZER, .go = PTHREAD_COND_INITIALIZER,
           .done = PTHREAD_COND_INITIALIZER };

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

static void cvt_run(const cvt_job_t *base, int nthreads) {
    if (nthreads > CVT_MAX_THREADS) nthreads = CVT_MAX_THREADS;
    if (nthreads < 1) nthreads = 1;
    if (!pool.started) {
        for (int i = 1; i < CVT_MAX_THREADS; i++) {
            pthread_t t;
            pthread_create(&t, NULL, cvt_worker, (void *)(long)i);
            pthread_detach(t);
        }
        pool.started = 1;
    }
    int rows = base->dh;
    pthread_mutex_lock(&pool.mu);
    for (int i = 0; i < CVT_MAX_THREADS; i++) {
        pool.jobs[i] = *base;
        pool.jobs[i].row0 = i < nthreads ? rows * i / nthreads : rows;
        pool.jobs[i].row1 = i < nthreads ? rows * (i + 1) / nthreads : rows;
    }
    pool.pending = CVT_MAX_THREADS - 1;   /* workers 1..3; caller runs slot 0 */
    pool.gen++;
    pthread_cond_broadcast(&pool.go);
    pthread_mutex_unlock(&pool.mu);
    cvt_rows(&pool.jobs[0]);
    pthread_mutex_lock(&pool.mu);
    while (pool.pending) pthread_cond_wait(&pool.done, &pool.mu);
    pthread_mutex_unlock(&pool.mu);
}

void yuv_to_fb(fbout_t *fb, const yuv_img_t *img, int threads) {
    static int last_w, last_h;
    if (img->w != last_w || img->h != last_h) {
        vlog("video %dx%d (%s) → fb %dx%d", img->w, img->h, img->cstep == 2 ? "NV12" : "I420", fb->w, fb->h);
        last_w = img->w; last_h = img->h;
        fbout_clear(fb);                 /* letterbox bars */
    }
    int dw = fb->w, dh = (int)((int64_t)fb->w * img->h / img->w);
    if (dh > fb->h) { dh = fb->h; dw = (int)((int64_t)fb->h * img->w / img->h); }
    int ox = (fb->w - dw) / 2, oy = (fb->h - dh) / 2;
    cvt_job_t job = {
        .img = *img,
        .dst = fbout_back(fb) + ((long)oy * fb->w + ox) * 4, .ds = fb->w * 4,
        .dw = dw, .dh = dh,
    };
    cvt_run(&job, threads);
}
