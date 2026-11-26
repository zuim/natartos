// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../bridge.h"
#include <assert.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

void vlog(const char *fmt, ...) { (void)fmt; }

static void *concurrent_conversion(void *arg) {
    const yuv_img_t *img = arg;
    for (int i = 0; i < 500; i++) {
        uint8_t dst[40];
        assert(yuv_packed_alpha_to_bgra(img, dst, 20, 4) == 0);
        assert(dst[3] == 16 && dst[7] == 126 && dst[11] == 235);
    }
    return NULL;
}

static uint8_t reference_clamp(int value) {
    return value < 0 ? 0 : value > 255 ? 255 : (uint8_t)value;
}

static void check_random_frames(void) {
    srand(12345);
    for (int width = 2; width <= 70; width += 2) {
        for (int full_range = 0; full_range <= 1; full_range++) {
            for (int mode = 0; mode < 3; mode++) {
                int cstep = mode == 0 ? 1 : 2;
                int height = width % 6 == 0 ? 2 : 6, ys = 2 * width + 3;
                int cs = width * cstep + 5, vs = cs + (mode == 2), ds = width * 4 + 7;
                size_t ysize = (size_t)ys * height, csize = (size_t)cs * (height / 2);
                size_t vsize = (size_t)vs * (height / 2);
                size_t dsize = (size_t)ds * height;
                uint8_t *y = malloc(ysize + 1), *u = malloc(csize + 2);
                uint8_t *v = mode == 1 ? u + 1 : malloc(vsize + 1);
                uint8_t *actual = malloc(dsize + 1), *expected = malloc(dsize + 1);
                assert(y && u && v && actual && expected);
                for (size_t i = 0; i < ysize + 1; i++) y[i] = (uint8_t)rand();
                for (size_t i = 0; i < csize + 2; i++) u[i] = (uint8_t)rand();
                if (mode != 1) for (size_t i = 0; i < vsize + 1; i++) v[i] = (uint8_t)rand();
                memset(actual, 0x5a, dsize + 1);
                memset(expected, 0x5a, dsize + 1);
                yuv_img_t img = {
                    .y = y + 1, .u = u, .v = v, .ys = ys, .us = cs, .vs = vs,
                    .cstep = cstep, .w = 2 * width, .h = height, .full_range = full_range,
                };
                int cy = full_range ? 65536 : 76309, yoff = full_range ? 0 : 16;
                int cbu = full_range ? 121609 : 138438, crv = full_range ? 103206 : 117489;
                int cgu = full_range ? -12276 : -13975, cgv = full_range ? -30679 : -34925;
                for (int row = 0; row < height; row++) {
                    for (int x = 0; x < width; x++) {
                        int Y = (img.y[row * ys + x] - yoff) * cy;
                        int U = img.u[(row / 2) * cs + (x / 2) * cstep] - 128;
                        int V = img.v[(row / 2) * vs + (x / 2) * cstep] - 128;
                        uint8_t *p = expected + 1 + row * ds + x * 4;
                        p[0] = reference_clamp((Y + cbu * U + 32768) >> 16);
                        p[1] = reference_clamp((Y + cgu * U + cgv * V + 32768) >> 16);
                        p[2] = reference_clamp((Y + crv * V + 32768) >> 16);
                        p[3] = reference_clamp(((img.y[row * ys + width + x] - yoff) * cy + 32768) >> 16);
                    }
                }
                assert(yuv_packed_alpha_to_bgra(&img, actual + 1, ds, 4) == 0);
                assert(memcmp(actual, expected, dsize + 1) == 0);
                if (mode == 1) {
                    size_t staging_size = ysize + csize;
                    uint8_t *staging = malloc(staging_size);
                    assert(staging);
                    for (int threads = 1; threads <= 4; threads *= 2) {
                        memset(staging, 0x5a, staging_size);
                        memset(actual, 0x5a, dsize + 1);
                        assert(yuv_packed_alpha_to_bgra_staged(&img, staging, staging_size,
                                                               actual + 1, ds, threads, 255) == 0);
                        assert(memcmp(actual, expected, dsize + 1) == 0);
                        for (int row = 0; row < height; row++)
                            for (int x = img.w; x < ys; x++)
                                assert(staging[(size_t)row * ys + x] == 0x5a);
                        for (int row = 0; row < height / 2; row++)
                            for (int x = width; x < cs; x++)
                                assert(staging[ysize + (size_t)row * cs + x] == 0x5a);
                    }
                    assert(yuv_packed_alpha_to_bgra_staged(&img, staging, staging_size - 1,
                                                           actual + 1, ds, 4, 255) == -1);
                    img.w = width;
                    for (int row = 0; row < height; row++)
                        for (int x = 0; x < width; x++)
                            expected[1 + row * ds + x * 4 + 3] = 255;
                    for (int threads = 1; threads <= 4; threads *= 2) {
                        memset(actual, 0x5a, dsize + 1);
                        assert(yuv_opaque_to_rgbx_staged(&img, staging, staging_size,
                                                         actual + 1, ds, threads) == 0);
                        assert(memcmp(actual, expected, dsize + 1) == 0);
                    }
                    free(staging);
                } else {
                    uint8_t staging[1];
                    assert(yuv_packed_alpha_to_bgra_staged(&img, staging, sizeof(staging),
                                                           actual + 1, ds, 4, 255) == -1);
                }
                free(y); free(u);
                if (mode != 1) free(v);
                free(actual); free(expected);
            }
        }
    }
}

int main(void) {
    uint8_t y[] = {16, 235, 81, 145, 16, 126, 235, 0,
                   16, 235, 81, 145, 16, 126, 235, 0};
    uint8_t uv[] = {128, 128, 128, 128, 128, 128, 128, 128};
    uint8_t dst[40];
    yuv_img_t img = {
        .y = y, .u = uv, .v = uv + 1, .ys = 8, .us = 8, .vs = 8,
        .cstep = 2, .w = 8, .h = 2, .full_range = 0,
    };
    uint8_t opaque_dst[72], staging[24];
    memset(opaque_dst, 0x5a, sizeof(opaque_dst));
    assert(yuv_opaque_to_rgbx_staged(&img, staging, sizeof(staging), opaque_dst, 36, 2) == 0);
    for (int row = 0; row < 2; row++) {
        for (int x = 0; x < 8; x++) assert(opaque_dst[row * 36 + x * 4 + 3] == 255);
        assert(opaque_dst[row * 36 + 32] == 0x5a);
    }
    assert(opaque_dst[0] == 0 && opaque_dst[4] == 255);
    assert(yuv_opaque_to_rgbx_staged(&img, staging, sizeof(staging) - 1, opaque_dst, 36, 2) == -1);
    memset(dst, 0x5a, sizeof(dst));
    assert(yuv_packed_alpha_to_bgra(&img, dst, 20, 2) == 0);
    for (int row = 0; row < 2; row++) {
        const uint8_t *p = dst + row * 20;
        assert(p[0] == 0 && p[1] == 0 && p[2] == 0 && p[3] == 0);
        assert(p[4] == 255 && p[5] == 255 && p[6] == 255 && p[7] == 128);
        assert(p[11] == 255 && p[15] == 0);
        assert(p[16] == 0x5a && p[19] == 0x5a);
    }
    uv[0] = 90; uv[1] = 240;
    assert(yuv_packed_alpha_to_bgra(&img, dst, 20, 2) == 0);
    assert(dst[2] > dst[0]);
    uv[0] = 128; uv[1] = 128;
    img.full_range = 1;
    assert(yuv_packed_alpha_to_bgra(&img, dst, 20, 1) == 0);
    assert(dst[3] == 16 && dst[7] == 126 && dst[11] == 235);
    uint8_t u[] = {128, 128, 128, 128}, v[] = {128, 128, 128, 128};
    img.u = u; img.v = v; img.us = 4; img.vs = 4; img.cstep = 1;
    assert(yuv_packed_alpha_to_bgra(&img, dst, 20, 2) == 0);
    assert(dst[0] == 16 && dst[3] == 16 && dst[4] == 235 && dst[7] == 126);
    pthread_t workers[4];
    for (int i = 0; i < 4; i++) assert(pthread_create(&workers[i], NULL, concurrent_conversion, &img) == 0);
    for (int i = 0; i < 4; i++) assert(pthread_join(workers[i], NULL) == 0);
    img.us = 3;
    assert(yuv_packed_alpha_to_bgra(&img, dst, 20, 1) == -1);
    img.us = 4;
    img.w = 6;
    assert(yuv_packed_alpha_to_bgra(&img, dst, 20, 1) == -1);
    img.w = 8;
    assert(yuv_packed_alpha_to_bgra(&img, dst, 15, 1) == -1);
    img.h = 1;
    assert(yuv_packed_alpha_to_bgra(&img, dst, 20, 1) == -1);
    assert(yuv_packed_alpha_to_bgra(NULL, dst, 20, 1) == -1);
    check_random_frames();
    uint8_t fade_y[80], fade_uv[40], fade_stage[120], fade_dst[160], baseline[160];
    for (int i = 0; i < 80; i++) fade_y[i] = (uint8_t)(i * 43);
    memset(fade_uv, 128, sizeof(fade_uv));
    img = (yuv_img_t){ .y = fade_y, .u = fade_uv, .v = fade_uv + 1,
                      .ys = 40, .us = 40, .vs = 40, .cstep = 2, .w = 40, .h = 2 };
    assert(yuv_packed_alpha_to_bgra(&img, baseline, 80, 2) == 0);
    for (int pass = 0; pass < 2; pass++) {
        for (int i = 0; i < 4; i++) {
            uint8_t alpha = (uint8_t[]){0, 128, 255, 64}[i];
            assert(yuv_packed_alpha_to_bgra_staged(&img, fade_stage, sizeof(fade_stage),
                                                   fade_dst, 80, 2, alpha) == 0);
            for (int pixel = 0; pixel < 40; pixel++) {
                assert(!memcmp(fade_dst + pixel * 4, baseline + pixel * 4, 3));
                assert(fade_dst[pixel * 4 + 3] == (baseline[pixel * 4 + 3] * alpha + 127) / 255);
            }
        }
        img.y = fade_stage;
        img.u = fade_stage + 80;
        img.v = img.u + 1;
    }
    img.w = 20;
    assert(yuv_opaque_to_rgbx_staged(&img, fade_stage, sizeof(fade_stage),
                                    fade_dst, 80, 2) == 0);
    for (int pixel = 0; pixel < 40; pixel++) assert(fade_dst[pixel * 4 + 3] == 255);
    return 0;
}
