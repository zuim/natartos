// SPDX-License-Identifier: AGPL-3.0-or-later
#include <dlfcn.h>
static void *fake_dlopen(const char *name, int flags);
static void *fake_dlsym(void *handle, const char *name);
#define dlopen fake_dlopen
#define dlsym fake_dlsym
#include "../hwdec.c"
#undef dlopen
#undef dlsym
#include <assert.h>
#include <stdarg.h>

static int push_error, sync_error, convert_error, failures, pushes, shown;
static int device_opens, display_opens;
static int stop_failure, display_failure, frees, closes, stops;
static int prepare_failure, prepare_calls, skip_output;
static int opaque_conversions;
static int global_alpha_failure, global_alpha_calls;
static uint8_t global_alpha;
static void *last_frame;
static const uint8_t *expected_staging;

static int fake_initialize(hal_dev_t *devices) { (void)devices; return 0; }
static int fake_device_open(const char *name, void *config, void **handle) {
    (void)name; (void)config; (void)handle;
    device_opens++;
    return -1;
}
static void fake_symbol(void) {}
static void *fake_dlopen(const char *name, int flags) {
    (void)name; (void)flags;
    return (void *)1;
}
static void *fake_dlsym(void *handle, const char *name) {
    (void)handle;
    if (!strcmp(name, "duss_hal_initialize")) return (void *)fake_initialize;
    if (!strcmp(name, "duss_hal_device_open")) return (void *)fake_device_open;
    return (void *)fake_symbol;
}

void vlog(const char *fmt, ...) { (void)fmt; }
uint64_t now_us(void) { return 1; }
int yuv_packed_alpha_to_bgra_staged(const yuv_img_t *img, uint8_t *staging, size_t staging_size,
                                    uint8_t *dst, int stride, int threads, uint8_t alpha) {
    (void)img; (void)staging; (void)staging_size; (void)stride; (void)threads;
    if (convert_error) return -1;
    if (expected_staging) {
        assert(staging == expected_staging && staging_size == 24);
        assert(staging != img->y);
        staging[0] = img->y[0];
        staging[16] = img->u[0];
    }
    dst[0]++;
    dst[3] = alpha;
    return 0;
}
int yuv_opaque_to_rgbx_staged(const yuv_img_t *img, uint8_t *staging, size_t staging_size,
                              uint8_t *dst, int stride, int threads) {
    opaque_conversions++;
    return yuv_packed_alpha_to_bgra_staged(img, staging, staging_size, dst, stride, threads, 255);
}
int dispout_push(dispout_t *disp, void *fb) {
    (void)disp;
    last_frame = fb;
    pushes++;
    return push_error;
}
int dispout_move_pane(dispout_t *disp, int x, int y) {
    (void)disp; (void)x; (void)y; return 0;
}
int dispout_set_pane_alpha(dispout_t *disp, uint8_t alpha) {
    (void)disp;
    global_alpha_calls++;
    if (global_alpha_failure) return -1;
    global_alpha = alpha;
    return 0;
}
dispout_t *dispout_open(int mask, int w, int h, int stereo,
                        dispout_release_cb release, void *user) {
    (void)mask; (void)w; (void)h; (void)stereo; (void)release; (void)user;
    display_opens++;
    return NULL;
}
dispout_t *dispout_open_pane(int w, int h, int x, int y, int opaque, uint8_t alpha,
                            dispout_release_cb release, void *user) {
    (void)w; (void)h; (void)x; (void)y; (void)opaque; (void)alpha; (void)release; (void)user;
    return NULL;
}
int dispout_close_checked(dispout_t *disp) { (void)disp; return display_failure ? -1 : 0; }
static int fake_stop(void *device, void *config) {
    (void)device;
    assert(config && *(uint32_t *)config == 0);
    stops++;
    return stop_failure ? -1 : 0;
}
static int fake_close(void *device) { (void)device; closes++; return 0; }
static int fake_free(void *memory) { (void)memory; frees++; return 0; }

static int fake_sync(void *mem, uint32_t direction) {
    (void)mem;
    assert(direction == 1);
    return sync_error;
}
static void failed(void *user) { (void)user; failures++; }
static int prepare(void *user) {
    (void)user;
    prepare_calls++;
    return prepare_failure ? -1 : 0;
}
static int should_skip(void *user) { (void)user; return skip_output; }
static void frame_shown(void *user, const yuv_img_t *img, uint64_t pts, uint32_t us) {
    (void)user; (void)img; (void)pts; (void)us;
    shown++;
}

typedef struct { hwdec_t *decoder; atomic_int done; int result; } drain_test_t;
static void *run_drain(void *arg) {
    drain_test_t *test = arg;
    test->result = hwdec_drain_overlay(test->decoder);
    atomic_store(&test->done, 1);
    return NULL;
}

int main(void) {
    hwdec_tune_t tune = { .direct_vops = 1 };
    assert(hwdec_open(eyepiece_rpc_v1_Codec_CODEC_HEVC, 1920, 1080, 4, -1, &tune, NULL, NULL) == NULL);
    assert(display_opens == 1 && device_opens == 0);

    hwdec_t reset = {0};
    uint64_t stream_desc[STRM_DESC_SZ / sizeof(uint64_t)] = {0};
    uint64_t frame_desc[FRM_DESC_SZ / sizeof(uint64_t)] = {0};
    uint64_t overlay_desc[OVERLAY_BUFS][FRM_DESC_SZ / sizeof(uint64_t)] = {{0}};
    pthread_mutex_init(&reset.mu, NULL);
    pthread_cond_init(&reset.ret_cv, NULL);
    pthread_cond_init(&reset.strm_cv, NULL);
    pthread_cond_init(&reset.callbacks_cv, NULL);
    reset.pane_output = 1;
    reset.disp = (dispout_t *)1;
    reset.nstrm = reset.nframe = 1;
    reset.strm[0] = stream_desc;
    reset.frame[0] = frame_desc;
    for (int i = 0; i < OVERLAY_BUFS; i++) reset.overlay[i].desc = overlay_desc[i];
    assert(overlay_drained(&reset));
    reset.strm_busy[0] = 1;
    assert(!overlay_drained(&reset));
    reset.strm_busy[0] = 0;
    reset.last_frame_pending = 1;
    assert(!overlay_drained(&reset));
    reset.last_frame_pending = 0;
    reset.callbacks_active = 1;
    assert(!overlay_drained(&reset));
    reset.callbacks_active = 0;
    reset.ret_n = 1;
    assert(!overlay_drained(&reset));
    reset.ret_n = 0;
    reset.returning = 1;
    assert(!overlay_drained(&reset));
    reset.returning = 0;
    reset.strm_busy[0] = 1;
    reset.last_frame_pending = 1;
    drain_test_t drain = { .decoder = &reset };
    pthread_t drain_thread;
    assert(pthread_create(&drain_thread, NULL, run_drain, &drain) == 0);
    usleep(20000);
    assert(!atomic_load(&drain.done));
    cb_strm_empty(&reset, stream_desc);
    usleep(20000);
    assert(!atomic_load(&drain.done));
    pthread_mutex_lock(&reset.mu);
    reset.last_frame_pending = 0;
    pthread_cond_broadcast(&reset.callbacks_cv);
    pthread_mutex_unlock(&reset.mu);
    assert(pthread_join(drain_thread, NULL) == 0);
    assert(drain.result == 0);
    hal.device_stop = fake_stop;
    hal.device_close = fake_close;
    hal.mem_free = fake_free;
    stop_failure = 1;
    assert(hwdec_reconfigure_close(&reset) == -1);
    assert(frees == 0 && closes == 0);
    stop_failure = 0;
    display_failure = 1;
    assert(hwdec_reconfigure_close(&reset) == -1);
    assert(frees == 0 && closes == 0);
    display_failure = 0;
    reset.staging = malloc(16);
    assert(hwdec_reconfigure_close(&reset) == 0);
    assert(frees == 2 + OVERLAY_BUFS && closes == 2 && stops == 4);
    assert(reset.disp == NULL && reset.staging == NULL);
    pthread_cond_destroy(&reset.callbacks_cv);
    pthread_cond_destroy(&reset.strm_cv);
    pthread_cond_destroy(&reset.ret_cv);
    pthread_mutex_destroy(&reset.mu);

    eyepiece_rpc_v1_Pane pane = { .x = 100, .y = 50, .width = 640, .height = 480, .format = eyepiece_rpc_v1_PaneFormat_PANE_BGRA };
    assert(ep_pane_bgra_valid(&pane, 1920, 1080));
    pane.width = 641;
    assert(!ep_pane_bgra_valid(&pane, 1920, 1080));
    pane.width = 640; pane.x = 1300;
    assert(!ep_pane_bgra_valid(&pane, 1920, 1080));
    pane.x = 100; pane.width = 640; pane.format = 255;
    assert(!ep_pane_bgra_valid(&pane, 1920, 1080));
    pane = (eyepiece_rpc_v1_Pane){ .width = 1920, .height = 1080, .format = eyepiece_rpc_v1_PaneFormat_PANE_BGRA };
    assert(ep_pane_bgra_valid(&pane, 1920, 1080));
    pane.format = eyepiece_rpc_v1_PaneFormat_PANE_OPAQUE_BGRA;
    assert(ep_pane_bgra_valid(&pane, 1920, 1080));
    pane.format = eyepiece_rpc_v1_PaneFormat_PANE_BGRA;
    pane.x = 1;
    assert(!ep_pane_bgra_valid(&pane, 1920, 1080));

    hwdec_t d = {0};
    uint64_t desc[OVERLAY_BUFS][FRM_DESC_SZ / sizeof(uint64_t)] = {{0}};
    uint8_t pixels[OVERLAY_BUFS][32] = {{0}};
    uint8_t initial_stage[24] = {0}, initial_retained[24] = {0};
    pthread_mutex_init(&d.mu, NULL);
    pthread_mutex_init(&d.staging_mu, NULL);
    d.staging = initial_stage;
    d.retained_staging = initial_retained;
    d.pane_output = 1;
    assert(hwdec_set_overlay_alpha(&d, 255) == 0);
    d.pane_w = 4; d.pane_h = 2; d.pane_stride = 16;
    d.output_error = failed;
    hal.mem_sync = fake_sync;
    for (int i = 0; i < OVERLAY_BUFS; i++) {
        d.overlay[i].desc = desc[i];
        d.overlay[i].va = pixels[i];
    }
    d.pane_opaque = 1;
    yuv_img_t opaque = { .w = 4, .h = 2 };
    assert(overlay_present(&d, &opaque, 99) == 1);
    assert(opaque_conversions == 1);
    assert(hwdec_set_overlay_alpha(&d, 128) == 0);
    assert(global_alpha == 128 && global_alpha_calls == 1 && d.pane_alpha == 128 && pushes == 1);
    global_alpha_failure = 1;
    assert(hwdec_set_overlay_alpha(&d, 64) == -1);
    assert(global_alpha == 128 && d.pane_alpha == 128 && pushes == 1);
    global_alpha_failure = 0;
    assert(hwdec_set_overlay_alpha(&d, 255) == 0);
    overlay_release(&d, desc[0]);
    d.pane_opaque = 0;
    pushes = 0;
    pixels[0][0] = 0;
    yuv_img_t img = { .w = 8, .h = 2 };
    for (int i = 0; i < OVERLAY_BUFS; i++) {
        assert(overlay_present(&d, &img, 100 + i) == 1);
        assert(last_frame == desc[i]);
        assert(U64(desc[i], FRM_PTS) == (uint64_t)(100 + i));
        assert(pixels[i][0] == 1);
    }
    assert(overlay_present(&d, &img, 200) == 0);
    assert(opaque_conversions == 1);
    assert(pushes == OVERLAY_BUFS);
    for (int i = 0; i < OVERLAY_BUFS; i++) assert(pixels[i][0] == 1);
    overlay_release(&d, desc[3]);
    assert(overlay_present(&d, &img, 201) == 1);
    assert(last_frame == desc[3] && pixels[3][0] == 2);
    overlay_release(&d, desc[3]);
    push_error = 1;
    assert(overlay_present(&d, &img, 202) == -1);
    assert(!d.overlay[3].busy && failures == 1);
    overlay_error(&d, "second failure");
    assert(failures == 1);
    atomic_store(&d.overlay_failed, 0);
    failures = 0;
    push_error = DISPOUT_PUSH_REJECTED;
    for (int i = 0; i < OVERLAY_REJECT_LIMIT - 1; i++)
        assert(overlay_present(&d, &img, 203 + i) == 0);
    assert(!d.overlay[3].busy && failures == 0);
    assert(overlay_present(&d, &img, 300) == -1);
    assert(!d.overlay[3].busy && failures == 1);

    uint64_t decoded[FRM_DESC_SZ / sizeof(uint64_t)] = {0};
    uint8_t decoded_pixels[24] = {42}, staged_pixels[24] = {0}, spare_pixels[24] = {0};
    decoded_pixels[16] = 99;
    d.pane_output = 1;
    d.nframe = 1;
    d.frame[0] = decoded;
    d.frame_va[0] = decoded_pixels;
    d.frame_size = sizeof(decoded_pixels);
    d.staging = staged_pixels;
    d.retained_staging = spare_pixels;
    d.cb = frame_shown;
    d.sync_dir = -1;
    pthread_cond_init(&d.ret_cv, NULL);
    U32(decoded, FRM_FMT) = PIXFMT_NV12;
    U32(decoded, FRM_W) = 8; U32(decoded, FRM_H) = 2;
    U32(decoded, FRM_STRIDE) = 8; U32(decoded, FRM_UV_OFF) = 16;
    U32(decoded, FRM_VALID_A) = 1; U32(decoded, FRM_VALID_B) = 1;
    d.prepare_output = prepare;
    d.skip_output = should_skip;
    int pushes_before = pushes;
    skip_output = 1;
    cb_frame_full(&d, decoded);
    assert(prepare_calls == 0 && pushes == pushes_before && d.ret_n == 1);
    skip_output = 0;
    prepare_failure = 1;
    cb_frame_full(&d, decoded);
    assert(prepare_calls == 1 && pushes == pushes_before && shown == 0 && d.ret_n == 2);
    prepare_failure = 0;
    for (int i = 0; i < OVERLAY_BUFS; i++) d.overlay[i].busy = 1;
    atomic_store(&d.overlay_failed, 0);
    failures = 0;
    cb_frame_full(&d, decoded);
    assert(failures == 0 && d.ret_n == 3);
    d.overlay[0].busy = 0;
    push_error = 0;
    expected_staging = staged_pixels;
    cb_frame_full(&d, decoded);
    assert(failures == 0 && shown == 1 && d.ret_n == 4);
    assert(d.retained_img.y == staged_pixels && d.retained_img.u == staged_pixels + 16);
    overlay_release(&d, desc[1]);
    decoded_pixels[0] = 7;
    expected_staging = spare_pixels;
    push_error = DISPOUT_PUSH_REJECTED;
    cb_frame_full(&d, decoded);
    assert(failures == 0 && shown == 1 && d.ret_n == 5);
    assert(d.retained_img.y[0] == 42 && !d.overlay[1].busy);
    push_error = 0;
    decoded_pixels[0] = 42;
    d.overlay[1].busy = 1;
    assert(hwdec_set_overlay_alpha(&d, 128) == -1);
    assert(d.pane_alpha == 255);
    overlay_release(&d, desc[1]);
    decoded_pixels[0] = 0;
    assert(hwdec_set_overlay_alpha(&d, 128) == 0);
    assert(last_frame == desc[1] && pixels[1][3] == 128 && pixels[0][3] == 255);
    assert(shown == 1 && d.ret_n == 5 && d.overlay_rejections == 0);
    overlay_release(&d, desc[1]);
    expected_staging = staged_pixels;
    assert(hwdec_set_overlay_alpha(&d, 255) == 0);
    assert(pixels[1][3] == 255);
    U32(decoded, FRM_FMT) = 0;
    cb_frame_full(&d, decoded);
    assert(failures == 1 && d.ret_n == 6);

    /* opaque pane, direct scan-out: the decoder frame itself is pushed, never converted */
    atomic_store(&d.overlay_failed, 0);
    failures = 0;
    U32(decoded, FRM_FMT) = PIXFMT_NV12;
    d.pane_opaque = 1; d.pane_direct = 1; d.pane_w = 8; d.pane_h = 2;
    d.disp = (dispout_t *)1;
    int conversions = opaque_conversions, returned = d.ret_n, shown_before = shown;
    pushes_before = pushes;
    cb_frame_full(&d, decoded);
    assert(pushes == pushes_before + 1 && last_frame == decoded && opaque_conversions == conversions);
    assert(shown == shown_before + 1 && d.ret_n == returned && failures == 0);
    push_error = DISPOUT_PUSH_REJECTED;
    cb_frame_full(&d, decoded);
    assert(shown == shown_before + 1 && d.ret_n == returned + 1 && failures == 0 && d.overlay_rejections == 1);
    d.overlay_rejections = OVERLAY_REJECT_LIMIT - 1;
    cb_frame_full(&d, decoded);
    assert(d.ret_n == returned + 2 && failures == 1);
    atomic_store(&d.overlay_failed, 0);
    failures = 0;
    push_error = 0;
    d.pane_w = 4;
    cb_frame_full(&d, decoded);
    assert(d.ret_n == returned + 3 && failures == 1 && shown == shown_before + 1);
    d.pane_opaque = 0; d.pane_direct = 0;
    d.disp = NULL;
    d.ret_n = 6;
    shown = 1;
    d.pane_output = 0;
    d.disp = (dispout_t *)1;
    prepare_failure = 1;
    pushes_before = pushes;
    cb_frame_full(&d, decoded);
    assert(pushes == pushes_before && shown == 1 && d.ret_n == 7);
    prepare_failure = 0;
    cb_frame_full(&d, decoded);
    assert(pushes == pushes_before + 1 && shown == 2);
    pthread_cond_destroy(&d.ret_cv);
    pthread_mutex_destroy(&d.staging_mu);
    pthread_mutex_destroy(&d.mu);
    return 0;
}
