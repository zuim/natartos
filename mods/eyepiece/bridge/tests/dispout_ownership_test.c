// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../dispout.c"
#include <assert.h>
#include <stdarg.h>

static void *released[8];
static int released_count, push_result, defer_pop;
static dispout_t *pop_during_push;
void vlog(const char *fmt, ...) { (void)fmt; }
uint64_t now_us(void) { static uint64_t t; return ++t; }
int hwdec_hal_init(void) { return 0; }
static int push(void *disp, int8_t plane, void *fb) {
    if (pop_during_push) {
        if (disp == (void *)1) {
            if (!defer_pop) on_pop(NULL, plane, fb, &pop_during_push->vctx[0]);
            return 0;
        }
        return -1001;
    }
    return push_result;
}
static void release(void *user, void *fb) {
    (void)user;
    released[released_count++] = fb;
}

int main(void) {
    dispout_t o = {0};
    uint64_t frames[4][0x330 / sizeof(uint64_t)] = {{0}};
    pthread_mutex_init(&o.mu, NULL);
    pthread_mutex_init(&o.push_mu, NULL);
    o.nvop = 1;
    o.vctx[0].o = &o;
    o.release = release;
    D.push_frame = push;
    for (int i = 0; i < 3; i++) assert(dispout_push(&o, frames[i]) == 0);
    assert(released_count == 0);
    on_pop(NULL, 6, frames[1], &o.vctx[0]);
    assert(released_count == 2);
    assert(released[0] == frames[0] && released[1] == frames[1]);
    on_pop(NULL, 6, frames[0], &o.vctx[0]);
    assert(released_count == 2);
    on_pop(NULL, 6, frames[2], &o.vctx[0]);
    assert(released_count == 3 && released[2] == frames[2]);
    push_result = -1;
    assert(dispout_push(&o, frames[3]) == DISPOUT_PUSH_ERROR);
    push_result = -1001;
    assert(dispout_push(&o, frames[3]) == DISPOUT_PUSH_REJECTED);
    push_result = 0;
    assert(dispout_push(&o, frames[3]) == DISPOUT_PUSH_OK);
    on_pop(NULL, 6, frames[3], &o.vctx[0]);
    assert(released_count == 4 && released[3] == frames[3]);
    o.nvop = 2;
    o.vctx[1].o = &o;
    o.vctx[1].v = 1;
    assert(dispout_push(&o, frames[0]) == DISPOUT_PUSH_OK);
    on_pop(NULL, 4, frames[0], &o.vctx[1]);
    assert(released_count == 4);
    on_pop(NULL, 4, frames[0], &o.vctx[0]);
    assert(released_count == 5 && released[4] == frames[0]);
    o.disp[0] = (void *)1;
    o.disp[1] = (void *)2;
    pop_during_push = &o;
    assert(dispout_push(&o, frames[1]) == DISPOUT_PUSH_OK);
    assert(released_count == 6 && released[5] == frames[1]);
    defer_pop = 1;
    assert(dispout_push(&o, frames[2]) == DISPOUT_PUSH_OK);
    assert(released_count == 6);
    on_pop(NULL, 4, frames[2], &o.vctx[0]);
    assert(released_count == 7 && released[6] == frames[2]);
    pop_during_push = NULL;
    push_result = -1001;
    assert(dispout_push(&o, frames[3]) == DISPOUT_PUSH_REJECTED);
    assert(released_count == 7);
    pthread_mutex_destroy(&o.push_mu);
    pthread_mutex_destroy(&o.mu);
    return 0;
}
