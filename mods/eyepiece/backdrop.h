// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef EYEPIECE_BACKDROP_H
#define EYEPIECE_BACKDROP_H

#include <stdint.h>

#ifndef EYEPIECE_BACKDROP_PATH
#define EYEPIECE_BACKDROP_PATH "/data/local/tmp/eyepiece.backdrop"
#endif

typedef struct {
    uint64_t request;
    uint64_t acknowledged;
} eyepiece_backdrop_request;

typedef struct {
    void *layer;
    int requested;
    int suppressed;
} eyepiece_backdrop_state;

static inline void eyepiece_backdrop_init_layer(eyepiece_backdrop_state *state,
                                                void *layer, int visible) {
    state->layer = layer;
    state->requested = !!visible;
}

static inline int eyepiece_backdrop_stock_visibility(eyepiece_backdrop_state *state,
                                                     void *layer, int visible) {
    if (state->layer != layer) return visible;
    state->requested = !!visible;
    return state->suppressed ? 0 : visible;
}

static inline int eyepiece_backdrop_visibility(const eyepiece_backdrop_state *state) {
    return state->suppressed ? 0 : state->requested;
}

#endif
