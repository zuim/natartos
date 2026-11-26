// SPDX-License-Identifier: AGPL-3.0-or-later
// Microbenchmark: nanopb envelope decode with a hot receive buffer, no I/O.
#include "../bridge.h"
#include "eyepiece-rpc.pb.h"
#include <pb_encode.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct { const uint8_t *data; size_t length; } bytes_t;

static bool encode_bytes(pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
    const bytes_t *bytes = *arg;
    return pb_encode_tag_for_field(stream, field) &&
           pb_encode_string(stream, bytes->data, bytes->length);
}

static uint64_t ns(void) {
    struct timespec time;
    if (clock_gettime(CLOCK_MONOTONIC, &time)) { perror("clock_gettime"); exit(1); }
    return (uint64_t)time.tv_sec * 1000000000 + time.tv_nsec;
}

static int compare(const void *a, const void *b) {
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

int main(void) {
    const size_t sizes[] = { 8192, 25000, 262144, (4u << 20) - 64 };
    const size_t iterations[] = { 20000, 20000, 5000, 500 };
    const size_t batches = 100;
    puts("access_unit_bytes,iterations,encoded_bytes,median_ns,p95_ns");
    for (size_t s = 0; s < sizeof(sizes) / sizeof(sizes[0]); s++) {
        size_t n = sizes[s], count = iterations[s];
        uint8_t *au = malloc(n), *buffer = malloc(n + 128);
        uint64_t *samples = malloc(batches * sizeof(*samples));
        if (!au || !buffer || !samples) { fprintf(stderr, "allocation failed\n"); return 1; }
        memset(au, 0xa5, n);
        bytes_t bytes = { au, n };
        eyepiece_rpc_v1_Envelope envelope = eyepiece_rpc_v1_Envelope_init_zero;
        envelope.which_message = eyepiece_rpc_v1_Envelope_video_tag;
        envelope.message.video.pts_us = 123456789;
        envelope.message.video.codec = 1;
        envelope.message.video.key = true;
        envelope.message.video.access_unit.funcs.encode = encode_bytes;
        envelope.message.video.access_unit.arg = &bytes;
        pb_ostream_t output = pb_ostream_from_buffer(buffer, n + 128);
        if (!pb_encode(&output, eyepiece_rpc_v1_Envelope_fields, &envelope) ||
            output.bytes_written > EP_MAX_PAYLOAD) {
            fprintf(stderr, "encode failed: %s\n", PB_GET_ERROR(&output)); return 1;
        }
        for (size_t batch = 0; batch < batches; batch++) {
            uint64_t start = ns();
            for (size_t i = 0; i < count / batches; i++) {
                eyepiece_rpc_v1_Envelope message;
                ep_bytes_t video, id;
                int rc = ep_wire_decode(buffer, output.bytes_written, &message, &video, &id);
                if (rc || message.which_message != eyepiece_rpc_v1_Envelope_video_tag || video.length != n ||
                    message.message.video.pts_us != 123456789) {
                    fprintf(stderr, "decode failed\n"); return 1;
                }
            }
            samples[batch] = (ns() - start) / (count / batches);
        }
        qsort(samples, batches, sizeof(*samples), compare);
        printf("%zu,%zu,%zu,%llu,%llu\n", n, count, output.bytes_written,
               (unsigned long long)samples[batches / 2],
               (unsigned long long)samples[batches * 95 / 100]);
        free(samples); free(buffer); free(au);
    }
    return 0;
}
