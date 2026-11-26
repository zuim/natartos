// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef EYEPIECE_WIRE_TEST_HELPERS_H
#define EYEPIECE_WIRE_TEST_HELPERS_H

#include "../bridge.h"
#include "eyepiece-rpc.pb.h"
#include <arpa/inet.h>
#include <assert.h>
#include <pb_decode.h>
#include <pb_encode.h>
#include <string.h>
#include <unistd.h>

typedef struct {
    uint8_t *data;
    size_t length;
    size_t capacity;
} test_bytes;

static bool test_encode_bytes(pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
    const test_bytes *bytes = *arg;
    return pb_encode_tag_for_field(stream, field) &&
           pb_encode_string(stream, bytes->data, bytes->length);
}

static bool test_decode_bytes(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    (void)field;
    test_bytes *bytes = *arg;
    if (stream->bytes_left > bytes->capacity) return false;
    bytes->length = stream->bytes_left;
    return pb_read(stream, bytes->data, bytes->length);
}

static bool test_decode_submessage(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    return test_decode_bytes(stream, field, arg);
}

static void test_write_all(int fd, const void *buf, size_t length) {
    const uint8_t *p = buf;
    while (length) {
        ssize_t n = write(fd, p, length);
        assert(n > 0);
        p += n;
        length -= (size_t)n;
    }
}

static void test_read_all(int fd, void *buf, size_t length) {
    uint8_t *p = buf;
    while (length) {
        ssize_t n = read(fd, p, length);
        assert(n > 0);
        p += n;
        length -= (size_t)n;
    }
}

static void test_send_id(int fd, uint32_t type, const pb_msgdesc_t *fields, const void *message, const char *id) {
    uint8_t body[4096], packet_data[4200];
    pb_ostream_t stream = pb_ostream_from_buffer(body, sizeof(body));
    assert(pb_encode(&stream, fields, message));
    size_t body_length = stream.bytes_written;
    stream = pb_ostream_from_buffer(packet_data, sizeof(packet_data));
    if (id) {
        assert(pb_encode_tag(&stream, PB_WT_STRING, eyepiece_rpc_v1_Envelope_id_tag));
        assert(pb_encode_string(&stream, (const uint8_t *)id, strlen(id)));
    }
    assert(pb_encode_tag(&stream, PB_WT_STRING, type));
    assert(pb_encode_string(&stream, body, body_length));
    uint32_t length = htonl((uint32_t)stream.bytes_written);
    test_write_all(fd, &length, sizeof(length));
    test_write_all(fd, packet_data, stream.bytes_written);
}

static void test_send(int fd, uint32_t type, const pb_msgdesc_t *fields, const void *message) {
    test_send_id(fd, type, fields, message, NULL);
}

static void test_receive_id(int fd, uint32_t type, const pb_msgdesc_t *fields, void *message, const char *id) {
    for (;;) {
        uint32_t length;
        uint8_t packet_data[4200], body[4096];
        test_read_all(fd, &length, sizeof(length));
        length = ntohl(length);
        assert(length && length <= sizeof(packet_data));
        test_read_all(fd, packet_data, length);
        test_bytes bytes = { body, 0, sizeof(body) };
        uint8_t id_data[32];
        test_bytes id_bytes = { id_data, 0, sizeof(id_data) };
        eyepiece_rpc_v1_Envelope packet = eyepiece_rpc_v1_Envelope_init_zero;
        packet.id.funcs.decode = test_decode_bytes;
        packet.id.arg = &id_bytes;
        packet.cb_message.funcs.decode = test_decode_submessage;
        packet.cb_message.arg = &bytes;
        pb_istream_t input = pb_istream_from_buffer(packet_data, length);
        assert(pb_decode(&input, eyepiece_rpc_v1_Envelope_fields, &packet));
        if (packet.which_message != type) continue;
        if (id) assert(id_bytes.length == strlen(id) && !memcmp(id_bytes.data, id, id_bytes.length));
        input = pb_istream_from_buffer(body, bytes.length);
        assert(pb_decode(&input, fields, message));
        return;
    }
}

static void test_receive(int fd, uint32_t type, const pb_msgdesc_t *fields, void *message) {
    test_receive_id(fd, type, fields, message, NULL);
}

#endif
