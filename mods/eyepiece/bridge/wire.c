// SPDX-License-Identifier: AGPL-3.0-or-later
#include "bridge.h"
#include <pb_decode.h>
#include <pb_encode.h>
#include <stdbool.h>

static bool read_bytes(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    (void)field;
    ep_bytes_t *view = *arg;
    view->data = stream->state;
    view->length = stream->bytes_left;
    return pb_read(stream, NULL, stream->bytes_left);
}

static bool write_bytes(pb_ostream_t *stream, const pb_field_t *field, void *const *arg) {
    const ep_bytes_t *view = *arg;
    return pb_encode_tag_for_field(stream, field) &&
           pb_encode_string(stream, view->data, view->length);
}

void ep_wire_set_bytes(pb_callback_t *field, ep_bytes_t *bytes) {
    field->funcs.encode = write_bytes;
    field->arg = bytes;
}

static bool read_submessage(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    (void)stream;
    if (field->tag == eyepiece_rpc_v1_Envelope_video_tag) {
        eyepiece_rpc_v1_Video *video = field->pData;
        if (video->access_unit.funcs.decode != read_bytes) {
            ep_bytes_t *view = *arg;
            view->data = NULL;
            view->length = 0;
        }
        video->access_unit.funcs.decode = read_bytes;
        video->access_unit.arg = *arg;
    }
    return true;
}

static int valid_id(ep_bytes_t id) {
    if (!id.length || id.length > 10 || (id.length > 1 && id.data[0] == '0')) return 0;
    uint32_t result = 0;
    for (size_t i = 0; i < id.length; i++) {
        if (id.data[i] < '0' || id.data[i] > '9') return 0;
        uint32_t digit = id.data[i] - '0';
        if (result > (UINT32_MAX - digit) / 10) return 0;
        result = result * 10 + digit;
    }
    return 1;
}

int ep_wire_decode(const uint8_t *buffer, size_t length, eyepiece_rpc_v1_Envelope *message,
                   ep_bytes_t *video, ep_bytes_t *id) {
    *message = (eyepiece_rpc_v1_Envelope)eyepiece_rpc_v1_Envelope_init_zero;
    *video = (ep_bytes_t){ buffer, 0 };
    *id = (ep_bytes_t){ buffer, 0 };
    message->id.funcs.decode = read_bytes;
    message->id.arg = id;
    message->cb_message.funcs.decode = read_submessage;
    message->cb_message.arg = video;
    pb_istream_t input = pb_istream_from_buffer(buffer, length);
    if (!pb_decode(&input, eyepiece_rpc_v1_Envelope_fields, message)) return -1;

    switch (message->which_message) {
    case eyepiece_rpc_v1_Envelope_video_tag:
        return message->message.video.codec <= eyepiece_rpc_v1_Codec_CODEC_HEVC && video->length ? 0 : -1;
    case eyepiece_rpc_v1_Envelope_ping_tag:
        return 0;
    case eyepiece_rpc_v1_Envelope_control_tag:
        return message->message.control.operation <= UINT8_MAX ? 0 : -1;
    case eyepiece_rpc_v1_Envelope_configure_tag:
        /* Inconsistent contents are acknowledged as invalid; only the correlation ID is structural. */
        return valid_id(*id) ? 0 : -1;
    case eyepiece_rpc_v1_Envelope_gui_alpha_tag:
    case eyepiece_rpc_v1_Envelope_pane_alpha_tag:
        return valid_id(*id) ? 0 : -1;
    default:
        return -1;
    }
}

int ep_wire_encode(const eyepiece_rpc_v1_Envelope *message,
                   uint8_t *output, size_t capacity, size_t *written) {
    pb_ostream_t stream = pb_ostream_from_buffer(output, capacity);
    if (!pb_encode(&stream, eyepiece_rpc_v1_Envelope_fields, message)) return -1;
    *written = stream.bytes_written;
    return 0;
}
