// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../bridge.h"
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int send_message(const eyepiece_rpc_v1_Envelope *message) {
    uint8_t bytes[1100];
    size_t written;
    if (ep_wire_encode(message, bytes, sizeof(bytes), &written)) return -1;
    uint32_t prefix = htonl((uint32_t)written);
    if (fwrite(&prefix, sizeof(prefix), 1, stdout) != 1 ||
        fwrite(bytes, written, 1, stdout) != 1 || fflush(stdout)) return -1;
    return 0;
}

int main(void) {
    eyepiece_rpc_v1_Envelope reply = eyepiece_rpc_v1_Envelope_init_zero;
    reply.which_message = eyepiece_rpc_v1_Envelope_info_tag;
    reply.message.info = (eyepiece_rpc_v1_Info){ EP_VERSION, 1920, 1080, 2, 1,
        eyepiece_rpc_v1_BridgeCapability_CAPABILITY_RECONFIG | eyepiece_rpc_v1_BridgeCapability_CAPABILITY_PANE_ALPHA };
    if (send_message(&reply)) return 1;
    reply = (eyepiece_rpc_v1_Envelope)eyepiece_rpc_v1_Envelope_init_zero;
    reply.which_message = eyepiece_rpc_v1_Envelope_pose_tag;
    reply.message.pose = (eyepiece_rpc_v1_Pose){ 1.25f, -2.5f, 3.75f, 123456, true,
        eyepiece_rpc_v1_PoseSource_POSE_SOURCE_MCU };
    if (send_message(&reply)) return 1;
    reply = (eyepiece_rpc_v1_Envelope)eyepiece_rpc_v1_Envelope_init_zero;
    reply.which_message = eyepiece_rpc_v1_Envelope_log_tag;
    ep_bytes_t log = { (const uint8_t *)"bridge test", 11 };
    ep_wire_set_bytes(&reply.message.log.line, &log);
    if (send_message(&reply)) return 1;
    for (;;) {
        uint32_t prefix;
        size_t got = fread(&prefix, 1, sizeof(prefix), stdin);
        if (!got && feof(stdin)) return 0;
        if (got != sizeof(prefix)) return 1;
        uint32_t length = ntohl(prefix);
        if (!length || length > EP_MAX_PAYLOAD) return 1;
        uint8_t *buffer = malloc(length);
        uint8_t *original = malloc(length);
        if (!buffer || !original) { free(buffer); free(original); return 1; }
        if (fread(buffer, 1, length, stdin) != length) { free(buffer); free(original); return 1; }
        memcpy(original, buffer, length);
        eyepiece_rpc_v1_Envelope message;
        ep_bytes_t video, id;
        if (ep_wire_decode(buffer, length, &message, &video, &id) || memcmp(buffer, original, length)) {
            free(original); free(buffer); return 1;
        }
        free(original);
        reply = (eyepiece_rpc_v1_Envelope)eyepiece_rpc_v1_Envelope_init_zero;
        switch (message.which_message) {
        case eyepiece_rpc_v1_Envelope_video_tag: {
            if (video.data < buffer || video.data + video.length > buffer + length) {
                free(buffer); return 1;
            }
            uint32_t checksum = 0;
            for (size_t i = 0; i < video.length; i++) checksum += video.data[i];
            reply.which_message = eyepiece_rpc_v1_Envelope_frame_tag;
            reply.message.frame = (eyepiece_rpc_v1_Frame){ message.message.video.pts_us,
                message.message.video.codec, message.message.video.key, checksum, (uint32_t)video.length, 0, 0 };
            break;
        }
        case eyepiece_rpc_v1_Envelope_configure_tag: {
            /* Echo every field, then acknowledge with the validity verdict. */
            const eyepiece_rpc_v1_Configure *c = &message.message.configure;
            reply.which_message = eyepiece_rpc_v1_Envelope_frame_tag;
            reply.message.frame = (eyepiece_rpc_v1_Frame){ c->codec, c->layout, c->has_pane,
                c->pane.x, c->pane.y, c->pane.width, c->pane.height };
            if (send_message(&reply)) { free(buffer); return 1; }
            reply = (eyepiece_rpc_v1_Envelope)eyepiece_rpc_v1_Envelope_init_zero;
            reply.which_message = eyepiece_rpc_v1_Envelope_configure_ack_tag;
            reply.message.configure_ack.status = ep_config_valid(c, 1920, 1080) ?
                EP_CONFIGURE_READY : EP_CONFIGURE_INVALID;
            ep_wire_set_bytes(&reply.id, &id);
            break;
        }
        case eyepiece_rpc_v1_Envelope_gui_alpha_tag:
        case eyepiece_rpc_v1_Envelope_pane_alpha_tag: {
            uint32_t alpha = message.which_message == eyepiece_rpc_v1_Envelope_pane_alpha_tag ?
                message.message.pane_alpha.alpha : message.message.gui_alpha.alpha;
            reply.which_message = message.which_message == eyepiece_rpc_v1_Envelope_pane_alpha_tag ?
                eyepiece_rpc_v1_Envelope_pane_alpha_ack_tag : eyepiece_rpc_v1_Envelope_gui_alpha_ack_tag;
            if (reply.which_message == eyepiece_rpc_v1_Envelope_pane_alpha_ack_tag)
                reply.message.pane_alpha_ack.status = alpha > UINT8_MAX ? 1u : alpha;
            else
                reply.message.gui_alpha_ack.status = alpha > UINT8_MAX ? 1u : alpha;
            ep_wire_set_bytes(&reply.id, &id);
            break;
        }
        case eyepiece_rpc_v1_Envelope_ping_tag:
            reply.which_message = eyepiece_rpc_v1_Envelope_pong_tag;
            reply.message.pong = (eyepiece_rpc_v1_Pong){ message.message.ping.pc_us, 987654 };
            break;
        case eyepiece_rpc_v1_Envelope_control_tag:
            reply.which_message = eyepiece_rpc_v1_Envelope_frame_tag;
            reply.message.frame = (eyepiece_rpc_v1_Frame){ message.message.control.operation,
                message.message.control.enabled, 0, 0, 0, 0, 0 };
            break;
        default:
            free(buffer); return 1;
        }
        int rc = send_message(&reply);
        free(buffer);
        if (rc) return 1;
    }
}
