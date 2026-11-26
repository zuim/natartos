// SPDX-License-Identifier: AGPL-3.0-or-later
/* Run against an isolated bridge started with -N -H -1. */
#include "wire_test_helpers.h"
#include <arpa/inet.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

static void video(int fd, uint64_t pts, uint8_t codec, int key) {
    uint8_t au = 1;
    test_bytes bytes = { &au, 1, 1 };
    eyepiece_rpc_v1_Video message = eyepiece_rpc_v1_Video_init_zero;
    message.pts_us = pts;
    message.codec = codec;
    message.key = key != 0;
    message.access_unit.funcs.encode = test_encode_bytes;
    message.access_unit.arg = &bytes;
    test_send(fd, eyepiece_rpc_v1_Envelope_video_tag, eyepiece_rpc_v1_Video_fields, &message);
}

static void configure(int fd, const char *id, const eyepiece_rpc_v1_Configure *request, uint32_t status) {
    eyepiece_rpc_v1_Acknowledgement ack = eyepiece_rpc_v1_Acknowledgement_init_zero;
    test_send_id(fd, eyepiece_rpc_v1_Envelope_configure_tag, eyepiece_rpc_v1_Configure_fields, request, id);
    test_receive_id(fd, eyepiece_rpc_v1_Envelope_configure_ack_tag, eyepiece_rpc_v1_Acknowledgement_fields, &ack, id);
    assert(ack.status == status);
}

static void pane_alpha(int fd, const char *id, uint32_t value, uint32_t status) {
    eyepiece_rpc_v1_PaneAlpha alpha = { value };
    eyepiece_rpc_v1_Acknowledgement ack = eyepiece_rpc_v1_Acknowledgement_init_zero;
    test_send_id(fd, eyepiece_rpc_v1_Envelope_pane_alpha_tag, eyepiece_rpc_v1_PaneAlpha_fields, &alpha, id);
    test_receive_id(fd, eyepiece_rpc_v1_Envelope_pane_alpha_ack_tag, eyepiece_rpc_v1_Acknowledgement_fields, &ack, id);
    assert(ack.status == status);
}

static void shown(int fd, uint64_t pts, uint8_t codec, int key) {
    eyepiece_rpc_v1_Frame frame = eyepiece_rpc_v1_Frame_init_zero;
    video(fd, pts, codec, key);
    test_receive(fd, eyepiece_rpc_v1_Envelope_frame_tag, eyepiece_rpc_v1_Frame_fields, &frame);
    assert(frame.pts_us == pts);
}

int main(int argc, char **argv) {
    assert(argc == 2);
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    assert(fd >= 0);
    struct timeval timeout = { 10, 0 };
    assert(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons((uint16_t)atoi(argv[1])) };
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    eyepiece_rpc_v1_Info info = eyepiece_rpc_v1_Info_init_zero;
    test_receive(fd, eyepiece_rpc_v1_Envelope_info_tag, eyepiece_rpc_v1_Info_fields, &info);
    assert(info.version == EP_VERSION);
    assert(info.flags & eyepiece_rpc_v1_BridgeCapability_CAPABILITY_RECONFIG);
    assert(info.flags & eyepiece_rpc_v1_BridgeCapability_CAPABILITY_PANE_ALPHA);

    /* There is no pane to apply opacity to before the first Configure. */
    pane_alpha(fd, "5", 128, 2);

    eyepiece_rpc_v1_Configure opaque = {
        eyepiece_rpc_v1_Codec_CODEC_H264, eyepiece_rpc_v1_StreamLayout_LAYOUT_MONO, true,
        { 100, 50, 640, 480, eyepiece_rpc_v1_PaneFormat_PANE_OPAQUE_BGRA } };
    configure(fd, "6", &opaque, EP_CONFIGURE_READY);
    pane_alpha(fd, "7", 128, 0);
    pane_alpha(fd, "8", 256, 1);
    shown(fd, 1, eyepiece_rpc_v1_Codec_CODEC_H264, 1);

    /* Invalid requests are acknowledged and leave the active stream untouched. */
    eyepiece_rpc_v1_Configure invalid = opaque;
    invalid.pane.x = 1800;
    configure(fd, "9", &invalid, EP_CONFIGURE_INVALID);
    invalid = opaque;
    invalid.codec = 2;
    configure(fd, "10", &invalid, EP_CONFIGURE_INVALID);
    invalid = opaque;
    invalid.layout = eyepiece_rpc_v1_StreamLayout_LAYOUT_SBS;
    configure(fd, "11", &invalid, EP_CONFIGURE_INVALID);
    invalid = opaque;
    invalid.pane.format = eyepiece_rpc_v1_PaneFormat_PANE_NV12;
    configure(fd, "12", &invalid, EP_CONFIGURE_INVALID);
    shown(fd, 2, eyepiece_rpc_v1_Codec_CODEC_H264, 1);

    /* Replacing the stream: new shape, HEVC; video waits for a keyframe. */
    eyepiece_rpc_v1_Configure packed = {
        eyepiece_rpc_v1_Codec_CODEC_HEVC, eyepiece_rpc_v1_StreamLayout_LAYOUT_MONO, true,
        { 300, 200, 320, 240, eyepiece_rpc_v1_PaneFormat_PANE_BGRA } };
    configure(fd, "13", &packed, EP_CONFIGURE_READY);
    pane_alpha(fd, "14", 255, 0);
    video(fd, 3, eyepiece_rpc_v1_Codec_CODEC_HEVC, 0);
    shown(fd, 4, eyepiece_rpc_v1_Codec_CODEC_HEVC, 1);

    /* Repeating the configuration only asks for a keyframe; moving the pane keeps the decoder. */
    configure(fd, "15", &packed, EP_CONFIGURE_READY);
    video(fd, 5, eyepiece_rpc_v1_Codec_CODEC_HEVC, 0);
    packed.pane.x = 1000;
    configure(fd, "16", &packed, EP_CONFIGURE_READY);
    shown(fd, 6, eyepiece_rpc_v1_Codec_CODEC_HEVC, 1);

    /* Dropping the pane selects the full-panel plane; stereo is not available here. */
    eyepiece_rpc_v1_Configure panel = {
        eyepiece_rpc_v1_Codec_CODEC_H264, eyepiece_rpc_v1_StreamLayout_LAYOUT_MONO, false, {0} };
    configure(fd, "17", &panel, EP_CONFIGURE_READY);
    pane_alpha(fd, "18", 255, 2);
    shown(fd, 7, eyepiece_rpc_v1_Codec_CODEC_H264, 1);
    eyepiece_rpc_v1_Configure stereo = panel;
    stereo.layout = eyepiece_rpc_v1_StreamLayout_LAYOUT_SBS;
    configure(fd, "19", &stereo, EP_CONFIGURE_UNSUPPORTED);
    shown(fd, 8, eyepiece_rpc_v1_Codec_CODEC_H264, 1);

    /* And back to a pane; the earlier opacity request survives the transitions. */
    configure(fd, "20", &opaque, EP_CONFIGURE_READY);
    pane_alpha(fd, "21", 64, 0);
    shown(fd, 9, eyepiece_rpc_v1_Codec_CODEC_H264, 1);
    close(fd);
    puts("configuration protocol passed");
    return 0;
}
