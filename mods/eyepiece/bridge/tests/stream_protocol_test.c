// SPDX-License-Identifier: AGPL-3.0-or-later
/* Run each case against a fresh bridge started with -N -H -1. */
#include "wire_test_helpers.h"
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>

int main(int argc, char **argv) {
    assert(argc == 3);
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    assert(fd >= 0);
    struct timeval timeout = { 3, 0 };
    assert(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == 0);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons((uint16_t)atoi(argv[1])) };
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    assert(connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0);
    eyepiece_rpc_v1_Info info = eyepiece_rpc_v1_Info_init_zero;
    test_receive(fd, eyepiece_rpc_v1_Envelope_info_tag, eyepiece_rpc_v1_Info_fields, &info);
    int wrong_codec = strcmp(argv[2], "wrong-codec") == 0;
    assert(wrong_codec || strcmp(argv[2], "before-configure") == 0);
    if (wrong_codec) {
        eyepiece_rpc_v1_Configure configure = {
            eyepiece_rpc_v1_Codec_CODEC_H264, eyepiece_rpc_v1_StreamLayout_LAYOUT_MONO, false, {0} };
        eyepiece_rpc_v1_Acknowledgement ack = eyepiece_rpc_v1_Acknowledgement_init_zero;
        test_send_id(fd, eyepiece_rpc_v1_Envelope_configure_tag, eyepiece_rpc_v1_Configure_fields, &configure, "1");
        test_receive_id(fd, eyepiece_rpc_v1_Envelope_configure_ack_tag, eyepiece_rpc_v1_Acknowledgement_fields, &ack, "1");
        assert(ack.status == 0);
    }
    uint8_t au = 1;
    test_bytes bytes = { &au, 1, 1 };
    eyepiece_rpc_v1_Video video = eyepiece_rpc_v1_Video_init_zero;
    video.codec = wrong_codec ? eyepiece_rpc_v1_Codec_CODEC_HEVC : eyepiece_rpc_v1_Codec_CODEC_H264;
    video.key = true;
    video.access_unit.funcs.encode = test_encode_bytes;
    video.access_unit.arg = &bytes;
    test_send(fd, eyepiece_rpc_v1_Envelope_video_tag, eyepiece_rpc_v1_Video_fields, &video);
    uint8_t buffer[4096];
    ssize_t count;
    while ((count = recv(fd, buffer, sizeof(buffer), 0)) > 0) {}
    assert(count == 0);
    close(fd);
    puts("invalid video stream rejected");
    return 0;
}
