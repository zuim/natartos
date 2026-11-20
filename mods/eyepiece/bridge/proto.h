// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * proto.h — eyepiece wire protocol between the PC (eyepiece-pc) and the
 * goggles (eyepiece_bridge).  One TCP connection, carried over USB by
 * `adb forward tcp:42420 tcp:42420`; video goes PC→goggles, head pose and
 * stats come back on the same socket (single combined channel).
 *
 * Every message: 8-byte header + payload, all little-endian.
 */
#ifndef EYEPIECE_PROTO_H
#define EYEPIECE_PROTO_H
#include <stdint.h>

#define EP_PORT        42420
#define EP_VERSION     1
#define EP_MAX_PAYLOAD (4u << 20)

typedef struct __attribute__((packed)) {
    uint8_t  magic0;   /* 'E' */
    uint8_t  magic1;   /* 'P' */
    uint8_t  type;
    uint8_t  flags;
    uint32_t len;      /* payload bytes */
} ep_hdr_t;

enum {
    /* PC → goggles */
    EP_VIDEO  = 0x01,  /* ep_video_t + Annex-B access unit; flags bit0 = keyframe */
    EP_PING   = 0x02,  /* u64 t_pc_us                                            */
    EP_CTRL   = 0x03,  /* u8 op (EP_OP_*)                                        */
    EP_STREAM = 0x04,  /* ep_stream_t: size/codec of the following video (needed by
                          the hardware decoder before it can allocate buffers)    */
    /* goggles → PC */
    EP_POSE   = 0x81,  /* ep_pose_t                                              */
    EP_PONG   = 0x82,  /* u64 t_pc_us, u64 t_dev_us                              */
    EP_LOG    = 0x83,  /* UTF-8 text                                             */
    EP_FRAME  = 0x84,  /* ep_frame_t, one per displayed frame                    */
    EP_INFO   = 0x85,  /* ep_info_t, sent once on connect                        */
};

#define EP_FLAG_KEY 0x01

enum { EP_CODEC_H264 = 0, EP_CODEC_HEVC = 1 };
enum { EP_OP_RECENTER = 1, EP_OP_QUIT = 2 };

typedef struct __attribute__((packed)) {
    uint64_t pts_us;   /* PC capture time (PC monotonic clock)                  */
    uint8_t  codec;    /* EP_CODEC_*                                            */
    uint8_t  rsv[3];
} ep_video_t;          /* followed by the access unit                           */

typedef struct __attribute__((packed)) {
    uint16_t width, height;
    uint8_t  codec;    /* EP_CODEC_*                                            */
    uint8_t  fps;
    uint16_t rsv;
} ep_stream_t;

enum { EP_POSE_SRC_MCU = 3 };   /* 1, 2: retired drone-fed sources (arhome push, hook) */

typedef struct __attribute__((packed)) {
    uint64_t t_dev_us; /* goggles monotonic clock when the sample arrived       */
    float    yaw_deg;  /* + = right                                             */
    float    pitch_deg;/* + = up                                                */
    float    roll_deg; /* + = right ear down                                    */
    uint8_t  active;   /* always 1                                              */
    uint8_t  source;   /* EP_POSE_SRC_*                                         */
    uint16_t rsv;
} ep_pose_t;

typedef struct __attribute__((packed)) {
    uint64_t pts_us;       /* echo of ep_video_t.pts_us                         */
    uint64_t t_recv_us;    /* goggles clock: AU fully received                  */
    uint64_t t_shown_us;   /* goggles clock: frame published to fb_shared       */
    uint32_t decode_us;
    uint32_t convert_us;
    uint32_t queue_depth;  /* AUs waiting for the decoder                       */
    uint32_t dropped;      /* AUs dropped so far (overflow / waiting for IDR)   */
} ep_frame_t;

typedef struct __attribute__((packed)) {
    uint32_t version;      /* EP_VERSION                                        */
    uint16_t fb_w, fb_h;   /* output framebuffer size                           */
    uint8_t  decoder;      /* 0 = none, 1 = software (libavcodec), 2 = hardware (VC8000D) */
    uint8_t  headtrack;    /* bit0 = MCU attitude stream received                */
    uint16_t rsv;
} ep_info_t;

#endif
