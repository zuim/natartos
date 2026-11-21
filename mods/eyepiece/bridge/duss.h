// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * duss.h — minimal, reverse-engineered view of DJI's on-device message bus
 * ("DUSS"/DUML framework, /system/lib64/libduml_frwk.so) as used by
 * /system/bin/dji_mb_ctrl, recovered by static analysis of the firmware.
 *
 * We never link against the DJI libraries at build time: everything is
 * resolved with dlopen/dlsym at runtime, so a symbol that is missing on a
 * different firmware just disables the feature instead of failing to start.
 */
#ifndef EYEPIECE_DUSS_H
#define EYEPIECE_DUSS_H

#include <stdint.h>

/* Host address = (type << 8) | index (from /system/etc/dji.json).        */
#define HOST_IMU_MCU      0x1902   /* imu:2 = the MCU, via dji_sys ICC          */
#define HOST_TEST_DIAG    0x1e07   /* test/diag: our command client             */
#define HOST_MEDIA_PLAYER 0x1e03   /* media_service/media_player: MCU attitude  */

#define DUSS_MSG_ID(set, id) ((uint32_t)(((uint32_t)(set) << 16) | (uint16_t)(id)))

/* attrib bits (seen in dji_mb_ctrl / libapp_util):
 *   0x0040  ack wanted ("ack after exec")   – needed for *_sync_* sends
 *   0x1000  sequence number supplied by caller
 *   (attrib & 0x60) == 0  →  fire-and-forget, no ack callback required   */
#define DUSS_ATTR_NEED_ACK  0x0040
#define DUSS_ATTR_SEQ       0x1000

/* Message as passed to duss_event_send_*() and as received by callbacks.  */
typedef struct __attribute__((packed)) {
    uint32_t msg_id;       /* 0x00  cmd_set << 16 | cmd_id                   */
    uint16_t dst_machine;  /* 0x04  0 = this machine                         */
    uint16_t dst_host;     /* 0x06  DUSS_HOST(type, index)                   */
    uint16_t attrib;       /* 0x08                                           */
    uint16_t _pad0;        /* 0x0a                                           */
    uint16_t seq;          /* 0x0c                                           */
    uint16_t _pad1;        /* 0x0e                                           */
    uint32_t len;          /* 0x10  payload length                           */
    uint8_t  data[];       /* 0x14  payload                                  */
} duss_msg_t;
#define DUSS_MSG_HDR 0x14

/* Dynamic command descriptor (0x28 bytes, copied by the framework).       */
typedef int (*duss_cmd_cb)(void *client, duss_msg_t *ev, void *user);
typedef struct {
    duss_cmd_cb req_cb;    /* 0x00 */
    void       *ack_cb;    /* 0x08 */
    uint32_t    flags;     /* 0x10 */
    uint32_t    _pad;
    void       *user;      /* 0x18 */
    void       *reserved;  /* 0x20 */
} duss_cmd_desc_t;

/* Sizes of the opaque structs dji_mb_ctrl passes (zero-initialised).     */
#define DUSS_SKETCH_ATTR_SZ  0xd0
#define DUSS_ROUTE_TABLE_SZ  0x810
#define DUSS_CLIENT_ATTR1_SZ 0x778
#define DUSS_CLIENT_ATTR2_SZ 0x3bc

typedef struct {
    int  (*sketch_initialize)(void *attr);
    const char *(*sketch_get_config_file_name)(void);
    int  (*mb_create_route_table)(const char *cfg, const char *service,
                                  const char *role, uint16_t *host, void *rt);
    void (*mb_destroy_route_table)(void *rt);
    int  (*event_create_client)(void *rt, uint16_t host, void *attr1,
                                void *attr2, void *unused, void **client);
    int  (*event_start_client)(void *client);
    int  (*event_destroy_client)(void *client);
    int  (*event_send_sync_v2)(void *client, duss_msg_t *msg, uint32_t timeout_us,
                               void *resp, uint32_t *resp_len);
    int  (*event_register_dynamic_command_with_user_data)(void *client, int cmd_set,
                                                          uint32_t cmd_id,
                                                          duss_cmd_desc_t *desc);
} duss_api_t;

/* ---- Commands used by eyepiece ------------------------------------------ */

/* MCU head attitude stream (no drone needed):
 * 0x15/0x31 {1,0,0,0,0} → imu:2 starts it, {0,0,0,0,0} stops it; the MCU
 * sends 0x15/0x32 at 100 Hz via dji_media_server to test:3.              */
#define MSG_MCU_ATTI_SET   DUSS_MSG_ID(0x15, 0x31)
#define MSG_MCU_ATTI_MSG   DUSS_MSG_ID(0x15, 0x32)
#define MCU_ATTI_SET_LEN   5
#define MCU_ATTI_MSG_LEN   26   /* f32 w,x,y,z; u32 t_us; u32 0; u16 rate */

#endif
