// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef EYEPIECE_BRIDGE_H
#define EYEPIECE_BRIDGE_H
#include <stddef.h>
#include <stdint.h>
#include "eyepiece-rpc.pb.h"

#define EP_PORT        42420
#define EP_VERSION     4
#define EP_MAX_PAYLOAD (4u << 20)
#define EP_SBS_EYE_W 1280
#define EP_SBS_EYE_H 720

/* Packed BGRA uses straight alpha, with alpha luma 16..235 at double width. */
static inline int ep_pane_bgra_valid(const eyepiece_rpc_v1_Pane *p, uint16_t panel_w, uint16_t panel_h) {
    return (p->format == eyepiece_rpc_v1_PaneFormat_PANE_BGRA ||
            p->format == eyepiece_rpc_v1_PaneFormat_PANE_OPAQUE_BGRA) &&
           p->width && p->height && !(p->width & 1) && !(p->height & 1) &&
           (uint64_t)p->x + p->width <= panel_w && (uint64_t)p->y + p->height <= panel_h &&
           (p->format != eyepiece_rpc_v1_PaneFormat_PANE_BGRA || (uint64_t)p->width * 2 <= UINT16_MAX);
}
/* Configure acknowledgement statuses. */
enum {
    EP_CONFIGURE_READY = 0,        /* applied; video may follow, starting with a keyframe */
    EP_CONFIGURE_INVALID = 1,      /* malformed or inconsistent; nothing changed */
    EP_CONFIGURE_FAILED = 2,       /* decoder torn down or lost; the connection is closed */
    EP_CONFIGURE_UNSUPPORTED = 3,  /* valid but not possible here or now; nothing changed */
};

/* Encoded frame size implied by a Configure; 0 on success. A pane sets the size
 * (twice as wide when packed with alpha); otherwise the layout does. */
static inline int ep_config_frame_size(const eyepiece_rpc_v1_Configure *c, uint16_t panel_w,
                                       uint16_t panel_h, uint32_t *width, uint32_t *height) {
    if (c->has_pane) {
        *width = c->pane.width * (c->pane.format == eyepiece_rpc_v1_PaneFormat_PANE_BGRA ? 2 : 1);
        *height = c->pane.height;
        return 0;
    }
    if (c->layout == eyepiece_rpc_v1_StreamLayout_LAYOUT_SBS) {
        *width = 2 * EP_SBS_EYE_W;
        *height = EP_SBS_EYE_H;
        return 0;
    }
    if (c->layout != eyepiece_rpc_v1_StreamLayout_LAYOUT_MONO) return -1;
    *width = panel_w;
    *height = panel_h;
    return 0;
}
static inline int ep_config_valid(const eyepiece_rpc_v1_Configure *c, uint16_t panel_w, uint16_t panel_h) {
    uint32_t width, height;
    return c->codec <= eyepiece_rpc_v1_Codec_CODEC_HEVC &&
           c->layout <= eyepiece_rpc_v1_StreamLayout_LAYOUT_SBS &&
           (!c->has_pane || (c->layout == eyepiece_rpc_v1_StreamLayout_LAYOUT_MONO &&
                             ep_pane_bgra_valid(&c->pane, panel_w, panel_h))) &&
           ep_config_frame_size(c, panel_w, panel_h, &width, &height) == 0;
}
/* Same decoder shape (everything except the pane position). */
static inline int ep_config_same_shape(const eyepiece_rpc_v1_Configure *a, const eyepiece_rpc_v1_Configure *b) {
    if (a->codec != b->codec || a->layout != b->layout || a->has_pane != b->has_pane) return 0;
    return !a->has_pane || (a->pane.width == b->pane.width && a->pane.height == b->pane.height &&
                            a->pane.format == b->pane.format);
}
static inline int ep_config_equal(const eyepiece_rpc_v1_Configure *a, const eyepiece_rpc_v1_Configure *b) {
    return ep_config_same_shape(a, b) && (!a->has_pane || (a->pane.x == b->pane.x && a->pane.y == b->pane.y));
}

typedef struct {
    const uint8_t *data;
    size_t length;
} ep_bytes_t;

/* Decoded video and ID views borrow buffer until it is freed. */
int ep_wire_decode(const uint8_t *buffer, size_t length, eyepiece_rpc_v1_Envelope *message,
                   ep_bytes_t *video, ep_bytes_t *id);
void ep_wire_set_bytes(pb_callback_t *field, ep_bytes_t *bytes);
int ep_wire_encode(const eyepiece_rpc_v1_Envelope *message,
                   uint8_t *output, size_t capacity, size_t *written);

uint64_t now_us(void);
/* $EYEPIECE_TMP/<name>, default /data/local/tmp/<name> (host tests set EYEPIECE_TMP) */
const char *tmp_path(const char *name);
void vlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* YUV 4:2:0 → BGRA for GFX1 panes. */
typedef struct {
    const uint8_t *y, *u, *v;     /* NV12: u = UV plane, v = u + 1            */
    int ys, us, vs;               /* strides                                  */
    int cstep;                    /* chroma sample step: 1 = I420, 2 = NV12   */
    int w, h;
    int full_range;
} yuv_img_t;
/* Packed color | grayscale-alpha video; writes straight-alpha BGRA at half width. */
int yuv_packed_alpha_to_bgra(const yuv_img_t *img, uint8_t *dst, int dst_stride, int threads);
/* NV12 input in uncached memory; planes must contain full strides, including the last row.
 * Staging must not overlap input or output. */
int yuv_packed_alpha_to_bgra_staged(const yuv_img_t *img, uint8_t *staging, size_t staging_size,
                                    uint8_t *dst, int dst_stride, int threads, uint8_t alpha);
int yuv_opaque_to_rgbx_staged(const yuv_img_t *img, uint8_t *staging, size_t staging_size,
                              uint8_t *dst, int dst_stride, int threads);

/* hwdec.c — VC8000D via /dev/vdec0 */
typedef struct hwdec hwdec_t;
typedef void (*hwdec_frame_cb)(void *user, const yuv_img_t *img, uint64_t pts, uint32_t convert_us);
typedef struct {
    int no_reorder, flag24, val2c;   /* -L r,f,v  start-config knobs              */
    int direct_vops;                 /* VOP bitmask for zero-copy scan-out */
    int stereo;                      /* SBS stream: 3D display timing (1901), 0 = mono */
    int pane_output;                 /* convert decoded video for GFX1 */
    int pane_opaque;                 /* normal-width video instead of packed color | alpha */
    uint8_t pane_alpha;              /* multiplies decoded pixel alpha */
    int pane_x, pane_y, pane_w, pane_h;
    int cvt_threads;
    int (*prepare_output)(void *user);
    int (*skip_output)(void *user);
    void (*output_error)(void *user);
} hwdec_tune_t;
hwdec_t *hwdec_open(int codec, int w, int h, int nframes, int sync_dir,
                    const hwdec_tune_t *tune, hwdec_frame_cb cb, void *user);
int  hwdec_submit(hwdec_t *d, const uint8_t *au, int len, uint64_t pts);
int  hwdec_move_overlay(hwdec_t *d, int x, int y);
int  hwdec_set_overlay_alpha(hwdec_t *d, uint8_t alpha);
int  hwdec_drain_overlay(hwdec_t *d);
void hwdec_stop_overlay(hwdec_t *d);
int  hwdec_reconfigure_close(hwdec_t *d);

/* dispout.c — VIDEO0 NV12 scan-out and GFX1 BGRA panes */
typedef struct dispout dispout_t;
typedef void (*dispout_release_cb)(void *user, void *fb);
/* Smaller streams are displayed 1:1, centered. */
dispout_t *dispout_open(int vop_mask, int src_w, int src_h, int stereo_res,
                        dispout_release_cb release, void *user);
dispout_t *dispout_open_pane(int w, int h, int x, int y, int opaque, uint8_t alpha,
                            dispout_release_cb release, void *user);
int  dispout_move_pane(dispout_t *o, int x, int y);
int  dispout_set_pane_alpha(dispout_t *o, uint8_t alpha);
enum { DISPOUT_PUSH_OK = 0, DISPOUT_PUSH_ERROR = -1, DISPOUT_PUSH_REJECTED = -2 };
int  dispout_push(dispout_t *o, void *fb);
int  dispout_close_checked(dispout_t *o);
void dispout_stereo_off(void);
void dispout_reset_stale_3d(void);
int  hwdec_hal_init(void);

/* usb_bulk.c - isolated FunctionFS bulk transport alongside the existing gadget functions */
int  bulk_recover(void);
int  bulk_setup(void);
int  bulk_event_fd(void);
int  bulk_session_selected(void);
int  bulk_tcp_claim(void);
void bulk_tcp_done(void);
void bulk_update_events(void);
int  bulk_pending_client(void);
void bulk_client_done(void);
void bulk_log_diagnostics(void);
void bulk_stop(void);

/* headtrack.c — head attitude from the goggles' MCU (100 Hz, no drone).
 * The DJI bus clients run in a forked helper process (isolates DJI's
 * framework init + signal handlers from the HAL used by hwdec).          */
typedef void (*pose_cb_t)(float yaw, float pitch, float roll);   /* degrees */
int  headtrack_spawn(pose_cb_t cb);       /* helper ready, MCU stream still off */
void headtrack_enable(int on);             /* start/stop the MCU stream (idempotent) */
void headtrack_stop(void);
/* eyepiece_bridge -Z: send only the MCU attitude stop (recovery / stopper). */
int  headtrack_stop_cli(void);

#endif
