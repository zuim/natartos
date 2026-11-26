// SPDX-License-Identifier: AGPL-3.0-or-later
#ifndef EYEPIECE_BRIDGE_H
#define EYEPIECE_BRIDGE_H
#include <stddef.h>
#include <stdint.h>
#include "proto.h"

uint64_t now_us(void);
/* $EYEPIECE_TMP/<name>, default /data/local/tmp/<name> (host tests set EYEPIECE_TMP) */
const char *tmp_path(const char *name);
void vlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* fbout.c — natartos fb_shared producer */
typedef struct {
    int fd;
    uint8_t *map;
    volatile uint32_t *counter;
    size_t slot, size;
    int w, h;
} fbout_t;
int      fbout_open(fbout_t *fb, int w, int h);
uint8_t *fbout_back(fbout_t *fb);
void     fbout_publish(fbout_t *fb);
void     fbout_clear(fbout_t *fb);
void     fbout_fill(fbout_t *fb, uint32_t bgra);   /* both slots; 0 = fully transparent */
int      fbout_stop_requested(void);
void     fbout_close(fbout_t *fb);
int      fbtext_width(const char *s, int scale);
void     fbtext_draw(uint8_t *slot, int fw, int fh, int x, int y, int scale, const char *s, uint32_t bgra);

/* yuv2fb.c — YUV 4:2:0 → BGRA into the fb back slot (fit + scale, threaded) */
typedef struct {
    const uint8_t *y, *u, *v;     /* NV12: u = UV plane, v = u + 1            */
    int ys, us, vs;               /* strides                                  */
    int cstep;                    /* chroma sample step: 1 = I420, 2 = NV12   */
    int w, h;
    int full_range;
} yuv_img_t;
void yuv_to_fb(fbout_t *fb, const yuv_img_t *img, int threads);

/* swdec.c — software decoder (device libavcodec) */
typedef struct swdec swdec_t;
swdec_t *swdec_open(int codec, int threads, int fast);
/* 1 = frame shown, 0 = no picture yet, -1 = error */
int swdec_decode(swdec_t *d, const uint8_t *au, int len, fbout_t *fb,
                 int cvt_threads, uint32_t *decode_us, uint32_t *convert_us);

/* hwdec.c — VC8000D via /dev/vdec0 (experimental) */
typedef struct hwdec hwdec_t;
/* img == NULL: the frame went straight to the display plane (stats only) */
typedef void (*hwdec_frame_cb)(void *user, const yuv_img_t *img, uint64_t pts);
typedef struct {
    int no_reorder, flag24, val2c;   /* -L r,f,v  start-config knobs              */
    int direct_vops;                 /* -O plane[:mask]: VOP bitmask for zero-copy
                                        scan-out on VIDEO0 (0 = convert into fb_shared) */
    int stereo;                      /* SBS stream: 3D display timing (1901), 0 = mono */
} hwdec_tune_t;
hwdec_t *hwdec_open(int codec, int w, int h, int nframes, int sync_dir,
                    const hwdec_tune_t *tune, hwdec_frame_cb cb, void *user);
int  hwdec_submit(hwdec_t *d, const uint8_t *au, int len, uint64_t pts);
void hwdec_close(hwdec_t *d);

/* dispout.c — zero-copy NV12 scan-out on DUSS_DISP_PLANE_VIDEO0 (experimental) */
typedef struct dispout dispout_t;
typedef void (*dispout_release_cb)(void *user, void *fb);
/* src_w/src_h: stream size; ≠ 1920x1080 → plane scaler to the panel (aspect kept) */
dispout_t *dispout_open(int vop_mask, int src_w, int src_h, int stereo_res, dispout_release_cb release, void *user);
int  dispout_push(dispout_t *o, void *fb);
void dispout_close(dispout_t *o);
void dispout_qsync_off(void);
void dispout_stereo_off(void);
void dispout_reset_stale_3d(void);
int  hwdec_hal_init(void);

/* rndis.c — USB network (RNDIS + DHCP) next to adb; 0 = active */
int rndis_setup(void);
#define RNDIS_ADDR "192.168.60.2"

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
int  headtrack_mcu_stop_only(void);
int  headtrack_stop_cli(void);

#endif
