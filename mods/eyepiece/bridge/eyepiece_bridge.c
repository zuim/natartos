// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * eyepiece_bridge — goggles side of Eyepiece (DJI Goggles 3 as a USB PC display
 * with head tracking).
 *
 *   PC ──adb forward tcp:42420──► eyepiece_bridge ──► decode ──► fb_shared ──► natartos overlay ──► display
 *   PC ◄── head pose / frame stats ── eyepiece_bridge ◄── MCU head attitude (bus, 100 Hz)
 *
 * Runs as root from `adb shell` or from the natartos "Eyepiece" menu entry.
 * Writes only /data/local/tmp/{fb_shared,fb_override,eyepiece.log}; never
 * touches partitions, /system or /vendor.  See mods/eyepiece/README.md.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include "bridge.h"

/* ---- options ------------------------------------------------------------ */
static int  opt_port = EP_PORT;
static int  opt_fb_w = 1920, opt_fb_h = 1080;
static int  opt_dec_threads = 4, opt_cvt_threads = 4, opt_fast = 0;
static int  opt_video = 1, opt_headtrack = 1, opt_back = 0, opt_once = 0, opt_mcu_stop = 0;
static int  opt_any_addr = 0, opt_null_dec = 0;
/* default: VC8000D + zero-copy scan-out on both eyes (measured on device: 60 fps,
 * decode 2.7 ms); falls back to software decode if it fails to open.     */
static int  opt_hw = 1, opt_hw_frames = 12, opt_hw_sync = -1;   /* -D, -F, -S */
static int  opt_direct = 1;  /* -O VOP mask (VOP0 drives both eyes via DSI2DSI), 0 = fb */
static hwdec_tune_t opt_hw_tune = { 1, 0, 0, 0 };                 /* -L, -O */
static int opt_gui_transparent = 1;                               /* -T clears it */
static const char *opt_log = NULL;   /* default: tmp_path("eyepiece.log") */

/* ---- logging / time ------------------------------------------------------ */
static FILE *g_log;
static pthread_mutex_t g_log_mu = PTHREAD_MUTEX_INITIALIZER;
static volatile sig_atomic_t g_exit;

uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

static void send_log_to_client(const char *s);

const char *tmp_path(const char *name) {
    /* a few distinct names, each resolved once — static per-name storage */
    static struct { const char *name; char path[256]; } cache[8];
    for (int i = 0; i < 8; i++) {
        if (cache[i].name == name) return cache[i].path;
        if (!cache[i].name) {
            const char *d = getenv("EYEPIECE_TMP");
            snprintf(cache[i].path, sizeof(cache[i].path), "%s/%s", d && *d ? d : "/data/local/tmp", name);
            cache[i].name = name;
            return cache[i].path;
        }
    }
    return name;
}

void vlog(const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    pthread_mutex_lock(&g_log_mu);
    fprintf(stderr, "[eyepiece] %s\n", buf);
    if (g_log) { fprintf(g_log, "%llu %s\n", (unsigned long long)(now_us() / 1000), buf); fflush(g_log); }
    pthread_mutex_unlock(&g_log_mu);
    send_log_to_client(buf);
}

/* ---- client socket ------------------------------------------------------- */
static int g_cfd = -1;
static int g_ls = -1;
static pthread_mutex_t g_tx_mu = PTHREAD_MUTEX_INITIALIZER;

static int write_all(int fd, const void *p, size_t n) {
    const uint8_t *b = p;
    while (n) {
        ssize_t w = send(fd, b, n, MSG_NOSIGNAL);
        if (w < 0) { if (errno == EINTR) continue; return -1; }
        b += w; n -= (size_t)w;
    }
    return 0;
}

static int read_all(int fd, void *p, size_t n) {
    uint8_t *b = p;
    while (n) {
        ssize_t r = recv(fd, b, n, 0);
        if (r == 0) return -1;
        if (r < 0) { if (errno == EINTR && !g_exit) continue; return -1; }
        b += r; n -= (size_t)r;
    }
    return 0;
}

static void tx(uint8_t type, const void *payload, uint32_t len) {
    pthread_mutex_lock(&g_tx_mu);
    if (g_cfd >= 0) {
        ep_hdr_t h = { 'E', 'P', type, 0, len };
        if (write_all(g_cfd, &h, sizeof(h)) < 0 || (len && write_all(g_cfd, payload, len) < 0)) {
            shutdown(g_cfd, SHUT_RDWR);   /* reader notices and cleans up */
        }
    }
    pthread_mutex_unlock(&g_tx_mu);
}

static __thread int t_in_log;
static void send_log_to_client(const char *s) {
    if (t_in_log) return;
    t_in_log = 1;
    tx(EP_LOG, s, (uint32_t)strlen(s));
    t_in_log = 0;
}

/* ---- head pose ----------------------------------------------------------- */
static pthread_mutex_t g_pose_mu = PTHREAD_MUTEX_INITIALIZER;
static float g_yaw0;
static int g_recenter = 1;      /* first sample defines "straight ahead" */
static uint32_t g_pose_count;
static uint8_t g_ht_flags;

/* MCU head attitude (helper process, 100 Hz).  Pitch/roll are gravity-
 * referenced and kept; yaw is arbitrary and drifts → recenter yaw only.   */
static void on_pose(float yaw, float pitch, float roll) {
    pthread_mutex_lock(&g_pose_mu);
    if (g_recenter) { g_yaw0 = yaw; g_recenter = 0; }
    float y = yaw - g_yaw0;
    g_pose_count++;
    pthread_mutex_unlock(&g_pose_mu);
    while (y > 180.f) y -= 360.f;
    while (y < -180.f) y += 360.f;
    g_ht_flags |= 1;
    ep_pose_t m = { now_us(), y, pitch, roll, 1, EP_POSE_SRC_MCU, 0 };
    tx(EP_POSE, &m, sizeof(m));
}

static void *pose_thread(void *arg) {
    (void)arg;
    uint64_t last_tick = 0;
    while (!g_exit) {
        uint64_t t = now_us();
        if (t - last_tick > 500000) {
            last_tick = t;
            if (opt_video && fbout_stop_requested()) {
                vlog("fb_stop seen (long-hold Back) → exiting");
                g_exit = 1;
            }
        }
        usleep(4000);
    }
    /* signals may land on any thread: make sure accept()/recv() wake up   */
    if (g_ls >= 0) shutdown(g_ls, SHUT_RDWR);
    pthread_mutex_lock(&g_tx_mu);
    if (g_cfd >= 0) shutdown(g_cfd, SHUT_RDWR);
    pthread_mutex_unlock(&g_tx_mu);
    return NULL;
}

/* ---- Back key (optional, for menu launches) ------------------------------ */
static void *back_thread(void *arg) {
    (void)arg;
    struct { long sec, usec; uint16_t type, code; int32_t value; } ev;
    int fd = open("/dev/input/event0", O_RDONLY);
    if (fd < 0) { vlog("back: cannot open event0: %s", strerror(errno)); return NULL; }
    /* A full click only: press then release within 1.5 s (longer = the
     * overlay's own long-hold exit), and nothing in the first 2 s — a stray
     * Back event (seen once right after a dji_glasses restart)
     * must not end the session.                                            */
    uint64_t t_start = now_us(), t_down = 0;
    while (!g_exit && read(fd, &ev, sizeof(ev)) == (ssize_t)sizeof(ev)) {
        if (ev.type != 1 /* EV_KEY */ || ev.code != 158 /* KEY_BACK */) continue;
        uint64_t t = now_us();
        vlog("back: key %s at +%.2f s", ev.value ? "down" : "up", (t - t_start) / 1e6);
        if (t - t_start < 2000000) { t_down = 0; continue; }
        if (ev.value == 1) { t_down = t; continue; }
        if (ev.value == 0 && t_down && t - t_down < 1500000) {
            vlog("Back clicked → exiting");
            g_exit = 1;               /* pose_thread wakes accept()/recv() */
        }
        t_down = 0;
    }
    close(fd);
    return NULL;
}

/* ---- decode queue -------------------------------------------------------- */
#define QCAP 16
typedef struct { uint8_t *buf; uint32_t len; uint64_t pts, t_recv; uint8_t codec, key; } au_t;
static au_t g_q[QCAP];
static int g_qh, g_qt, g_qn;
static pthread_mutex_t g_q_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_q_cv = PTHREAD_COND_INITIALIZER;
static uint32_t g_dropped;
static int g_need_key = 1;

static void q_clear_locked(void) {
    while (g_qn) { free(g_q[g_qh].buf); g_qh = (g_qh + 1) % QCAP; g_qn--; g_dropped++; }
}

static void q_push(au_t a) {
    pthread_mutex_lock(&g_q_mu);
    if (g_need_key && !a.key) { g_dropped++; free(a.buf); pthread_mutex_unlock(&g_q_mu); return; }
    g_need_key = 0;
    /* a keyframe makes everything queued before it irrelevant: if we are
     * lagging, skip ahead instead of letting latency accumulate            */
    if (a.key && g_qn > 1) q_clear_locked();
    if (g_qn == QCAP) {               /* hopelessly behind: resync on IDR  */
        q_clear_locked();
        g_need_key = 1;
        free(a.buf); g_dropped++;
        pthread_mutex_unlock(&g_q_mu);
        vlog("decode queue overflow → waiting for next keyframe");
        return;
    }
    g_q[g_qt] = a; g_qt = (g_qt + 1) % QCAP; g_qn++;
    pthread_cond_signal(&g_q_cv);
    pthread_mutex_unlock(&g_q_mu);
}

static fbout_t g_fb;
static ep_stream_t g_stream;           /* last EP_STREAM from the PC (0 = unknown) */

/* ---- hardware decode: frames arrive on a decoder callback thread -------- */
#define TRING 64
static struct { uint64_t pts, t_recv, t_sub; } g_tring[TRING];
static unsigned g_tring_i;

static void hw_frame(void *user, const yuv_img_t *img, uint64_t pts) {
    (void)user;
    uint64_t t0 = now_us(), t_recv = 0, t_sub = 0;
    for (unsigned i = 0; i < TRING; i++)
        if (g_tring[i].pts == pts) { t_recv = g_tring[i].t_recv; t_sub = g_tring[i].t_sub; break; }
    if (img) {                       /* converted path; NULL = already on VIDEO0 */
        yuv_to_fb(&g_fb, img, opt_cvt_threads);
        fbout_publish(&g_fb);
    }
    uint64_t t1 = now_us();
    ep_frame_t f = { pts, t_recv, t1, t_sub ? (uint32_t)(t0 - t_sub) : 0, (uint32_t)(t1 - t0),
                     (uint32_t)g_qn, g_dropped };
    tx(EP_FRAME, &f, sizeof(f));
}

static void *decode_thread(void *arg) {
    (void)arg;
    swdec_t *dec[2] = { NULL, NULL };
    int failed[2] = { 0, 0 };
    hwdec_t *hw = NULL;
    ep_stream_t hw_cfg = { 0, 0, 0, 0, 0 };
    int hw_failed = 0;
    while (!g_exit) {
        pthread_mutex_lock(&g_q_mu);
        while (!g_qn && !g_exit) pthread_cond_wait(&g_q_cv, &g_q_mu);
        if (g_exit) { pthread_mutex_unlock(&g_q_mu); break; }
        au_t a = g_q[g_qh]; g_qh = (g_qh + 1) % QCAP; g_qn--;
        uint32_t depth = (uint32_t)g_qn;
        pthread_mutex_unlock(&g_q_mu);

        int c = a.codec == EP_CODEC_HEVC ? 1 : 0;
        if (opt_null_dec) {           /* plumbing test: "show" every AU */
            ep_frame_t f = { a.pts, a.t_recv, now_us(), 0, 0, depth, g_dropped };
            tx(EP_FRAME, &f, sizeof(f));
            free(a.buf);
            continue;
        }
        if (opt_hw && !hw_failed) {
            pthread_mutex_lock(&g_q_mu);
            ep_stream_t st = g_stream;
            pthread_mutex_unlock(&g_q_mu);
            if (!st.width) { st.width = (uint16_t)opt_fb_w; st.height = (uint16_t)opt_fb_h; }
            st.codec = a.codec;
            if (hw && (st.width != hw_cfg.width || st.height != hw_cfg.height || st.codec != hw_cfg.codec)) {
                vlog("hwdec: stream changed → reopening");
                hwdec_close(hw); hw = NULL;
            }
            if (!hw) {
                if (!a.key) { free(a.buf); continue; }      /* (re)start on an IDR */
                hw = hwdec_open(st.codec, st.width, st.height, opt_hw_frames, opt_hw_sync,
                                &opt_hw_tune, hw_frame, NULL);
                hw_cfg = st;
                if (!hw) { hw_failed = 1; vlog("hwdec unavailable → software decode"); }
            }
            if (hw) {
                unsigned i = g_tring_i++ % TRING;
                g_tring[i].pts = a.pts; g_tring[i].t_recv = a.t_recv; g_tring[i].t_sub = now_us();
                if (hwdec_submit(hw, a.buf, (int)a.len, a.pts) < 0) {
                    pthread_mutex_lock(&g_q_mu); g_need_key = 1; pthread_mutex_unlock(&g_q_mu);
                }
                free(a.buf);
                continue;
            }
        }
        if (!dec[c] && !failed[c]) {
            dec[c] = swdec_open(a.codec, opt_dec_threads, opt_fast);
            if (!dec[c]) failed[c] = 1;
        }
        if (dec[c]) {
            uint32_t dus = 0, cus = 0;
            int r = swdec_decode(dec[c], a.buf, (int)a.len, &g_fb, opt_cvt_threads, &dus, &cus);
            if (r > 0) {
                ep_frame_t f = { a.pts, a.t_recv, now_us(), dus, cus, depth, g_dropped };
                tx(EP_FRAME, &f, sizeof(f));
            } else if (r < 0) {
                pthread_mutex_lock(&g_q_mu);
                g_need_key = 1;          /* corrupt reference chain: resync */
                pthread_mutex_unlock(&g_q_mu);
            }
        }
        free(a.buf);
    }
    return NULL;
}

/* ---- connection handling -------------------------------------------------- */
/* ---- on-screen status (fb_shared via the natartos overlay) ---------------- */
static int g_showing_video;

static void show_waiting(void) {
    if (!opt_video) return;
    static const char *L1 = "EYEPIECE - WAITING FOR THE PC";
    static const char *L2 = "ON THE PC: EYEPIECE-PC  OR  TOOLYA WEB > EYEPIECE STREAM";
    const char *L3 = opt_back ? "PRESS BACK TO EXIT" : "";
    fbout_fill(&g_fb, 0xff000000u);
    for (int i = 0; i < 2; i++) {
        uint8_t *slot = g_fb.map + 64 + (size_t)i * g_fb.slot;
        int cy = g_fb.h / 2;
        /* big enough to read through the lenses: 40x56 px title, 25x35 px hints */
        fbtext_draw(slot, g_fb.w, g_fb.h, (g_fb.w - fbtext_width(L1, 8)) / 2, cy - 140, 8, L1, 0xffffffffu);
        fbtext_draw(slot, g_fb.w, g_fb.h, (g_fb.w - fbtext_width(L2, 5)) / 2, cy + 10, 5, L2, 0xffc8c8c8u);
        fbtext_draw(slot, g_fb.w, g_fb.h, (g_fb.w - fbtext_width(L3, 5)) / 2, cy + 90, 5, L3, 0xffc8c8c8u);
    }
    fbout_publish(&g_fb);
    g_showing_video = 0;
}

/* first frame of a session: direct scan-out needs the GUI layer transparent;
 * the fb_shared paths overwrite the message with video anyway             */
static void *clear_thread(void *arg) {
    (void)arg;
    fbout_fill(&g_fb, 0x00000000u);     /* 16 MB: not on the receive thread */
    fbout_publish(&g_fb);
    return NULL;
}
static void show_video(void) {
    if (g_showing_video) return;
    g_showing_video = 1;
    if (opt_hw_tune.direct_vops && opt_gui_transparent) {
        pthread_t t;
        if (pthread_create(&t, NULL, clear_thread, NULL) == 0) pthread_detach(t);
        else clear_thread(NULL);
    }
}

static void serve_client(int fd) {
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    pthread_mutex_lock(&g_tx_mu);
    g_cfd = fd;
    pthread_mutex_unlock(&g_tx_mu);
    pthread_mutex_lock(&g_q_mu);
    g_need_key = 1;
    pthread_mutex_unlock(&g_q_mu);

    ep_info_t info = { EP_VERSION, (uint16_t)opt_fb_w, (uint16_t)opt_fb_h,
                       (uint8_t)(opt_video ? (opt_hw ? 2 : 1) : 0), g_ht_flags, 0 };
    tx(EP_INFO, &info, sizeof(info));
    vlog("client connected");

    uint64_t frames = 0, bytes = 0, t_stat = now_us();
    while (!g_exit) {
        ep_hdr_t h;
        if (read_all(fd, &h, sizeof(h)) < 0) break;
        if (h.magic0 != 'E' || h.magic1 != 'P' || h.len > EP_MAX_PAYLOAD) {
            vlog("protocol error (type 0x%02x len %u) → dropping client", h.type, h.len);
            break;
        }
        uint8_t *p = malloc(h.len ? h.len : 1);
        if (!p) break;
        if (h.len && read_all(fd, p, h.len) < 0) { free(p); break; }
        switch (h.type) {
        case EP_VIDEO: {
            if (h.len <= sizeof(ep_video_t) || !opt_video) { free(p); break; }
            ep_video_t v;
            memcpy(&v, p, sizeof(v));
            uint32_t n = h.len - (uint32_t)sizeof(v);
            memmove(p, p + sizeof(v), n);
            au_t a = { p, n, v.pts_us, now_us(), v.codec, (uint8_t)(h.flags & EP_FLAG_KEY) };
            show_video();
            q_push(a);
            frames++; bytes += n;
            p = NULL;
            break;
        }
        case EP_STREAM:
            if (h.len >= sizeof(ep_stream_t)) {
                pthread_mutex_lock(&g_q_mu);
                memcpy(&g_stream, p, sizeof(g_stream));
                pthread_mutex_unlock(&g_q_mu);
                vlog("stream: %ux%u %s @%u fps", g_stream.width, g_stream.height,
                     g_stream.codec == EP_CODEC_HEVC ? "HEVC" : "H264", g_stream.fps);
            }
            break;
        case EP_PING: {
            uint64_t pong[2] = { 0, now_us() };
            if (h.len >= 8) memcpy(&pong[0], p, 8);
            tx(EP_PONG, pong, sizeof(pong));
            break;
        }
        case EP_CTRL:
            if (h.len >= 1 && p[0] == EP_OP_RECENTER) {
                pthread_mutex_lock(&g_pose_mu); g_recenter = 1; pthread_mutex_unlock(&g_pose_mu);
                vlog("recenter requested");
            } else if (h.len >= 1 && p[0] == EP_OP_QUIT) {
                vlog("quit requested by PC");
                g_exit = 1;
            }
            break;
        default:
            break;   /* forward compatible: ignore unknown types */
        }
        free(p);
        uint64_t t = now_us();
        if (t - t_stat > 5000000) {
            vlog("rx %.1f fps, %.2f Mbit/s, dropped %u, pose samples %u",
                 frames * 1e6 / (double)(t - t_stat), bytes * 8.0 / (double)(t - t_stat),
                 g_dropped, g_pose_count);
            frames = bytes = 0; t_stat = t;
        }
    }
    pthread_mutex_lock(&g_tx_mu);
    g_cfd = -1;
    pthread_mutex_unlock(&g_tx_mu);
    close(fd);
    pthread_mutex_lock(&g_q_mu);
    q_clear_locked();
    pthread_mutex_unlock(&g_q_mu);
    vlog("client disconnected");
    if (!opt_once) show_waiting();       /* menu-launched bridge keeps running */
}

static void on_signal(int sig) { (void)sig; g_exit = 1; }

/* On a crash, hand the display back: without fb_override the overlay stops
 * injecting and the stock UI repaints (otherwise the last frame would stay
 * frozen until a long-hold Back).  Only async-signal-safe calls here.      */
static char g_flag_path[256];
static void on_crash(int sig) {
    if (g_flag_path[0]) unlink(g_flag_path);
    dispout_qsync_off();          /* best effort: don't leave the panel in VRR mode */
    signal(sig, SIG_DFL);
    raise(sig);
}

static void usage(const char *a0) {
    fprintf(stderr,
        "usage: %s [options]\n"
        "  -p PORT     TCP port on 127.0.0.1 (default %d; PC: adb forward tcp:%d tcp:%d)\n"
        "  -A          listen on all interfaces (e.g. RNDIS/Wi-Fi) instead of loopback\n"
        "  -s WxH      output framebuffer size (default 1920x1080)\n"
        "  -t N        decoder slice threads (default 4)\n"
        "  -c N        colour-convert threads (default 4)\n"
        "  -f          fast decode (skip loop filter)\n"
        "  -V          no video (head tracking only)\n"
        "  -H          no head tracking\n"
        "  -Z          only send the MCU head-attitude stop, then exit (recovery)\n"
        "  -b          exit on Back key (menu launches)\n"
        "  -1          exit after the first client disconnects\n"
        "  -D sw|hw    decoder: hw = VC8000D via /dev/vdec0 (default), sw = device libavcodec\n"
        "              (-D sw / -D hw both imply -O fb unless -O plane is given after them)\n"
        "  -F N        hw: number of frame buffers (default 12)\n"
        "  -S DIR      hw: duss_hal_mem_sync(frame, DIR) before reading frames (default: none)\n"
        "  -O plane[:M] zero-copy scan-out on the VIDEO0 plane of VOP mask M (1 = vop0,\n"
        "              3 = both eyes; default: plane:3)\n"
        "  -O fb       convert into fb_shared instead (overlay path; slow with -D hw: uncached)\n"
        "  -M          log display timing (push→latch, refresh interval) every 3 s\n"
        "  -Q VFP      EXPERIMENTAL: display qsync (variable refresh), max front porch VFP lines\n"
        "  -T          with -O plane: keep the GUI visible over the video (default: GUI made\n"
        "              transparent through the overlay)\n"
        "  -L R,F,V    hw: start-config knobs: no_reorder (default 1), +0x24, +0x2c (default 0)\n"
        "  -N          null decoder (protocol/latency plumbing test, nothing displayed)\n"
        "  -l FILE     log file (default /data/local/tmp/eyepiece.log, '-' = none)\n",
        a0, EP_PORT, EP_PORT, EP_PORT);
}

int main(int argc, char **argv) {
    int o;
    while ((o = getopt(argc, argv, "p:As:t:c:fVHb1ND:F:S:L:O:TMQ:l:Zh")) != -1) {
        switch (o) {
        case 'p': opt_port = atoi(optarg); break;
        case 'A': opt_any_addr = 1; break;
        case 's': if (sscanf(optarg, "%dx%d", &opt_fb_w, &opt_fb_h) != 2) { usage(argv[0]); return 2; } break;
        case 't': opt_dec_threads = atoi(optarg); break;
        case 'c': opt_cvt_threads = atoi(optarg); break;
        case 'f': opt_fast = 1; break;
        case 'V': opt_video = 0; break;
        case 'H': opt_headtrack = 0; break;
        case 'b': opt_back = 1; break;
        case '1': opt_once = 1; break;
        case 'N': opt_null_dec = 1; break;
        case 'D': opt_hw = !strcmp(optarg, "hw"); opt_direct = 0; break;
        case 'F': opt_hw_frames = atoi(optarg); break;
        case 'O':
            if (!strncmp(optarg, "plane", 5)) {
                opt_hw = 1;
                opt_direct = optarg[5] == ':' ? atoi(optarg + 6) : 1;
            } else if (!strcmp(optarg, "fb")) {
                opt_direct = 0;
            }
            break;
        case 'T': opt_gui_transparent = 0; break;
        case 'M': setenv("EYEPIECE_METRICS", "1", 1); break;   /* display timing metrics */
        case 'Q': setenv("EYEPIECE_QSYNC", optarg, 1); break;  /* experimental VRR, vfp lines */
        case 'S': opt_hw_sync = atoi(optarg); break;
        case 'L': sscanf(optarg, "%d,%d,%d", &opt_hw_tune.no_reorder, &opt_hw_tune.flag24,
                         &opt_hw_tune.val2c); break;
        case 'l': opt_log = optarg; break;
        case 'Z': opt_mcu_stop = 1; break;   /* only send the MCU attitude stop */
        default: usage(argv[0]); return 2;
        }
    }
    if (opt_mcu_stop) {
        /* stopper / manual recovery: stderr only — never touch the running
         * bridge's log file, fb_shared or display                          */
        int rc = headtrack_stop_cli();
        fflush(stderr);
        _exit(rc);   /* skip DJI framework atexit/destructors */
    }
    if (opt_null_dec) opt_hw = 0;
    opt_hw_tune.direct_vops = opt_hw ? opt_direct : 0;
    if (!opt_log) opt_log = tmp_path("eyepiece.log");
    if (strcmp(opt_log, "-")) g_log = fopen(opt_log, "w");

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = on_signal;          /* no SA_RESTART: unblock accept/recv */
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    /* respect nohup: started from `adb shell nohup …`, SIGHUP must stay
     * ignored or the bridge dies when the adb shell closes               */
    struct sigaction cur;
    if (sigaction(SIGHUP, NULL, &cur) == 0 && cur.sa_handler != SIG_IGN)
        sigaction(SIGHUP, &sa, NULL);
    signal(SIGPIPE, SIG_IGN);
    snprintf(g_flag_path, sizeof(g_flag_path), "%s", tmp_path("fb_override"));
    signal(SIGSEGV, on_crash);
    signal(SIGBUS, on_crash);
    signal(SIGABRT, on_crash);
    signal(SIGILL, on_crash);

    vlog("eyepiece_bridge v%d starting (port %d, fb %dx%d)", EP_VERSION, opt_port, opt_fb_w, opt_fb_h);

    if (opt_video && fbout_open(&g_fb, opt_fb_w, opt_fb_h) < 0) {
        vlog("framebuffer unavailable → head tracking only");
        opt_video = 0;
    }
    /* until a PC streams: a black screen with instructions (the GUI is hidden
     * by fb_override anyway — the message says why). The first video AU makes
     * the GUI layer transparent for the VIDEO0 plane below (show_video).    */
    show_waiting();
    /* fork the bus helper before any thread exists */
    if (opt_headtrack && headtrack_spawn(on_pose) < 0)
        vlog("bus head tracking unavailable");

    pthread_t th;
    pthread_create(&th, NULL, pose_thread, NULL); pthread_detach(th);
    if (opt_video) { pthread_create(&th, NULL, decode_thread, NULL); pthread_detach(th); }
    if (opt_back)  { pthread_create(&th, NULL, back_thread, NULL); pthread_detach(th); }

    int ls = g_ls = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons((uint16_t)opt_port) };
    addr.sin_addr.s_addr = htonl(opt_any_addr ? INADDR_ANY : INADDR_LOOPBACK);
    if (bind(ls, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(ls, 1) < 0) {
        vlog("bind/listen :%d failed: %s", opt_port, strerror(errno));
        g_exit = 1;
    } else {
        vlog("listening on %s:%d", opt_any_addr ? "0.0.0.0" : "127.0.0.1", opt_port);
    }

    while (!g_exit) {
        int fd = accept(ls, NULL, NULL);
        if (fd < 0) {
            if (errno == EINTR) continue;
            if (!g_exit) vlog("accept: %s", strerror(errno));
            break;
        }
        serve_client(fd);
        if (opt_once) break;
    }

    vlog("shutting down");
    g_exit = 1;
    pthread_cond_broadcast(&g_q_cv);
    if (ls >= 0) close(ls);
    if (opt_headtrack) headtrack_stop();
    dispout_qsync_off();                 /* panel back to fixed refresh */
    if (opt_video) fbout_close(&g_fb);   /* removes fb_override → overlay restores the UI */
    if (g_log) fclose(g_log);
    _exit(0);   /* don't run DJI framework atexit/destructors */
}
