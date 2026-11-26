// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * eyepiece_bridge — goggles side of Eyepiece (DJI Goggles 3 as a USB PC display
 * with head tracking).
 *
 *   PC ──adb forward tcp:42420──► eyepiece_bridge ──► decode ──► VIDEO0 plane ──► display
 *   PC ◄── head pose / frame stats ── eyepiece_bridge ◄── MCU head attitude (bus, 100 Hz)
 *
 * Runs as root from `adb shell` or from the natartos "Eyepiece" menu entry.
 * Writes only temporary bridge state under /data/local/tmp; never touches
 * partitions, /system or /vendor.  See docs/goggles/bridge.md in eyepiece-pc.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif
#include "bridge.h"
#include "../backdrop.h"
#include "../../overlay/gui_alpha.h"

/* ---- options ------------------------------------------------------------ */
static int  opt_port = EP_PORT;
static int  opt_fb_w = 1920, opt_fb_h = 1080;
static int  opt_cvt_threads = 4;
static int  opt_video = 1, opt_headtrack = 1, opt_back = 0, opt_once = 0, opt_mcu_stop = 0;
static int  opt_pane_reconnect = 0;
static int  opt_any_addr = 0, opt_null_dec = 0;
/* VC8000D + zero-copy scan-out on both eyes (measured on device: 60 fps, decode 2.7 ms). */
static int  opt_hw_frames = 12, opt_hw_sync = -1;
static int  opt_hw_frames_set;
static int  opt_bulk = 0;    /* -B: dedicated FunctionFS bulk interface alongside ADB */
static int  opt_direct = 1;  /* -O VOP mask (VOP0 drives both eyes via DSI2DSI) */
static hwdec_tune_t opt_hw_tune = { .no_reorder = 1 };
static const char *opt_log = NULL;   /* default: tmp_path("eyepiece.log") */
static int g_restore_arhome;

#if defined(__ANDROID__)
static int wait_arhome_state(const char *expected) {
    char state[PROP_VALUE_MAX];
    for (int i = 0; i < 50; i++) {
        if (__system_property_get("init.svc.dji_arhome", state) > 0 &&
            !strcmp(state, expected))
            return 0;
        usleep(100000);
    }
    return -1;
}

static int stop_arhome_for_gfx1(void) {
    char enabled[PROP_VALUE_MAX], state[PROP_VALUE_MAX];
    if (__system_property_get("dji.arhome_service", enabled) <= 0 ||
        __system_property_get("init.svc.dji_arhome", state) <= 0) {
        vlog("cannot read dji_arhome service state; refusing GFX1 output");
        return -1;
    }
    if (!strcmp(enabled, "0")) {
        if (strcmp(state, "stopped") && wait_arhome_state("stopped") < 0) {
            vlog("dji_arhome did not stop after being disabled; refusing GFX1 output");
            return -1;
        }
        vlog("dji_arhome already disabled; leaving its state unchanged");
        return 0;
    }
    if (strcmp(enabled, "1")) {
        vlog("unexpected dji.arhome_service value '%s'; refusing GFX1 output", enabled);
        return -1;
    }
    if (!strcmp(state, "stopped")) {
        vlog("dji_arhome is not running; leaving its state unchanged");
        return 0;
    }
    if (strcmp(state, "running")) {
        vlog("dji_arhome state is '%s'; refusing GFX1 output", state);
        return -1;
    }
    if (__system_property_set("dji.arhome_service", "0") != 0) {
        vlog("failed to stop dji_arhome service");
        return -1;
    }
    g_restore_arhome = 1;
    if (wait_arhome_state("stopped") < 0) {
        vlog("dji_arhome did not stop; restoring its enabled state");
        return -1;
    }
    vlog("stopped dji_arhome for GFX1; will restore it on bridge exit");
    return 0;
}

static void restore_arhome(void) {
    if (!g_restore_arhome) return;
    if (__system_property_set("dji.arhome_service", "1") != 0) {
        vlog("failed to restore dji_arhome; run: setprop dji.arhome_service 1");
        return;
    }
    g_restore_arhome = 0;
    if (wait_arhome_state("running") < 0)
        vlog("dji_arhome restore requested but service did not reach running state");
    else
        vlog("restored dji_arhome service");
}
#else
static int stop_arhome_for_gfx1(void) { return 0; }
static void restore_arhome(void) {}
#endif

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
static int g_cfd_bulk;
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

static int write_bulk_all(int fd, const void *p, size_t n) {
    const uint8_t *b = p;
    uint64_t deadline = now_us() + 500000;
    while (n) {
        if (g_exit) { errno = ECANCELED; return -1; }
        ssize_t written = send(fd, b, n, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (written > 0) {
            b += written;
            n -= (size_t)written;
            continue;
        }
        if (written < 0 && errno == EINTR) continue;
        if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return -1;

        uint64_t current_time = now_us();
        if (current_time >= deadline) { errno = ETIMEDOUT; return -1; }
        struct pollfd pfd = { .fd = fd, .events = POLLOUT };
        int timeout_ms = (int)((deadline - current_time + 999) / 1000);
        int ready;
        do { ready = poll(&pfd, 1, timeout_ms); }
        while (ready < 0 && errno == EINTR && !g_exit);
        if (ready == 0) { errno = ETIMEDOUT; return -1; }
        if (ready < 0) return -1;
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) { errno = EPIPE; return -1; }
    }
    return 0;
}

static int write_client_all(int fd, const void *p, size_t n) {
    return g_cfd_bulk ? write_bulk_all(fd, p, n) : write_all(fd, p, n);
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

static int read_all_bulk(int fd, void *p, size_t n) {
    size_t requested = n;
    uint8_t *b = p;
    uint64_t deadline = now_us() + 5000000;
    while (n) {
        uint64_t now = now_us();
        if (now >= deadline) {
            vlog("bulk: protocol read timed out (requested=%zu remaining=%zu)", requested, n);
            bulk_log_diagnostics();
            return -1;
        }
        int timeout_ms = (int)((deadline - now + 999) / 1000);
        struct pollfd pfd = { .fd = fd, .events = POLLIN };
        int ready;
        do { ready = poll(&pfd, 1, timeout_ms); } while (ready < 0 && errno == EINTR && !g_exit);
        if (ready <= 0) {
            vlog("bulk: protocol read %s (requested=%zu remaining=%zu)",
                 ready == 0 ? "timed out" : strerror(errno), requested, n);
            bulk_log_diagnostics();
            return -1;
        }
        if (!(pfd.revents & POLLIN)) {
            vlog("bulk: protocol socket closed (revents=0x%x requested=%zu remaining=%zu)",
                 pfd.revents, requested, n);
            bulk_log_diagnostics();
            return -1;
        }
        ssize_t r = recv(fd, b, n, 0);
        if (r < 0 && errno == EINTR && !g_exit) continue;
        if (r <= 0) {
            vlog("bulk: protocol socket %s (requested=%zu remaining=%zu)",
                 r == 0 ? "EOF" : strerror(errno), requested, n);
            bulk_log_diagnostics();
            return -1;
        }
        b += r;
        n -= (size_t)r;
    }
    return 0;
}

static void tx(const eyepiece_rpc_v1_Envelope *message) {
    pthread_mutex_lock(&g_tx_mu);
    if (g_cfd >= 0) {
        uint8_t packet[1100];
        size_t packet_len;
        if (ep_wire_encode(message, packet, sizeof(packet), &packet_len) < 0) {
            fprintf(stderr, "[eyepiece] cannot encode bridge protobuf type %u\n", message->which_message);
            shutdown(g_cfd, SHUT_RDWR);
        } else {
            uint32_t length = htonl((uint32_t)packet_len);
            if (write_client_all(g_cfd, &length, sizeof(length)) < 0 ||
                write_client_all(g_cfd, packet, packet_len) < 0) {
                shutdown(g_cfd, SHUT_RDWR);   /* reader notices and cleans up */
            }
        }
    }
    pthread_mutex_unlock(&g_tx_mu);
}

static __thread int t_in_log;
static void send_log_to_client(const char *s) {
    if (t_in_log) return;
    t_in_log = 1;
    ep_bytes_t line = { (const uint8_t *)s, strlen(s) };
    eyepiece_rpc_v1_Envelope message = eyepiece_rpc_v1_Envelope_init_zero;
    message.which_message = eyepiece_rpc_v1_Envelope_log_tag;
    ep_wire_set_bytes(&message.message.log.line, &line);
    tx(&message);
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
    eyepiece_rpc_v1_Envelope message = eyepiece_rpc_v1_Envelope_init_zero;
    message.which_message = eyepiece_rpc_v1_Envelope_pose_tag;
    message.message.pose = (eyepiece_rpc_v1_Pose){
        .device_us = now_us(), .yaw = y, .pitch = pitch, .roll = roll,
        .active = true, .source = eyepiece_rpc_v1_PoseSource_POSE_SOURCE_MCU
    };
    tx(&message);
}

static void *pose_thread(void *arg) {
    (void)arg;
    while (!g_exit) usleep(4000);
    /* signals may land on any thread: make sure accept()/recv() wake up   */
    if (g_ls >= 0) shutdown(g_ls, SHUT_RDWR);
    pthread_mutex_lock(&g_tx_mu);
    if (g_cfd >= 0) shutdown(g_cfd, SHUT_RDWR);
    pthread_mutex_unlock(&g_tx_mu);
    return NULL;
}

/* ---- Back key (optional, for menu launches) ------------------------------ */
/* Open the input device whose name is `want` (event numbers depend on probe
 * order and on dji_glasses creating omc_keys as uinput); fallback path if no
 * name matches. */
static int open_input(const char *want, const char *fallback) {
    char path[32], name[64];
    for (int n = 0; n < 10; n++) {
        snprintf(path, sizeof(path), "/dev/input/event%d", n);
        int fd = open(path, O_RDONLY | O_CLOEXEC);
        if (fd < 0) continue;
        if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) > 0) {
            name[sizeof(name) - 1] = 0;
            if (strcmp(name, want) == 0) return fd;
        }
        close(fd);
    }
    return open(fallback, O_RDONLY | O_CLOEXEC);
}

/* -b: grab both key devices so the GUI does not navigate while streaming. A Back click
 * (< 1.5 s) or a long hold both exit; the overlay's own long-hold exit can't
 * see the key while it is grabbed. Nothing counts in the first 2 s: a stray
 * Back event was seen once right after a dji_glasses restart. The grab ends
 * with the process.                                                        */
static void *back_thread(void *arg) {
    (void)arg;
    struct { long sec, usec; uint16_t type, code; int32_t value; } ev;
    struct pollfd pf[2] = {
        { .fd = open_input("gpio_keys", "/dev/input/event0"), .events = POLLIN },
        { .fd = open_input("omc_keys",  "/dev/input/event3"), .events = POLLIN },
    };
    if (pf[0].fd < 0) { vlog("back: cannot open gpio_keys: %s", strerror(errno)); return NULL; }
    for (int i = 0; i < 2; i++)
        if (pf[i].fd >= 0 && ioctl(pf[i].fd, EVIOCGRAB, 1) < 0)
            vlog("back: grab %d failed: %s", i, strerror(errno));
    vlog("back: buttons grabbed (gpio_keys%s)", pf[1].fd >= 0 ? " + omc_keys" : "");
    uint64_t t_start = now_us(), t_down = 0;
    while (!g_exit) {
        int r = poll(pf, pf[1].fd >= 0 ? 2 : 1, 100);
        if (r < 0 && errno != EINTR) break;
        uint64_t t = now_us();
        if (t_down && t - t_down >= 1500000) {      /* long hold, no release needed */
            vlog("Back held → exiting");
            g_exit = 1;
            break;
        }
        if (r <= 0) continue;
        if (pf[1].fd >= 0 && (pf[1].revents & POLLIN))
            (void)!read(pf[1].fd, &ev, sizeof(ev));   /* 5D: swallowed */
        if (!(pf[0].revents & POLLIN)) continue;
        if (read(pf[0].fd, &ev, sizeof(ev)) != (ssize_t)sizeof(ev)) break;
        if (ev.type != EV_KEY || ev.code != KEY_BACK) continue;
        vlog("back: key %s at +%.2f s", ev.value ? "down" : "up", (t - t_start) / 1e6);
        if (t - t_start < 2000000) { t_down = 0; continue; }
        if (ev.value == 1) { t_down = t; continue; }
        if (ev.value == 0 && t_down) {
            vlog("Back clicked → exiting");
            g_exit = 1;               /* pose_thread wakes accept()/recv() */
        }
        t_down = 0;
    }
    for (int i = 0; i < 2; i++) if (pf[i].fd >= 0) close(pf[i].fd);
    return NULL;
}

/* ---- decode queue -------------------------------------------------------- */
#define QCAP 16
typedef struct { uint8_t *buf; const uint8_t *data; uint32_t len; uint64_t pts, t_recv; uint8_t codec, key; } au_t;
static au_t g_q[QCAP];
static int g_qh, g_qt, g_qn;
static pthread_mutex_t g_q_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_q_cv = PTHREAD_COND_INITIALIZER;
static uint32_t g_dropped;
static int g_need_key = 1;

static void q_clear_locked(void) {
    while (g_qn) { free(g_q[g_qh].buf); g_qh = (g_qh + 1) % QCAP; g_qn--; g_dropped++; }
}

static void q_resync_locked(void) {
    q_clear_locked();
    g_need_key = 1;
}

static void q_push(au_t a) {
    pthread_mutex_lock(&g_q_mu);
    if (g_need_key && !a.key) { g_dropped++; free(a.buf); pthread_mutex_unlock(&g_q_mu); return; }
    g_need_key = 0;
    /* a keyframe makes everything queued before it irrelevant: if we are
     * lagging, skip ahead instead of letting latency accumulate            */
    if (a.key && g_qn > 1) q_clear_locked();
    if (g_qn == QCAP) {               /* hopelessly behind: resync on IDR  */
        q_resync_locked();
        free(a.buf); g_dropped++;
        pthread_mutex_unlock(&g_q_mu);
        vlog("decode queue overflow → waiting for next keyframe");
        return;
    }
    g_q[g_qt] = a; g_qt = (g_qt + 1) % QCAP; g_qn++;
    pthread_cond_signal(&g_q_cv);
    pthread_mutex_unlock(&g_q_mu);
}

/* The one stream description: the last Configure the decoder thread applied. It outlives
 * connections, so a retained GFX1 session keeps it while no client is connected. */
static eyepiece_rpc_v1_Configure g_config;
static int g_configured;                   /* g_config is valid */
static int g_pane_active;                  /* g_config selects the GFX1 overlay */
static int g_pane_has_frame;
static uint8_t g_pane_alpha = 255, g_pane_alpha_request;
static int g_pane_alpha_pending, g_pane_alpha_result;
static eyepiece_rpc_v1_Configure g_config_request;
static int g_config_pending, g_config_result;
static int g_disconnect_drain_pending, g_disconnect_drain_result;

/* ---- hardware decode: frames arrive on a decoder callback thread -------- */
#define TRING 64
static struct { uint64_t pts, t_recv, t_sub; } g_tring[TRING];
static unsigned g_tring_i;

static void hw_output_error(void *user) {
    (void)user;
    g_exit = 1;
    pthread_mutex_lock(&g_tx_mu);
    if (g_cfd >= 0) shutdown(g_cfd, SHUT_RDWR);
    pthread_mutex_unlock(&g_tx_mu);
}

static pthread_mutex_t g_backdrop_mu = PTHREAD_MUTEX_INITIALIZER;
static int g_backdrop_fd = -1;
static int g_backdrop_failed;
static int g_gui_alpha_refresh_pending;
static int gui_alpha_refresh(void);

static int backdrop_prepare_output(void *user) {
    (void)user;
    if (__atomic_exchange_n(&g_gui_alpha_refresh_pending, 0, __ATOMIC_ACQ_REL) &&
        gui_alpha_refresh() < 0) {
        vlog("gui alpha: failed to restore blending after display setup");
        hw_output_error(NULL);
        return -1;
    }
    pthread_mutex_lock(&g_backdrop_mu);
    if (g_backdrop_fd >= 0) { pthread_mutex_unlock(&g_backdrop_mu); return 0; }
    if (g_backdrop_failed || g_exit) { pthread_mutex_unlock(&g_backdrop_mu); return -1; }

    char path[256];
    const char *dir = getenv("EYEPIECE_TMP");
    int len = snprintf(path, sizeof(path), "%s/%s", dir && *dir ? dir : "/data/local/tmp",
                       "eyepiece.backdrop");
    if (len < 0 || (size_t)len >= sizeof(path)) {
        vlog("backdrop: request path too long");
        goto failed;
    }
    int fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    if (fd < 0) { vlog("backdrop: open failed: %s", strerror(errno)); goto failed; }
    if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
        vlog("backdrop: request lock failed: %s", strerror(errno));
        close(fd);
        goto failed;
    }
    eyepiece_backdrop_request request = {
        .request = ((uint64_t)(unsigned)getpid() << 32) ^ now_us(),
        .acknowledged = 0
    };
    if (!request.request) request.request = 1;
    if (pwrite(fd, &request, sizeof(request), 0) != sizeof(request)) {
        vlog("backdrop: request write failed: %s", strerror(errno));
        close(fd);
        goto failed;
    }
    uint64_t t_request = now_us();
    for (int i = 0; i < 1000 && !g_exit; i++) {
        uint64_t acknowledged = 0;
        ssize_t n = pread(fd, &acknowledged, sizeof(acknowledged),
                          offsetof(eyepiece_backdrop_request, acknowledged));
        if (n != sizeof(acknowledged)) {
            vlog("backdrop: acknowledgement read failed");
            close(fd);
            goto failed;
        }
        if (acknowledged == request.request) {
            g_backdrop_fd = fd;
            pthread_mutex_unlock(&g_backdrop_mu);
            vlog("backdrop: suppression acknowledged in %llu us",
                 (unsigned long long)(now_us() - t_request));
            return 0;
        }
        usleep(2000);
    }
    if (!g_exit) vlog("backdrop: GUI hook did not acknowledge; refusing video");
    close(fd);
failed:
    g_backdrop_failed = 1;
    pthread_mutex_unlock(&g_backdrop_mu);
    hw_output_error(NULL);
    return -1;
}

static void hw_frame(void *user, const yuv_img_t *img, uint64_t pts, uint32_t converted_us) {
    (void)user;
    uint64_t t0 = now_us(), t_recv = 0, t_sub = 0;
    pthread_mutex_lock(&g_q_mu);
    for (unsigned i = 0; i < TRING; i++)
        if (g_tring[i].pts == pts) { t_recv = g_tring[i].t_recv; t_sub = g_tring[i].t_sub; break; }
    uint32_t depth = (uint32_t)g_qn, dropped = g_dropped;
    if (g_pane_active && !img) g_pane_has_frame = 1;
    pthread_mutex_unlock(&g_q_mu);
    uint64_t t1 = now_us();
    uint64_t elapsed = t_sub && t0 >= t_sub ? t0 - t_sub : 0;
    eyepiece_rpc_v1_Envelope message = eyepiece_rpc_v1_Envelope_init_zero;
    message.which_message = eyepiece_rpc_v1_Envelope_frame_tag;
    message.message.frame = (eyepiece_rpc_v1_Frame){
        pts, t_recv, t1, (uint32_t)(elapsed > converted_us ? elapsed - converted_us : 0),
        converted_us + (uint32_t)(t1 - t0), depth, dropped
    };
    tx(&message);
}

static int hw_skip_output(void *user) {
    (void)user;
    pthread_mutex_lock(&g_q_mu);
    int skip = g_qn > 2;
    pthread_mutex_unlock(&g_q_mu);
    return skip;
}

static hwdec_t *open_decoder(const eyepiece_rpc_v1_Configure *config) {
    uint32_t width, height;
    if (ep_config_frame_size(config, (uint16_t)opt_fb_w, (uint16_t)opt_fb_h, &width, &height)) return NULL;
    __atomic_store_n(&g_gui_alpha_refresh_pending, 1, __ATOMIC_RELEASE);
    if (!config->has_pane) {
        opt_hw_tune.stereo = config->layout == eyepiece_rpc_v1_StreamLayout_LAYOUT_SBS &&
                             opt_hw_tune.direct_vops ? 1901 : 0;
        return hwdec_open(config->codec, (int)width, (int)height, opt_hw_frames, opt_hw_sync,
                          &opt_hw_tune, hw_frame, NULL);
    }
    const eyepiece_rpc_v1_Pane *pane = &config->pane;
    hwdec_tune_t tune = opt_hw_tune;
    tune.stereo = 0;
    tune.pane_output = 1;
    tune.pane_opaque = pane->format == eyepiece_rpc_v1_PaneFormat_PANE_OPAQUE_BGRA;
    tune.pane_alpha = g_pane_alpha;
    tune.pane_x = pane->x; tune.pane_y = pane->y;
    tune.pane_w = pane->width; tune.pane_h = pane->height;
    tune.cvt_threads = opt_cvt_threads;
    tune.prepare_output = backdrop_prepare_output;
    tune.skip_output = hw_skip_output;
    tune.output_error = hw_output_error;
    int hw_frames = opt_hw_frames;
    if (!opt_hw_frames_set && tune.pane_opaque && config->codec == eyepiece_rpc_v1_Codec_CODEC_H264 &&
        width * height <= 1280u * 720u)
        hw_frames = 20;
    return hwdec_open(config->codec, (int)width, (int)height, hw_frames, opt_hw_sync,
                      &tune, hw_frame, NULL);
}

/* Bring the decoder in line with a new Configure. The decoder thread owns the decoder, so
 * this only runs there. A decoder that does not exist yet is simply opened on the first
 * keyframe after the request; one that exists is moved or torn down here.
 *   READY       the request is now the active configuration
 *   UNSUPPORTED the existing decoder cannot be replaced; it is untouched
 *   FAILED      an existing overlay decoder could not be moved or released; its display
 *               resources stay held until exit, so the bridge ends */
static uint32_t apply_configuration(hwdec_t **hw, eyepiece_rpc_v1_Configure *hw_config,
                                    const eyepiece_rpc_v1_Configure *request) {
    if (g_exit) return EP_CONFIGURE_FAILED;
    if (!*hw || ep_config_equal(request, hw_config)) return EP_CONFIGURE_READY;
    if (!hw_config->has_pane) return EP_CONFIGURE_UNSUPPORTED;   /* VIDEO0 decoders are kept until exit */
    if (ep_config_same_shape(request, hw_config)) {
        if (hwdec_move_overlay(*hw, request->pane.x, request->pane.y) != 0) {
            g_exit = 1;
            return EP_CONFIGURE_FAILED;
        }
        hw_config->pane.x = request->pane.x;
        hw_config->pane.y = request->pane.y;
        __atomic_store_n(&g_gui_alpha_refresh_pending, 1, __ATOMIC_RELEASE);
        return EP_CONFIGURE_READY;
    }
    if (hwdec_reconfigure_close(*hw) != 0) {
        g_exit = 1;
        return EP_CONFIGURE_FAILED;
    }
    *hw = NULL;
    pthread_mutex_lock(&g_q_mu);
    g_pane_has_frame = 0;
    pthread_mutex_unlock(&g_q_mu);
    return EP_CONFIGURE_READY;
}

static void close_connection(void) {
    pthread_mutex_lock(&g_tx_mu);
    if (g_cfd >= 0) shutdown(g_cfd, SHUT_RDWR);      /* serve_client cleans up */
    pthread_mutex_unlock(&g_tx_mu);
    pthread_mutex_lock(&g_q_mu);
    q_clear_locked();
    g_need_key = 1;
    pthread_mutex_unlock(&g_q_mu);
}

static void *decode_thread(void *arg) {
    (void)arg;
    hwdec_t *hw = NULL;
    eyepiece_rpc_v1_Configure hw_config = eyepiece_rpc_v1_Configure_init_zero;
    for (;;) {
        pthread_mutex_lock(&g_q_mu);
        while (!g_qn && !g_config_pending && !g_pane_alpha_pending && !g_disconnect_drain_pending && !g_exit)
            pthread_cond_wait(&g_q_cv, &g_q_mu);
        if (g_pane_alpha_pending) {
            uint8_t alpha = g_pane_alpha_request;
            pthread_mutex_unlock(&g_q_mu);
            int failed = g_exit || (hw && hwdec_set_overlay_alpha(hw, alpha) != 0);
            pthread_mutex_lock(&g_q_mu);
            if (!failed) g_pane_alpha = alpha;
            g_pane_alpha_result = failed ? -1 : 0;
            g_pane_alpha_pending = 0;
            pthread_cond_broadcast(&g_q_cv);
            pthread_mutex_unlock(&g_q_mu);
            continue;
        }
        if (g_disconnect_drain_pending) {
            pthread_mutex_unlock(&g_q_mu);
            int failed = !g_exit && hw && hwdec_drain_overlay(hw) != 0;
            pthread_mutex_lock(&g_q_mu);
            if (failed) g_exit = 1;
            g_disconnect_drain_result = failed ? -1 : 0;
            g_disconnect_drain_pending = 0;
            pthread_cond_broadcast(&g_q_cv);
            pthread_mutex_unlock(&g_q_mu);
            continue;
        }
        if (g_config_pending) {
            eyepiece_rpc_v1_Configure request = g_config_request;
            pthread_mutex_unlock(&g_q_mu);
            uint32_t status = apply_configuration(&hw, &hw_config, &request);
            pthread_mutex_lock(&g_q_mu);
            if (status == EP_CONFIGURE_READY) {
                g_config = request;
                g_configured = 1;
                g_pane_active = request.has_pane;
                q_resync_locked();           /* old video is meaningless; wait for a keyframe */
            }
            g_config_result = (int)status;
            g_config_pending = 0;
            pthread_cond_broadcast(&g_q_cv);
            pthread_mutex_unlock(&g_q_mu);
            continue;
        }
        if (g_exit) { pthread_mutex_unlock(&g_q_mu); break; }
        au_t a = g_q[g_qh]; g_qh = (g_qh + 1) % QCAP; g_qn--;
        uint32_t depth = (uint32_t)g_qn;
        uint32_t dropped = g_dropped;
        eyepiece_rpc_v1_Configure config = g_config;
        int configured = g_configured;
        pthread_mutex_unlock(&g_q_mu);

        if (!configured || a.codec != config.codec) {   /* stale video from before a Configure */
            free(a.buf);
            continue;
        }
        if (opt_null_dec) {           /* plumbing test: "show" every AU */
            eyepiece_rpc_v1_Envelope message = eyepiece_rpc_v1_Envelope_init_zero;
            message.which_message = eyepiece_rpc_v1_Envelope_frame_tag;
            message.message.frame = (eyepiece_rpc_v1_Frame){ a.pts, a.t_recv, now_us(), 0, 0, depth, dropped };
            tx(&message);
            free(a.buf);
            continue;
        }
        if (!hw) {
            if (!a.key) { free(a.buf); continue; }      /* (re)start on an IDR */
            hw = open_decoder(&config);
            if (!hw) {
                /* Nothing was acquired. Close this connection; the next Configure retries. */
                vlog("hwdec: cannot open decoder; closing connection");
                free(a.buf);
                close_connection();
                continue;
            }
            hw_config = config;
        }
        pthread_mutex_lock(&g_q_mu);
        unsigned i = g_tring_i++ % TRING;
        g_tring[i].pts = a.pts; g_tring[i].t_recv = a.t_recv; g_tring[i].t_sub = now_us();
        pthread_mutex_unlock(&g_q_mu);
        if (hwdec_submit(hw, a.data, (int)a.len, a.pts) < 0) {
            pthread_mutex_lock(&g_q_mu);
            q_resync_locked();
            pthread_mutex_unlock(&g_q_mu);
            vlog("decoder submit failed → waiting for next keyframe");
        }
        free(a.buf);
    }
    if (hw && g_pane_active) hwdec_stop_overlay(hw);
    return NULL;
}

/* ---- connection handling -------------------------------------------------- */
/* ---- GUI alpha RPC ------------------------------------------------------- */
static int g_gui_alpha_fd = -1;
static uint64_t g_gui_alpha_sequence;
static uint8_t g_gui_alpha_value = 255;
static pthread_mutex_t g_gui_alpha_mu = PTHREAD_MUTEX_INITIALIZER;

static void gui_alpha_end_locked(void) {
    if (g_gui_alpha_fd >= 0) close(g_gui_alpha_fd);
    g_gui_alpha_fd = -1;
}

static void gui_alpha_end(void) {
    pthread_mutex_lock(&g_gui_alpha_mu);
    gui_alpha_end_locked();
    pthread_mutex_unlock(&g_gui_alpha_mu);
}

static int gui_alpha_set_locked(uint8_t alpha) {
    if (g_gui_alpha_fd < 0) {
        int fd = open(GUI_ALPHA_PATH, O_RDWR | O_CREAT | O_CLOEXEC, 0666);
        if (fd < 0) {
            vlog("gui alpha: open failed: %s", strerror(errno));
            return -1;
        }
        int locked = 0;
        for (int i = 0; i < 50 && !g_exit; i++) {
            if (flock(fd, LOCK_EX | LOCK_NB) == 0) {
                locked = 1;
                break;
            }
            if (errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR) break;
            usleep(10000);
        }
        if (!locked) {
            vlog("gui alpha: lock failed: %s", strerror(errno));
            close(fd);
            return -1;
        }
        gui_alpha_request previous;
        ssize_t n = pread(fd, &previous, sizeof(previous), 0);
        if (n != 0 && n != sizeof(previous)) {
            vlog("gui alpha: previous request read failed");
            close(fd);
            return -1;
        }
        if (n == sizeof(previous)) g_gui_alpha_sequence = previous.request;
        g_gui_alpha_fd = fd;
    }
    gui_alpha_request request = {
        .request = ++g_gui_alpha_sequence,
        .alpha = alpha,
    };
    if (!request.request) request.request = ++g_gui_alpha_sequence;
    if (pwrite(g_gui_alpha_fd, &request, sizeof(request), 0) != sizeof(request)) {
        vlog("gui alpha: request write failed: %s", strerror(errno));
        gui_alpha_end_locked();
        return -1;
    }
    for (int i = 0; i < 100 && !g_exit; i++) {
        gui_alpha_request reply;
        if (pread(g_gui_alpha_fd, &reply, sizeof(reply), 0) != sizeof(reply)) {
            vlog("gui alpha: acknowledgement read failed");
            break;
        }
        if (reply.acknowledged == request.request) {
            if (!reply.status) {
                g_gui_alpha_value = alpha;
                return 0;
            }
            vlog("gui alpha: overlay rejected request (%u)", reply.status);
            gui_alpha_end_locked();
            return -1;
        }
        usleep(20000);
    }
    vlog("gui alpha: overlay did not acknowledge");
    gui_alpha_end_locked();
    return -1;
}

static int gui_alpha_set(uint8_t alpha) {
    pthread_mutex_lock(&g_gui_alpha_mu);
    int result = gui_alpha_set_locked(alpha);
    pthread_mutex_unlock(&g_gui_alpha_mu);
    return result;
}

static int gui_alpha_refresh(void) {
    pthread_mutex_lock(&g_q_mu);
    int pane_active = g_pane_active;
    pthread_mutex_unlock(&g_q_mu);
    pthread_mutex_lock(&g_gui_alpha_mu);
    int result = g_gui_alpha_fd < 0 && !pane_active ? 0 :
        gui_alpha_set_locked(g_gui_alpha_fd < 0 ? 255 : g_gui_alpha_value);
    pthread_mutex_unlock(&g_gui_alpha_mu);
    return result;
}

static int gui_alpha_active(void) {
    pthread_mutex_lock(&g_gui_alpha_mu);
    int active = g_gui_alpha_fd >= 0;
    pthread_mutex_unlock(&g_gui_alpha_mu);
    return active;
}

static void gui_alpha_disconnect(void) {
    pthread_mutex_lock(&g_q_mu);
    int retain = opt_pane_reconnect && g_pane_active && g_pane_has_frame;
    pthread_mutex_unlock(&g_q_mu);
    if (!retain) gui_alpha_end();
}

static void drain_inflight_pane(void) {
    pthread_mutex_lock(&g_q_mu);
    if (opt_pane_reconnect && g_pane_active && !g_pane_has_frame && !g_exit) {
        g_disconnect_drain_pending = 1;
        pthread_cond_signal(&g_q_cv);
        while (g_disconnect_drain_pending) pthread_cond_wait(&g_q_cv, &g_q_mu);
        if (g_disconnect_drain_result)
            vlog("pane: ending bridge after first-frame drain failure");
    }
    pthread_mutex_unlock(&g_q_mu);
}

static int pane_active_now(void) {
    pthread_mutex_lock(&g_q_mu);
    int active = g_pane_active;
    pthread_mutex_unlock(&g_q_mu);
    return active;
}

/* Validate a Configure, take what a new GFX1 session needs, and have the decoder thread
 * apply it. Anything but READY or FAILED leaves the active stream as it was. */
static uint32_t configure(const eyepiece_rpc_v1_Configure *request, int pane_supported) {
    if (!ep_config_valid(request, (uint16_t)opt_fb_w, (uint16_t)opt_fb_h)) return EP_CONFIGURE_INVALID;
    if (!opt_video || (request->has_pane && !pane_supported)) return EP_CONFIGURE_UNSUPPORTED;
    if (!request->has_pane && request->layout == eyepiece_rpc_v1_StreamLayout_LAYOUT_SBS &&
        !(opt_video && opt_hw_tune.direct_vops))
        return EP_CONFIGURE_UNSUPPORTED;

    int first_pane = request->has_pane && !pane_active_now();
    if (first_pane && !opt_null_dec && stop_arhome_for_gfx1() < 0) {
        vlog("cannot prepare GFX1");
        restore_arhome();
        return EP_CONFIGURE_UNSUPPORTED;
    }
    pthread_mutex_lock(&g_q_mu);
    uint32_t status;
    if (g_exit) {
        status = EP_CONFIGURE_FAILED;
    } else {
        g_config_request = *request;
        g_config_pending = 1;
        pthread_cond_signal(&g_q_cv);
        while (g_config_pending) pthread_cond_wait(&g_q_cv, &g_q_mu);
        status = (uint32_t)g_config_result;
    }
    int pane_active = g_pane_active;
    pthread_mutex_unlock(&g_q_mu);
    if (first_pane && !pane_active && status != EP_CONFIGURE_FAILED && !opt_null_dec) restore_arhome();
    return status;
}

static void serve_client(int fd, int usb_bulk) {
    int one = 1;
    if (!usb_bulk) setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    pthread_mutex_lock(&g_tx_mu);
    g_cfd = fd;
    g_cfd_bulk = usb_bulk;
    pthread_mutex_unlock(&g_tx_mu);
    pthread_mutex_lock(&g_q_mu);
    g_need_key = 1;
    pthread_mutex_unlock(&g_q_mu);

    int pane_supported = (opt_once || opt_pane_reconnect) && opt_video &&
                         (opt_direct == 1 || opt_null_dec) &&
                         opt_fb_w == 1920 && opt_fb_h == 1080 &&
                         !getenv("EYEPIECE_STEREO");
    eyepiece_rpc_v1_Envelope info = eyepiece_rpc_v1_Envelope_init_zero;
    info.which_message = eyepiece_rpc_v1_Envelope_info_tag;
    info.message.info = (eyepiece_rpc_v1_Info){ EP_VERSION, (uint16_t)opt_fb_w, (uint16_t)opt_fb_h,
                       (uint8_t)(opt_video && !opt_null_dec ? 2 : 0), g_ht_flags,
                       ((opt_video && opt_hw_tune.direct_vops ? eyepiece_rpc_v1_BridgeCapability_CAPABILITY_STEREO : 0) |
                                 (pane_supported ? eyepiece_rpc_v1_BridgeCapability_CAPABILITY_PANE_BGRA | eyepiece_rpc_v1_BridgeCapability_CAPABILITY_PANE_OPAQUE_BGRA | eyepiece_rpc_v1_BridgeCapability_CAPABILITY_PANE_ALPHA : 0) |
                                 (pane_supported && opt_pane_reconnect ? eyepiece_rpc_v1_BridgeCapability_CAPABILITY_PANE_RECONNECT : 0) |
                                 (pane_supported ? eyepiece_rpc_v1_BridgeCapability_CAPABILITY_RECONFIG : 0) |
                                 eyepiece_rpc_v1_BridgeCapability_CAPABILITY_GUI_ALPHA |
                                 (gui_alpha_active() ? eyepiece_rpc_v1_BridgeCapability_CAPABILITY_GUI_ALPHA_ACTIVE : 0)) };
    tx(&info);
    vlog("client connected");

    uint64_t frames = 0, bytes = 0, t_stat = now_us();
    int configured = 0;              /* this connection has an acknowledged Configure */
    while (!g_exit) {
        uint32_t length_be;
        int read_result = usb_bulk ? read_all_bulk(fd, &length_be, sizeof(length_be)) : read_all(fd, &length_be, sizeof(length_be));
        if (read_result < 0) break;
        uint32_t length = ntohl(length_be);
        if (!length || length > EP_MAX_PAYLOAD) {
            vlog("protocol error (protobuf length %u; incompatible client?) → dropping client", length);
            break;
        }
        uint8_t *p = malloc(length);
        if (!p) { vlog("protocol: cannot allocate message buffer"); break; }
        read_result = usb_bulk ? read_all_bulk(fd, p, length) : read_all(fd, p, length);
        if (read_result < 0) { free(p); break; }
        eyepiece_rpc_v1_Envelope h;
        ep_bytes_t video, id;
        if (ep_wire_decode(p, length, &h, &video, &id) < 0) {
            vlog("protocol: invalid protobuf message");
            free(p);
            break;
        }
        switch (h.which_message) {
        case eyepiece_rpc_v1_Envelope_video_tag: {
            if (!video.length || !opt_video) break;
            pthread_mutex_lock(&g_q_mu);
            int matches = configured && g_configured && h.message.video.codec == g_config.codec;
            pthread_mutex_unlock(&g_q_mu);
            if (!matches) {
                vlog("protocol: video without a matching acknowledged Configure");
                shutdown(fd, SHUT_RDWR);
                break;
            }
            uint32_t n = (uint32_t)video.length;
            au_t a = { p, video.data, n, h.message.video.pts_us, now_us(), h.message.video.codec, h.message.video.key };
            q_push(a);
            frames++; bytes += n;
            p = NULL;
            break;
        }
        case eyepiece_rpc_v1_Envelope_configure_tag: {
            eyepiece_rpc_v1_Configure request = h.message.configure;
            eyepiece_rpc_v1_Envelope reply = eyepiece_rpc_v1_Envelope_init_zero;
            reply.which_message = eyepiece_rpc_v1_Envelope_configure_ack_tag;
            ep_wire_set_bytes(&reply.id, &id);
            uint32_t status = configure(&request, pane_supported);
            reply.message.configure_ack.status = status;
            tx(&reply);
            if (status == EP_CONFIGURE_READY) {
                configured = 1;
                uint32_t width = 0, height = 0;
                ep_config_frame_size(&request, (uint16_t)opt_fb_w, (uint16_t)opt_fb_h, &width, &height);
                vlog("configure: %ux%u %s%s", width, height,
                     request.codec == eyepiece_rpc_v1_Codec_CODEC_HEVC ? "HEVC" : "H264",
                     request.has_pane ? (request.pane.format == eyepiece_rpc_v1_PaneFormat_PANE_BGRA ?
                                         " packed-alpha pane" : " opaque pane") :
                                        (request.layout == eyepiece_rpc_v1_StreamLayout_LAYOUT_SBS ? " stereo" : ""));
            } else if (status == EP_CONFIGURE_FAILED) {
                vlog("configuration failed; closing connection");
                configured = 0;
                shutdown(fd, SHUT_RDWR);
            } else {
                vlog("configuration rejected (status %u); keeping the active stream", status);
            }
            break;
        }
        case eyepiece_rpc_v1_Envelope_gui_alpha_tag: {
            eyepiece_rpc_v1_GuiAlpha request = h.message.gui_alpha;
            eyepiece_rpc_v1_Envelope reply = eyepiece_rpc_v1_Envelope_init_zero;
            reply.which_message = eyepiece_rpc_v1_Envelope_gui_alpha_ack_tag;
            ep_wire_set_bytes(&reply.id, &id);
            if (request.alpha > UINT8_MAX) reply.message.gui_alpha_ack.status = 1;
            else if (gui_alpha_set(request.alpha) != 0) reply.message.gui_alpha_ack.status = 2;
            tx(&reply);
            break;
        }
        case eyepiece_rpc_v1_Envelope_pane_alpha_tag: {
            eyepiece_rpc_v1_PaneAlpha request = h.message.pane_alpha;
            eyepiece_rpc_v1_Envelope reply = eyepiece_rpc_v1_Envelope_init_zero;
            reply.which_message = eyepiece_rpc_v1_Envelope_pane_alpha_ack_tag;
            ep_wire_set_bytes(&reply.id, &id);
            eyepiece_rpc_v1_Acknowledgement *ack = &reply.message.pane_alpha_ack;
            if (request.alpha > UINT8_MAX) {
                ack->status = 1;
            } else if (!pane_supported || !configured || !pane_active_now()) {
                ack->status = 2;
            } else {
                pthread_mutex_lock(&g_q_mu);
                if (g_exit) {
                    ack->status = 2;
                } else {
                    g_pane_alpha_request = (uint8_t)request.alpha;
                    g_pane_alpha_pending = 1;
                    pthread_cond_signal(&g_q_cv);
                    while (g_pane_alpha_pending) pthread_cond_wait(&g_q_cv, &g_q_mu);
                    ack->status = g_pane_alpha_result ? 2 : 0;
                }
                pthread_mutex_unlock(&g_q_mu);
            }
            if (ack->status) vlog("pane opacity request rejected (status %u)", ack->status);
            tx(&reply);
            break;
        }
        case eyepiece_rpc_v1_Envelope_ping_tag: {
            eyepiece_rpc_v1_Envelope reply = eyepiece_rpc_v1_Envelope_init_zero;
            reply.which_message = eyepiece_rpc_v1_Envelope_pong_tag;
            reply.message.pong = (eyepiece_rpc_v1_Pong){ h.message.ping.pc_us, now_us() };
            tx(&reply);
            break;
        }
        case eyepiece_rpc_v1_Envelope_control_tag:
            if (h.message.control.operation == eyepiece_rpc_v1_ControlOperation_CONTROL_RECENTER) {
                pthread_mutex_lock(&g_pose_mu); g_recenter = 1; pthread_mutex_unlock(&g_pose_mu);
                vlog("recenter requested");
            } else if (h.message.control.operation == eyepiece_rpc_v1_ControlOperation_CONTROL_HEADTRACK) {
                vlog("head tracking %s requested", h.message.control.enabled ? "on" : "off");
                if (h.message.control.enabled) { pthread_mutex_lock(&g_pose_mu); g_recenter = 1; pthread_mutex_unlock(&g_pose_mu); }
                if (opt_headtrack) headtrack_enable(h.message.control.enabled != 0);
            } else if (h.message.control.operation == eyepiece_rpc_v1_ControlOperation_CONTROL_QUIT) {
                vlog("quit requested by PC");
                g_exit = 1;
            } else {
                vlog("protocol: ignoring unknown control operation %u", h.message.control.operation);
            }
            break;
        default:
            vlog("protocol: ignoring unknown message type %u", h.which_message);
            break;
        }
        free(p);
        uint64_t t = now_us();
        if (t - t_stat > 5000000) {
            pthread_mutex_lock(&g_q_mu);
            uint32_t dropped = g_dropped;
            pthread_mutex_unlock(&g_q_mu);
            vlog("rx %.1f fps, %.2f Mbit/s, dropped %u, pose samples %u",
                 frames * 1e6 / (double)(t - t_stat), bytes * 8.0 / (double)(t - t_stat),
                 dropped, g_pose_count);
            frames = bytes = 0; t_stat = t;
        }
    }
    pthread_mutex_lock(&g_tx_mu);
    g_cfd = -1;
    g_cfd_bulk = 0;
    pthread_mutex_unlock(&g_tx_mu);
    if (!usb_bulk) close(fd);
    if (opt_headtrack) headtrack_enable(0);   /* only while a PC asks for it */
    pthread_mutex_lock(&g_q_mu);
    q_clear_locked();
    pthread_mutex_unlock(&g_q_mu);
    drain_inflight_pane();
    gui_alpha_disconnect();
    vlog("client disconnected");
}

static void on_signal(int sig) { (void)sig; g_exit = 1; }

/* On a crash, reset stereo timing before restoring the default signal action.
 * Only async-signal-safe calls here.                                      */
static void on_crash(int sig) {
    dispout_stereo_off();         /* nor in the 3D timing */
    signal(sig, SIG_DFL);
    raise(sig);
}

static void usage(const char *a0) {
    fprintf(stderr,
        "usage: %s [options]\n"
        "  -p PORT     TCP port on 127.0.0.1 (default %d; PC: adb forward tcp:%d tcp:%d)\n"
        "  -A          listen on all interfaces (e.g. Wi-Fi) instead of loopback\n"
        "  -B          add a dedicated FunctionFS bulk interface alongside the current USB functions\n"
        "  -s WxH      output dimensions (default 1920x1080)\n"
        "  -c N        colour-convert threads (default 4)\n"
        "  -V          no video (head tracking only)\n"
        "  -H          no head tracking\n"
        "  -Z          only send the MCU head-attitude stop, then exit (recovery)\n"
        "  -b          exit on Back key (menu launches)\n"
        "  -1          exit after the first client disconnects\n"
        "  -R          allow matching GFX1 sessions to reconnect; stay running until quit\n"
        "  -F N        hw: number of frame buffers (default 12)\n"
        "  -S DIR      hw: duss_hal_mem_sync(frame, DIR) before reading frames (default: none)\n"
        "  -O plane[:M] zero-copy scan-out on VOP mask M (1 = vop0, 3 = both VOPs;\n"
        "              default: plane:1; GUI stays visible, opacity uses the RPC)\n"
        "  -L R,F,V    hw: start-config knobs: no_reorder (default 1), +0x24, +0x2c (default 0)\n"
        "  -N          null decoder (protocol/latency plumbing test, nothing displayed)\n"
        "  -l FILE     log file (default /data/local/tmp/eyepiece.log, '-' = none)\n",
        a0, EP_PORT, EP_PORT, EP_PORT);
}

/* One bridge at a time, the newest wins. A second bridge (menu tap while a
 * PC-deployed one runs, or the other way round) used to fail on the busy port
 * and overwrite the running bridge's log. Now the newcomer asks the holder of
 * eyepiece.lock to shut down cleanly (SIGTERM: stops head tracking, hands the
 * display back), waits for the lock, and only then touches shared state. The
 * lock is inherited by the head-tracking helper, so it is free only once the
 * old MCU stream is stopped too.                                           */
static int g_took_over = 0;
static int take_over(void) {
    int fd = open(tmp_path("eyepiece.lock"), O_RDWR | O_CREAT | O_CLOEXEC, 0666);
    if (fd < 0) return 0;                        /* no lock dir: old behaviour */
    if (flock(fd, LOCK_EX | LOCK_NB) < 0) {
        char b[16] = {0};
        (void)!pread(fd, b, sizeof(b) - 1, 0);
        pid_t old = (pid_t)atoi(b);
        fprintf(stderr, "another bridge (pid %d) is running: asking it to exit\n", (int)old);
        if (old > 1) kill(old, SIGTERM);
        int ok = 0;
        for (int i = 0; i < 50 && !ok; i++) { usleep(100000); ok = flock(fd, LOCK_EX | LOCK_NB) == 0; }
        if (!ok && old > 1) {
            kill(old, SIGKILL);                  /* its helper stops the MCU on parent death */
            for (int i = 0; i < 30 && !ok; i++) { usleep(100000); ok = flock(fd, LOCK_EX | LOCK_NB) == 0; }
        }
        if (!ok) { close(fd); return -1; }
        g_took_over = 1;
    }
    char b[16];
    int n = snprintf(b, sizeof(b), "%d\n", (int)getpid());
    if (ftruncate(fd, 0) == 0) (void)!pwrite(fd, b, (size_t)n, 0);
    return 0;                                    /* fd stays open = lock held */
}

int main(int argc, char **argv) {
    int o;
    while ((o = getopt(argc, argv, "p:As:c:VHb1RNF:S:L:O:l:ZB")) != -1) {
        switch (o) {
        case 'p': opt_port = atoi(optarg); break;
        case 'A': opt_any_addr = 1; break;
        case 's': if (sscanf(optarg, "%dx%d", &opt_fb_w, &opt_fb_h) != 2) { usage(argv[0]); return 2; } break;
        case 'c': opt_cvt_threads = atoi(optarg); break;
        case 'V': opt_video = 0; break;
        case 'H': opt_headtrack = 0; break;
        case 'b': opt_back = 1; break;
        case '1': opt_once = 1; break;
        case 'R': opt_pane_reconnect = 1; opt_once = 0; break;
        case 'N': opt_null_dec = 1; break;
        case 'F': opt_hw_frames = atoi(optarg); opt_hw_frames_set = 1; break;
        case 'O':
            if (strncmp(optarg, "plane", 5) || (optarg[5] && optarg[5] != ':')) {
                fprintf(stderr, "invalid -O value: %s\n", optarg);
                return 2;
            }
            opt_direct = optarg[5] == ':' ? atoi(optarg + 6) : 1;
            break;
        case 'S': opt_hw_sync = atoi(optarg); break;
        case 'L': sscanf(optarg, "%d,%d,%d", &opt_hw_tune.no_reorder, &opt_hw_tune.flag24,
                         &opt_hw_tune.val2c); break;
        case 'l': opt_log = optarg; break;
        case 'Z': opt_mcu_stop = 1; break;   /* only send the MCU attitude stop */
        case 'B': opt_bulk = 1; break;
        default: usage(argv[0]); return 2;
        }
    }
    if (opt_mcu_stop) {
        /* stopper / manual recovery: stderr only — never touch the running
         * bridge's log file or display                                     */
        int rc = headtrack_stop_cli();
        fflush(stderr);
        _exit(rc);   /* skip DJI framework atexit/destructors */
    }
    opt_hw_tune.direct_vops = opt_null_dec ? 0 : opt_direct;
    if (opt_hw_tune.direct_vops)
        opt_hw_tune.prepare_output = backdrop_prepare_output;
    if (take_over() < 0) {
        fprintf(stderr, "another bridge did not exit; giving up\n");
        _exit(1);
    }
    /* the overlay may still be cleaning up after the previous app (it pokes
     * the GUI with key events for ~2 s); don't grab the keys or claim the
     * screen before it is done                                           */
    if (g_took_over) usleep(300000);   /* let the overlay notice the old bridge left */
    for (int i = 0; i < 50 && access(tmp_path("fb_cleanup"), F_OK) == 0; i++) usleep(100000);
    if (!opt_log) {
        opt_log = tmp_path("eyepiece.log");
        char prev[300];
        snprintf(prev, sizeof(prev), "%s.1", opt_log);
        rename(opt_log, prev);                   /* keep the previous session's log */
    }
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
    signal(SIGSEGV, on_crash);
    signal(SIGBUS, on_crash);
    signal(SIGABRT, on_crash);
    signal(SIGILL, on_crash);

    vlog("eyepiece_bridge v%d starting (port %d, panel %dx%d)", EP_VERSION, opt_port, opt_fb_w, opt_fb_h);
    if (bulk_recover() < 0) {
        vlog("bulk: refusing startup because interrupted gadget recovery failed");
        return 1;
    }
    if (opt_video && opt_hw_tune.direct_vops && !opt_null_dec) dispout_reset_stale_3d();

    /* fork the bus helper before any thread exists */
    if (opt_headtrack && headtrack_spawn(on_pose) < 0)
        vlog("bus head tracking unavailable");

    pthread_t th, decode_th;
    int startup_failed = 0, ls = -1;
    int decode_started = 0;
    int r = pthread_create(&th, NULL, pose_thread, NULL);
    if (r != 0) {
        vlog("shutdown watcher thread creation failed: %s", strerror(r));
        startup_failed = 1;
        goto shutdown;
    }
    r = pthread_detach(th);
    if (r != 0) vlog("shutdown watcher thread detach failed: %s", strerror(r));
    if (opt_video) {
        r = pthread_create(&decode_th, NULL, decode_thread, NULL);
        if (r != 0) {
            vlog("decode thread creation failed: %s", strerror(r));
            startup_failed = 1;
            goto shutdown;
        }
        decode_started = 1;
    }
    if (opt_back) {
        r = pthread_create(&th, NULL, back_thread, NULL);
        if (r != 0) {
            vlog("back thread creation failed: %s", strerror(r));
            startup_failed = 1;
            goto shutdown;
        }
        r = pthread_detach(th);
        if (r != 0) vlog("back thread detach failed: %s", strerror(r));
    }

    if (opt_bulk && bulk_setup() < 0)
        vlog("bulk: unavailable; continuing with TCP transports");
    ls = g_ls = socket(AF_INET, SOCK_STREAM, 0);
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
        int bulk_fd = opt_bulk ? bulk_pending_client() : -1;
        struct pollfd pfds[3];
        int ls_index = -1, ep0_index = -1, bulk_index = -1;
        nfds_t nfd = 0;
        if (ls >= 0) {
            ls_index = (int)nfd;
            pfds[nfd++] = (struct pollfd){ .fd = ls, .events = POLLIN };
        }
        int ep0 = opt_bulk ? bulk_event_fd() : -1;
        if (ep0 >= 0) {
            ep0_index = (int)nfd;
            pfds[nfd++] = (struct pollfd){ .fd = ep0, .events = POLLIN };
        }
        if (bulk_fd >= 0) {
            bulk_index = (int)nfd;
            pfds[nfd++] = (struct pollfd){ .fd = bulk_fd, .events = POLLIN };
        }
        if (!nfd) break;
        int ready = poll(pfds, nfd, -1);
        if (ready < 0) {
            if (errno == EINTR) continue;
            if (!g_exit) vlog("poll: %s", strerror(errno));
            break;
        }
        if (ep0_index >= 0 && (pfds[ep0_index].revents & (POLLIN | POLLERR | POLLHUP))) {
            bulk_update_events();
        }
        if (bulk_index >= 0 &&
            ((pfds[bulk_index].revents & POLLIN) || bulk_session_selected())) {
            serve_client(bulk_fd, 1);
            bulk_client_done();
            if (opt_once) break;
            continue;
        }
        if (bulk_index >= 0 && (pfds[bulk_index].revents & (POLLERR | POLLHUP | POLLNVAL))) {
            bulk_client_done();
        }
        if (ls_index < 0 || !(pfds[ls_index].revents & POLLIN)) continue;

        int fd = accept(ls, NULL, NULL);
        if (fd < 0) {
            if (errno == EINTR) continue;
            if (!g_exit) vlog("accept: %s", strerror(errno));
            break;
        }
        if (bulk_fd >= 0 && !bulk_tcp_claim()) {
            close(fd);
            if (bulk_session_selected()) {
                serve_client(bulk_fd, 1);
                bulk_client_done();
                if (opt_once) break;
            }
            continue;
        }
        serve_client(fd, 0);
        if (bulk_fd >= 0) bulk_tcp_done();
        if (opt_once) break;
    }

shutdown:
    vlog("shutting down");
    pthread_mutex_lock(&g_q_mu);
    g_exit = 1;
    pthread_cond_broadcast(&g_q_cv);
    pthread_mutex_unlock(&g_q_mu);
    if (decode_started && g_pane_active) pthread_join(decode_th, NULL);
    else if (decode_started) pthread_detach(decode_th);
    if (ls >= 0) close(ls);
    if (opt_bulk) bulk_stop();
    if (opt_headtrack) headtrack_stop();
    dispout_stereo_off();                /* VOP0 back to the 2D timing */
    restore_arhome();
    if (g_log) fclose(g_log);
    _exit(startup_failed ? 1 : 0);   /* don't run DJI framework atexit/destructors */
}
