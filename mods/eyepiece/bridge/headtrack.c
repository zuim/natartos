// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * headtrack.c — head orientation from the goggles' MCU (no drone needed).
 *
 * The MCU (bus host imu:2) streams its head attitude as a unit quaternion at
 * 100 Hz once asked:
 *   start  Glass 0x15/0x31 {01 00 00 00 00} (ACK) from test:7 to imu:2
 *   data   Glass 0x15/0x32, 26 B: f32 w,x,y,z; u32 t_us; u32 0; u16 rate —
 *          sent by the MCU to dji_media_server, which relays it to
 *          media_service/media_player = test:3.  We hold test:3 ourselves
 *          (only dji_media_server's liveview-ATW AttitudeParser uses it, and
 *          only with a drone stream).
 *   stop   Glass 0x15/0x31 {00 00 00 00 00}
 *
 * The stream outlives its requester, and with nobody on test:3 every packet
 * logs a router error into /blackbox, so the stop is sent on every exit path:
 * helper exit, bridge death (helper PDEATHSIG), helper death (the bridge runs
 * a "eyepiece_bridge -Z" stopper), and `eyepiece_bridge -Z` by hand / from the PC tools.
 * Rules: start only if we got test:3; stop only what we started; nothing but
 * these two fixed payloads is ever sent to imu:2.
 *
 * The bus clients live in a forked helper process so DJI's framework init and
 * signal handlers stay out of the decoder/display process.
 */
#include <dlfcn.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>
#include "bridge.h"
#include "duss.h"

static duss_api_t api;
static void *g_client;          /* test:7: MCU start/stop        */
static void *g_client_mp;       /* test:3: receives the stream   */
static uint16_t g_seq = 0x4000;
static volatile uint64_t g_last_mcu_us;
static volatile uint32_t g_mcu_count, g_mcu_bad;
static pose_cb_t g_cb;
static int g_mcu_started;       /* we sent a successful start → we owe a stop */

static int load_duss(void) {
    void *h = dlopen("libduml_frwk.so", RTLD_NOW | RTLD_GLOBAL);
    if (!h) { vlog("headtrack: dlopen libduml_frwk.so: %s", dlerror()); return -1; }
#define L(f, n) do { *(void **)&api.f = dlsym(h, n); \
        if (!api.f) { vlog("headtrack: missing %s", n); return -1; } } while (0)
    L(sketch_initialize, "duss_sketch_initialize");
    L(sketch_get_config_file_name, "duss_sketch_get_config_file_name");
    L(mb_create_route_table, "duss_mb_create_route_table");
    L(mb_destroy_route_table, "duss_mb_destroy_route_table");
    L(event_create_client, "duss_event_create_client");
    L(event_start_client, "duss_event_start_client");
    L(event_send_sync_v2, "duss_event_send_sync_v2");
    L(event_register_dynamic_command_with_user_data,
      "duss_event_register_dynamic_command_with_user_data");
#undef L
    return 0;
}

static int send_sync(uint16_t dst, uint32_t msg_id, const void *data, uint32_t len,
                     uint8_t *resp_byte0) {
    uint8_t buf[DUSS_MSG_HDR + 64];
    uint8_t resp[DUSS_MSG_HDR + 1024];
    uint32_t resp_len = sizeof(resp);
    duss_msg_t *m = (duss_msg_t *)buf;
    if (len > 64) return -1;
    memset(buf, 0, sizeof(buf));
    m->msg_id = msg_id;
    m->dst_host = dst;
    m->attrib = DUSS_ATTR_NEED_ACK | DUSS_ATTR_SEQ;
    m->seq = g_seq++;
    m->len = len;
    memcpy(m->data, data, len);
    memset(resp, 0, sizeof(resp));
    int r = api.event_send_sync_v2(g_client, m, 300000, resp, &resp_len);
    duss_msg_t *rm = (duss_msg_t *)resp;
    if (r == 0 && resp_byte0) *resp_byte0 = rm->len ? rm->data[0] : 0xff;
    return r;
}

/* ---- MCU attitude stream ------------------------------------------------- */

static int mcu_attitude_cmd(int enable) {
    static const uint8_t on[MCU_ATTI_SET_LEN]  = { 1, 0, 0, 0, 0 };
    static const uint8_t off[MCU_ATTI_SET_LEN] = { 0, 0, 0, 0, 0 };
    uint8_t rc = 0xff;
    int r = send_sync(HOST_IMU_MCU, MSG_MCU_ATTI_SET, enable ? on : off, MCU_ATTI_SET_LEN, &rc);
    vlog("headtrack: MCU attitude %s → imu:2: ret=%d resp=0x%02x",
         enable ? "start" : "stop", r, rc);
    return (r == 0 && rc == 0) ? 0 : -1;
}

/* Quaternion (w,x,y,z; x = roll, y = pitch, z = yaw axis) → degrees, ZYX.
 * Verified on the device: this gives the protocol's
 * +yaw = right, +pitch = up, +roll = right ear down.                       */
static void quat_to_ypr(const float q[4], float *yaw, float *pitch, float *roll) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float sp = 2.f * (w * y - z * x);
    if (sp > 1.f) sp = 1.f;
    if (sp < -1.f) sp = -1.f;
    const float deg = 180.f / (float)M_PI;
    *yaw   = atan2f(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z)) * deg;
    *pitch = asinf(sp) * deg;
    *roll  = atan2f(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y)) * deg;
}

/* Glass 0x15/0x32 relayed by dji_media_server to test:3.  Framework thread. */
static int on_mcu_attitude(void *client, duss_msg_t *ev, void *user) {
    (void)client; (void)user;
    if (!ev || ev->len < MCU_ATTI_MSG_LEN || ev->len > 0x1000) { g_mcu_bad++; return 0; }
    float q[4];
    memcpy(q, ev->data, sizeof(q));
    float n = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!(n > 0.9f && n < 1.1f)) { g_mcu_bad++; return 0; }   /* also rejects NaN */
    g_last_mcu_us = now_us();
    g_mcu_count++;
    float yaw, pitch, roll;
    quat_to_ypr(q, &yaw, &pitch, &roll);
    if (g_cb) g_cb(yaw, pitch, roll);
    return 0;
}

/* ---- bus setup ------------------------------------------------------------ */

static int create_client(const char *cfg, const char *svc, const char *role,
                         uint16_t want_host, void **out) {
    static uint8_t rt[DUSS_ROUTE_TABLE_SZ];
    static uint8_t a1[2][DUSS_CLIENT_ATTR1_SZ], a2[2][DUSS_CLIENT_ATTR2_SZ];
    static int slot;
    uint16_t host = 0;
    memset(rt, 0, sizeof(rt));
    int r = api.mb_create_route_table(cfg, svc, role, &host, rt);
    if (r != 0) { vlog("headtrack: route table %s/%s: %d", svc, role, r); return -1; }
    if (host != want_host) {
        vlog("headtrack: %s/%s is host 0x%04x, expected 0x%04x", svc, role, host, want_host);
        api.mb_destroy_route_table(rt);
        return -1;
    }
    int k = slot++ & 1;
    r = api.event_create_client(rt, host, a1[k], a2[k], NULL, out);
    if (r == 0) r = api.event_start_client(*out);
    api.mb_destroy_route_table(rt);
    if (r != 0) { vlog("headtrack: client %s/%s (0x%04x): %d", svc, role, host, r); *out = NULL; return -1; }
    return 0;
}

/* DUSS framework init + the test:7 command client.                         */
static int bus_init(void) {
    if (load_duss() < 0) return -1;
    /* duss_sketch_initialize attribute block, filled exactly like
     * dji_mb_ctrl: {u64 5, u32 0x100, ..., cfg@0xa8, u8 1 @0xbc}          */
    static uint8_t sketch[DUSS_SKETCH_ATTR_SZ];
    const char *cfg = api.sketch_get_config_file_name();
    memset(sketch, 0, sizeof(sketch));
    *(uint64_t *)(sketch + 0x00) = 5;
    *(uint32_t *)(sketch + 0x08) = 0x100;
    *(const char **)(sketch + 0xa8) = cfg;
    sketch[0xbc] = 1;
    /* the framework installs its own SIGTERM/SIGHUP handlers: keep ours    */
    struct sigaction saved_term, saved_int, saved_hup;
    sigaction(SIGTERM, NULL, &saved_term);
    sigaction(SIGINT, NULL, &saved_int);
    sigaction(SIGHUP, NULL, &saved_hup);
    int r = api.sketch_initialize(sketch);
    sigaction(SIGTERM, &saved_term, NULL);
    sigaction(SIGINT, &saved_int, NULL);
    sigaction(SIGHUP, &saved_hup, NULL);
    if (r != 0) { vlog("headtrack: duss_sketch_initialize: %d", r); return -1; }
    if (create_client(cfg, "test", "diag", HOST_TEST_DIAG, &g_client) < 0) return -1;
    vlog("headtrack: on bus as host 0x%04x", HOST_TEST_DIAG);
    return 0;
}

static void mcu_try_start(void) {
    if (g_mcu_started) return;
    if (mcu_attitude_cmd(1) == 0) {
        g_mcu_started = 1;
        g_last_mcu_us = now_us();      /* grace: first packet takes ~10 ms */
    }
}

static int headtrack_bus_start(pose_cb_t cb) {
    g_cb = cb;
    if (bus_init() < 0) return -1;
    /* only if test:3 is free: if dji_media_server's ATW holds it, DJI uses
     * the stream itself → leave it alone (and never stop it)              */
    if (create_client(api.sketch_get_config_file_name(), "media_service", "media_player",
                      HOST_MEDIA_PLAYER, &g_client_mp) < 0) {
        vlog("headtrack: test:3 unavailable → no head tracking (liveview ATW active?)");
        return -1;
    }
    static duss_cmd_desc_t desc;
    memset(&desc, 0, sizeof(desc));
    desc.req_cb = on_mcu_attitude;
    int r = api.event_register_dynamic_command_with_user_data(g_client_mp, 0x15, 0x32, &desc);
    if (r != 0) { vlog("headtrack: register 0x15/0x32: %d", r); return -1; }
    mcu_try_start();
    return 0;
}

/* Called at 10 Hz by the helper.                                           */
static void headtrack_bus_tick(void) {
    static uint64_t last_try, last_log;
    static uint32_t last_count;
    uint64_t t = now_us();
    if (t - g_last_mcu_us > 1000000 && t - last_try > 3000000) {
        /* stream silent: retry.  Normal once at startup — the first start
         * is ACKed, but packets only reach a freshly created test:3 client
         * after ~1 s — and after an MCU reset.                            */
        last_try = t;
        g_mcu_started = 0;
        mcu_try_start();
    }
    if (t - last_log > 10000000) {
        last_log = t;
        vlog("headtrack: MCU attitude %u packets/10 s, %u rejected",
             g_mcu_count - last_count, g_mcu_bad);
        last_count = g_mcu_count;
    }
}

static void headtrack_bus_stop(void) {
    if (g_mcu_started) { mcu_attitude_cmd(0); g_mcu_started = 0; }
    /* don't destroy the clients: the process is about to _exit and the
     * framework cleans the hosts up.                                       */
}

/* Stand-alone stop (stopper process, eyepiece_bridge -Z): joins as test:7 only.  */
int headtrack_mcu_stop_only(void) {
    if (bus_init() < 0) return -1;
    return mcu_attitude_cmd(0);
}

/* ---- helper process ------------------------------------------------------ *
 * The child owns the DJI bus clients and forwards samples over a pipe as
 * fixed 12-byte records {f32 yaw, f32 pitch, f32 roll}.                     */

typedef struct __attribute__((packed)) {
    float yaw, pitch, roll;
} pose_rec_t;

static int g_pipe_w = -1;
static volatile sig_atomic_t g_child_exit;
static pid_t g_child;
static pose_cb_t g_parent_cb;

static void child_on_pose(float yaw, float pitch, float roll) {
    pose_rec_t r = { yaw, pitch, roll };
    if (write(g_pipe_w, &r, sizeof(r)) < 0 && errno == EPIPE) g_child_exit = 1;
}

static void child_sig(int sig) { (void)sig; g_child_exit = 1; }

static void child_main(int wfd) {
    g_pipe_w = wfd;
    prctl(PR_SET_PDEATHSIG, SIGTERM);              /* die with the bridge */
    signal(SIGPIPE, SIG_IGN);
    /* rc 3 = never started (the reader thread then skips the stopper)     */
    if (headtrack_bus_start(child_on_pose) < 0) {
        if (g_mcu_started) { headtrack_bus_stop(); _exit(0); }
        _exit(3);
    }
    struct sigaction sa;                           /* after DUSS init (it installs its own) */
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = child_sig;
    sigaction(SIGTERM, &sa, NULL);
    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGHUP, &sa, NULL);
    while (!g_child_exit && getppid() != 1) {
        headtrack_bus_tick();
        usleep(100000);
    }
    headtrack_bus_stop();
    _exit(0);
}

/* Run "eyepiece_bridge -Z" (fresh process: only joins as test:7 and sends the MCU
 * attitude stop).  Used when the helper died without sending it itself.
 * fork+exec, not plain fork: the bridge is multithreaded by now, and the
 * DJI framework init in a forked copy could deadlock on inherited locks.   */
static void run_stopper(void) {
    pid_t pid = fork();
    if (pid < 0) { vlog("headtrack: stopper fork: %s", strerror(errno)); return; }
    if (pid == 0) {
        char *argv[] = { "eyepiece_bridge", "-Z", NULL };
        execv("/proc/self/exe", argv);
        _exit(127);
    }
    int st = 0;
    for (int i = 0; i < 60; i++) {                 /* ≤ 3 s */
        if (waitpid(pid, &st, WNOHANG) == pid) {
            vlog("headtrack: stopper done (rc %d)", WIFEXITED(st) ? WEXITSTATUS(st) : -1);
            return;
        }
        usleep(50000);
    }
    kill(pid, SIGKILL);
    waitpid(pid, NULL, 0);
    vlog("headtrack: stopper hung, killed — run `eyepiece_bridge -Z` by hand");
}

static volatile int g_helper_done;     /* set by the reader thread after reaping */

static void *reader_thread(void *arg) {
    int fd = (int)(long)arg;
    pose_rec_t r;
    while (read(fd, &r, sizeof(r)) == (ssize_t)sizeof(r)) {
        if (g_parent_cb) g_parent_cb(r.yaw, r.pitch, r.roll);
    }
    close(fd);
    int st = 0;
    waitpid(g_child, &st, 0);             /* EOF = helper is exiting; we are the only reaper */
    int clean = WIFEXITED(st) && (WEXITSTATUS(st) == 0 || WEXITSTATUS(st) == 3);
    vlog("headtrack: helper exited (%s %d)", WIFEXITED(st) ? "rc" : "sig",
         WIFEXITED(st) ? WEXITSTATUS(st) : WTERMSIG(st));
    /* rc 0 = it sent the stop itself; rc 3 = bus init failed, never started.
     * Anything else (crash, SIGKILL): the MCU stream may still be running. */
    if (!clean) run_stopper();
    g_helper_done = 1;
    return NULL;
}

int headtrack_spawn(pose_cb_t cb) {
    g_parent_cb = cb;
    int p[2];
    if (pipe(p) < 0) return -1;
    pid_t pid = fork();
    if (pid < 0) { close(p[0]); close(p[1]); return -1; }
    if (pid == 0) { close(p[0]); child_main(p[1]); }
    close(p[1]);
    g_child = pid;
    pthread_t t;
    pthread_create(&t, NULL, reader_thread, (void *)(long)p[0]);
    pthread_detach(t);
    vlog("headtrack: bus helper pid %d", (int)pid);
    return 0;
}

void headtrack_stop(void) {
    if (g_child <= 0) return;
    kill(g_child, SIGTERM);              /* helper: MCU stop + unsubscribe, then rc 0 */
    for (int i = 0; i < 40 && !g_helper_done; i++) usleep(50000);   /* ≤ 2 s */
    if (!g_helper_done) {
        vlog("headtrack: helper didn't exit, killing it");
        kill(g_child, SIGKILL);          /* reader thread then runs the stopper */
        for (int i = 0; i < 80 && !g_helper_done; i++) usleep(50000);   /* ≤ 4 s */
    }
    g_child = 0;
}

int headtrack_stop_cli(void) {
    signal(SIGPIPE, SIG_IGN);
    int r = headtrack_mcu_stop_only();
    return r == 0 ? 0 : 1;
}

