// SPDX-License-Identifier: AGPL-3.0-or-later
static char test_path[256];
#include <pthread.h>
static int test_pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                               void *(*start)(void *), void *arg);
#define pthread_create test_pthread_create
#define GUI_ALPHA_PATH test_path
#define main eyepiece_bridge_main
#include "../eyepiece_bridge.c"
#undef main
#undef pthread_create

#include <assert.h>
#include <sys/wait.h>
#include "wire_test_helpers.h"

static int fail_thread_call, thread_calls;

static int test_pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                               void *(*start)(void *), void *arg) {
    if (fail_thread_call && ++thread_calls == fail_thread_call) return EAGAIN;
    return pthread_create(thread, attr, start, arg);
}

static void check_thread_start_failures(void) {
    char dir[] = "/tmp/eyepiece-thread-test-XXXXXX";
    assert(mkdtemp(dir));
    for (int failure = 1; failure <= 3; failure++) {
        pid_t child = fork();
        assert(child >= 0);
        if (child == 0) {
            assert(setenv("EYEPIECE_TMP", dir, 1) == 0);
            fail_thread_call = failure;
            char *argv[] = { "eyepiece_bridge", "-H", "-b", "-l", "-", NULL };
            eyepiece_bridge_main(5, argv);
            _exit(2);
        }
        int status;
        assert(waitpid(child, &status, 0) == child);
        assert(WIFEXITED(status) && WEXITSTATUS(status) == 1);
    }
    char lock_path[256];
    assert(snprintf(lock_path, sizeof(lock_path), "%s/eyepiece.lock", dir) < (int)sizeof(lock_path));
    assert(unlink(lock_path) == 0);
    assert(rmdir(dir) == 0);
}

typedef struct {
    uint64_t previous;
    uint32_t alpha;
    uint32_t status;
} fake_request;

static void *fake_gui(void *arg) {
    fake_request *expected = arg;
    for (int i = 0; i < 200; i++) {
        int fd = open(test_path, O_RDWR);
        if (fd >= 0) {
            gui_alpha_request request;
            if (flock(fd, LOCK_SH | LOCK_NB) < 0 && errno == EWOULDBLOCK &&
                pread(fd, &request, sizeof(request), 0) == sizeof(request) &&
                request.request != expected->previous && request.alpha == expected->alpha) {
                assert(pwrite(fd, &expected->previous, sizeof(expected->previous),
                              offsetof(gui_alpha_request, acknowledged)) == sizeof(expected->previous));
                usleep(80000);
                assert(pwrite(fd, &expected->status, sizeof(expected->status),
                              offsetof(gui_alpha_request, status)) == sizeof(expected->status));
                assert(pwrite(fd, &request.request, sizeof(request.request),
                              offsetof(gui_alpha_request, acknowledged)) == sizeof(request.request));
                close(fd);
                return NULL;
            }
            close(fd);
        }
        usleep(10000);
    }
    assert(!"request not received");
    return NULL;
}

static void *serve_test_client(void *arg) {
    serve_client(*(int *)arg, 0);
    return NULL;
}

static void *release_shared_lock(void *arg) {
    int fd = *(int *)arg;
    usleep(80000);
    assert(flock(fd, LOCK_UN) == 0);
    close(fd);
    return NULL;
}

static void *finish_inflight_frame(void *arg) {
    (void)arg;
    pthread_mutex_lock(&g_q_mu);
    while (!g_disconnect_drain_pending) pthread_cond_wait(&g_q_cv, &g_q_mu);
    g_pane_has_frame = 1;
    g_disconnect_drain_result = 0;
    g_disconnect_drain_pending = 0;
    pthread_cond_broadcast(&g_q_cv);
    pthread_mutex_unlock(&g_q_mu);
    return NULL;
}

int main(void) {
    check_thread_start_failures();
    q_push((au_t){ .buf = malloc(1), .key = 1 });
    q_push((au_t){ .buf = malloc(1) });
    assert(!hw_skip_output(NULL));
    q_push((au_t){ .buf = malloc(1) });
    assert(hw_skip_output(NULL));
    pthread_mutex_lock(&g_q_mu);
    q_resync_locked();
    pthread_mutex_unlock(&g_q_mu);
    assert(g_qn == 0 && g_need_key && !hw_skip_output(NULL));
    q_push((au_t){ .buf = malloc(1) });
    assert(g_qn == 0 && g_need_key);
    q_push((au_t){ .buf = malloc(1), .key = 1 });
    assert(g_qn == 1 && !g_need_key);
    pthread_mutex_lock(&g_q_mu);
    q_resync_locked();
    pthread_mutex_unlock(&g_q_mu);

    char dir[256];
    const char *tmp = getenv("EYEPIECE_TMP");
    assert(snprintf(dir, sizeof(dir), "%s/eyepiece-gui-alpha-XXXXXX",
                    tmp && *tmp ? tmp : "/data/local/tmp") < (int)sizeof(dir));
    assert(mkdtemp(dir));
    assert(snprintf(test_path, sizeof(test_path), "%s/request", dir) > 0);
    int fd = open(test_path, O_CREAT | O_EXCL | O_RDWR, 0600);
    assert(fd >= 0);
    gui_alpha_request stale = { 42, 42, 12, 0 };
    assert(write(fd, &stale, sizeof(stale)) == sizeof(stale));
    close(fd);

    assert(gui_alpha_effective(0, 0, 20) == 255);
    assert(gui_alpha_effective(0, 1, 128) == 128);
    assert(gui_alpha_effective(1, 1, 128) == 0);
    const char planes[] = "VOP 0 - vop_lcdc0:\n\tplane 5 - gfx0:\n"
                          "\t\tblending : pos: 0-0, order: 1, alpha: pixel, alg: 0\n"
                          "\tplane 6 - gfx1:\n\t\tblending : pos: 0-0, order: 2, alpha: pixel\n"
                          "VOP 1 - vop_lcdc1:\n\tplane 5 - gfx0:\n\t\tblending : order: 0\n";
    assert(gui_alpha_plane_order(planes, sizeof(planes) - 1, 5) == 1);
    assert(gui_alpha_plane_order(planes, sizeof(planes) - 1, 6) == 2);
    assert(gui_alpha_plane_order(planes, sizeof(planes) - 1, 4) < 0);
    assert(gui_alpha_blend_order(planes, sizeof(planes) - 1, 5) == 1);
    const char collision[] = "VOP 0:\n\tplane 5 - gfx0:\n\t\tblending : order: 1\n"
                             "\tplane 6 - gfx1:\n\t\tblending : order: 1\n";
    assert(gui_alpha_blend_order(collision, sizeof(collision) - 1, 5) == 2);
    assert(gui_alpha_blend_order(collision, sizeof(collision) - 1, 6) == 1);

    fake_request request = { 42, 128, 0 };
    fd = open(test_path, O_RDWR);
    assert(fd >= 0 && flock(fd, LOCK_SH | LOCK_NB) == 0);
    pthread_t thread, release;
    assert(pthread_create(&release, NULL, release_shared_lock, &fd) == 0);
    assert(pthread_create(&thread, NULL, fake_gui, &request) == 0);
    uint64_t start = now_us();
    assert(gui_alpha_set(128) == 0);
    assert(now_us() - start >= 50000);
    assert(pthread_join(release, NULL) == 0);
    assert(pthread_join(thread, NULL) == 0);
    assert(g_gui_alpha_sequence == 43);
    fd = open(test_path, O_RDWR);
    assert(fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) < 0 && errno == EWOULDBLOCK);
    close(fd);

    request = (fake_request){ 43, 128, 0 };
    assert(pthread_create(&thread, NULL, fake_gui, &request) == 0);
    assert(gui_alpha_refresh() == 0);
    assert(pthread_join(thread, NULL) == 0);
    assert(g_gui_alpha_sequence == 44);

    opt_pane_reconnect = g_pane_active = 1;
    g_pane_has_frame = 0;
    assert(pthread_create(&thread, NULL, finish_inflight_frame, NULL) == 0);
    drain_inflight_pane();
    assert(pthread_join(thread, NULL) == 0);
    gui_alpha_disconnect();
    assert(g_gui_alpha_fd >= 0);
    g_pane_has_frame = 0;
    gui_alpha_disconnect();
    assert(g_gui_alpha_fd < 0);

    request = (fake_request){ 44, 255, 0 };
    assert(pthread_create(&thread, NULL, fake_gui, &request) == 0);
    assert(gui_alpha_refresh() == 0);
    assert(pthread_join(thread, NULL) == 0);
    assert(g_gui_alpha_fd >= 0);
    gui_alpha_disconnect();
    assert(g_gui_alpha_fd < 0);
    g_pane_active = 0;
    assert(gui_alpha_refresh() == 0 && g_gui_alpha_fd < 0);

    request = (fake_request){ 45, 64, 2 };
    assert(pthread_create(&thread, NULL, fake_gui, &request) == 0);
    assert(gui_alpha_set(64) < 0);
    assert(pthread_join(thread, NULL) == 0);
    assert(g_gui_alpha_fd < 0);

    int sockets[2];
    assert(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets) == 0);
    opt_once = 1;
    opt_video = opt_headtrack = opt_pane_reconnect = 0;
    pthread_t server;
    assert(pthread_create(&server, NULL, serve_test_client, &sockets[1]) == 0);
    eyepiece_rpc_v1_Info info = eyepiece_rpc_v1_Info_init_zero;
    test_receive(sockets[0], eyepiece_rpc_v1_Envelope_info_tag, eyepiece_rpc_v1_Info_fields, &info);
    assert(info.flags & eyepiece_rpc_v1_BridgeCapability_CAPABILITY_GUI_ALPHA);
    eyepiece_rpc_v1_GuiAlpha message = { 256 };
    test_send_id(sockets[0], eyepiece_rpc_v1_Envelope_gui_alpha_tag, eyepiece_rpc_v1_GuiAlpha_fields, &message, "7");
    eyepiece_rpc_v1_Acknowledgement ack = eyepiece_rpc_v1_Acknowledgement_init_zero;
    test_receive_id(sockets[0], eyepiece_rpc_v1_Envelope_gui_alpha_ack_tag, eyepiece_rpc_v1_Acknowledgement_fields, &ack, "7");
    assert(ack.status == 1);
    request = (fake_request){ 46, 32, 0 };
    assert(pthread_create(&thread, NULL, fake_gui, &request) == 0);
    message.alpha = 32;
    test_send_id(sockets[0], eyepiece_rpc_v1_Envelope_gui_alpha_tag, eyepiece_rpc_v1_GuiAlpha_fields, &message, "8");
    test_receive_id(sockets[0], eyepiece_rpc_v1_Envelope_gui_alpha_ack_tag, eyepiece_rpc_v1_Acknowledgement_fields, &ack, "8");
    assert(ack.status == 0);
    assert(pthread_join(thread, NULL) == 0);
    close(sockets[0]);
    assert(pthread_join(server, NULL) == 0);
    assert(g_gui_alpha_fd < 0);

    int ready[2];
    assert(pipe(ready) == 0);
    pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        close(ready[0]);
        int held = open(test_path, O_RDWR);
        assert(held >= 0 && flock(held, LOCK_EX) == 0);
        assert(write(ready[1], "x", 1) == 1);
        for (;;) pause();
    }
    close(ready[1]);
    char marker;
    assert(read(ready[0], &marker, 1) == 1);
    close(ready[0]);
    fd = open(test_path, O_RDWR);
    assert(fd >= 0 && flock(fd, LOCK_SH | LOCK_NB) < 0 && errno == EWOULDBLOCK);
    assert(kill(child, SIGKILL) == 0);
    assert(waitpid(child, NULL, 0) == child);
    assert(flock(fd, LOCK_SH | LOCK_NB) == 0);
    close(fd);

    assert(unlink(test_path) == 0);
    assert(rmdir(dir) == 0);
    return 0;
}
