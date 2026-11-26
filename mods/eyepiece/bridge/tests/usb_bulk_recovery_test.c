// SPDX-License-Identifier: AGPL-3.0-or-later
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

static int test_fflush(FILE *stream);
static int test_fsync(int fd);
static int test_open(const char *path, int flags, ...);
static int test_pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                               void *(*start)(void *), void *arg);
#define fflush test_fflush
#define fsync test_fsync
#define open test_open
#define pthread_create test_pthread_create
#include "../usb_bulk.c"
#undef fflush
#undef fsync
#undef open
#undef pthread_create

#include <assert.h>
#include <stdlib.h>

static char state_path[256];
static int fail_flush, fail_sync, sync_calls, log_count;
static int fail_thread_call, thread_calls;
static size_t expected_size;

const char *tmp_path(const char *name) { (void)name; return state_path; }
void vlog(const char *fmt, ...) { (void)fmt; log_count++; }

static int test_open(const char *path, int flags, ...) {
    if (!strcmp(path, EP_IN) || !strcmp(path, EP_OUT)) return open("/dev/null", flags);
    if (!(flags & O_CREAT)) return open(path, flags);
    va_list ap;
    va_start(ap, flags);
    mode_t mode = (mode_t)va_arg(ap, int);
    va_end(ap);
    return open(path, flags, mode);
}

static int test_pthread_create(pthread_t *thread, const pthread_attr_t *attr,
                               void *(*start)(void *), void *arg) {
    if (fail_thread_call && ++thread_calls == fail_thread_call) return EAGAIN;
    return pthread_create(thread, attr, start, arg);
}

static int test_fflush(FILE *stream) {
    if (fail_flush) { errno = EIO; return EOF; }
    return fflush(stream);
}

static int test_fsync(int fd) {
    struct stat st;
    assert(fstat(fd, &st) == 0);
    assert((size_t)st.st_size == expected_size);
    sync_calls++;
    if (fail_sync) { errno = EIO; return -1; }
    return fsync(fd);
}

static void check_snapshot(const char *expected) {
    char content[128];
    int n = read_text(state_path, content, sizeof(content));
    assert(n == (int)strlen(expected));
    assert(!strcmp(content, expected));
}

static void check_no_temporary_file(void) {
    char path[260];
    assert(snprintf(path, sizeof(path), "%s.tmp", state_path) < (int)sizeof(path));
    assert(access(path, F_OK) == -1 && errno == ENOENT);
}

int main(void) {
    char dir[] = "/tmp/eyepiece-bulk-test-XXXXXX";
    assert(mkdtemp(dir));
    assert(snprintf(state_path, sizeof(state_path), "%s/state", dir) > 0);
    assert(bulk_recover() == 0 && log_count == 0);

    strcpy(g_old_udc, "gadget.0");
    expected_size = strlen("gadget.0 3\n");
    assert(save_state(3) == 0);
    assert(sync_calls == 1 && g_state_saved);
    check_snapshot("gadget.0 3");

    g_state_saved = 0;
    fail_flush = 1;
    assert(save_state(4) == -EIO);
    assert(sync_calls == 1 && !g_state_saved);
    check_snapshot("gadget.0 3");
    check_no_temporary_file();
    fail_flush = 0;
    fail_sync = 1;
    assert(save_state(4) == -EIO);
    assert(sync_calls == 2 && !g_state_saved);
    check_snapshot("gadget.0 3");
    check_no_temporary_file();

    int fd = open(state_path, O_WRONLY | O_TRUNC);
    assert(fd >= 0 && close(fd) == 0);
    assert(bulk_recover() == -1 && log_count == 1);
    assert(unlink(state_path) == 0);
    strcpy(state_path, dir);
    assert(bulk_recover() == -1 && log_count == 2);
    assert(rmdir(dir) == 0);

    for (int failure = 1; failure <= 2; failure++) {
        fail_thread_call = failure;
        thread_calls = 0;
        errno = 0;
        assert(start_client() == -1 && errno == EAGAIN);
        assert(g_server_fd == -1 && g_pump_fd == -1 && g_ep_in == -1 && g_ep_out == -1);
        assert(!g_rx_started && !g_tx_started && !g_pump_wake_installed);
    }
    return 0;
}
