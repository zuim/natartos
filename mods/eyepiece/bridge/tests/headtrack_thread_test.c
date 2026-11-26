// SPDX-License-Identifier: AGPL-3.0-or-later
#include <errno.h>
#include <pthread.h>
#include <unistd.h>

static pid_t test_fork(void);
static int fail_thread_create(pthread_t *thread, const pthread_attr_t *attr,
                              void *(*start)(void *), void *arg);
#define fork test_fork
#define pthread_create fail_thread_create
#include "../headtrack.c"
#undef fork
#undef pthread_create

#include <assert.h>
#include <stdarg.h>

static pid_t helper_pid;

void vlog(const char *fmt, ...) { (void)fmt; }
uint64_t now_us(void) { return 1; }

static pid_t test_fork(void) {
    pid_t pid = fork();
    if (pid == 0) for (;;) pause();
    helper_pid = pid;
    return pid;
}

static int fail_thread_create(pthread_t *thread, const pthread_attr_t *attr,
                              void *(*start)(void *), void *arg) {
    (void)thread; (void)attr; (void)start; (void)arg;
    return EAGAIN;
}

int main(void) {
    assert(headtrack_spawn(NULL) == -1);
    assert(helper_pid > 0 && g_child == 0 && g_ctl_w == -1);
    int status;
    assert(waitpid(helper_pid, &status, WNOHANG) == -1 && errno == ECHILD);
    return 0;
}
