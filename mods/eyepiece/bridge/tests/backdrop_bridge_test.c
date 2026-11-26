// SPDX-License-Identifier: AGPL-3.0-or-later
#define main eyepiece_bridge_main
#include "../eyepiece_bridge.c"
#undef main

#include <assert.h>
#include <sys/stat.h>

static char test_path[256];

static void *fake_gui(void *arg) {
    (void)arg;
    for (int i = 0; i < 200; i++) {
        int fd = open(test_path, O_RDWR);
        if (fd >= 0) {
            if (flock(fd, LOCK_EX | LOCK_NB) < 0 && errno == EWOULDBLOCK) {
                eyepiece_backdrop_request request;
                if (pread(fd, &request, sizeof(request), 0) == sizeof(request) &&
                    request.request && request.acknowledged == 0) {
                    assert(pwrite(fd, &request.request, sizeof(request.request),
                                  offsetof(eyepiece_backdrop_request, acknowledged)) ==
                           sizeof(request.request));
                    close(fd);
                    return NULL;
                }
            }
            close(fd);
        }
        usleep(10000);
    }
    assert(!"bridge did not issue a request");
    return NULL;
}

int main(void) {
    char dir[] = "/data/local/tmp/eyepiece-backdrop-XXXXXX";
    assert(mkdtemp(dir));
    assert(setenv("EYEPIECE_TMP", dir, 1) == 0);
    assert(snprintf(test_path, sizeof(test_path), "%s/eyepiece.backdrop", dir) > 0);
    int fd = open(test_path, O_CREAT | O_EXCL | O_RDWR, 0600);
    assert(fd >= 0);
    eyepiece_backdrop_request stale = { 123, 123 };
    assert(write(fd, &stale, sizeof(stale)) == sizeof(stale));
    close(fd);

    pthread_t gui;
    assert(pthread_create(&gui, NULL, fake_gui, NULL) == 0);
    assert(backdrop_prepare_output(NULL) == 0);
    assert(pthread_join(gui, NULL) == 0);
    fd = open(test_path, O_RDWR);
    assert(fd >= 0);
    assert(flock(fd, LOCK_EX | LOCK_NB) < 0 && errno == EWOULDBLOCK);
    close(fd);
    assert(backdrop_prepare_output(NULL) == 0);
    close(g_backdrop_fd);
    g_backdrop_fd = -1;

    assert(backdrop_prepare_output(NULL) < 0);
    assert(g_backdrop_failed && g_exit);
    fd = open(test_path, O_RDWR);
    assert(fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) == 0);
    close(fd);
    assert(unlink(test_path) == 0);
    assert(rmdir(dir) == 0);
    return 0;
}
