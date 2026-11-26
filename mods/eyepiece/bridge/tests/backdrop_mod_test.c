// SPDX-License-Identifier: AGPL-3.0-or-later
#define EYEPIECE_BACKDROP_PATH "/data/local/tmp/eyepiece-backdrop-test-8c051a36"
#include "../../mod.c"

#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <string.h>
#include <unistd.h>

static unsigned char layer[0x320];
static int setter_calls;
static unsigned char *image;
static int fake_signals(void);

struct fake_dl_info { const char *name; void *base; const char *symbol; void *address; };
static int fake_dladdr(const void *address, struct fake_dl_info *info) {
    (void)address;
    *info = (struct fake_dl_info){ "libew_gls.so", image, 0, 0 };
    return 1;
}
static void *fake_dlsym(void *handle, const char *name) {
    (void)handle;
    if (!strcmp(name, "dladdr")) return (void *)fake_dladdr;
    if (!strcmp(name, "__vmt_DuBackgroundBackgroundLayer")) return image + 0x6a06268;
    if (!strcmp(name, "EwProcessSignals")) return (void *)fake_signals;
    return 0;
}

static void fake_init(void *object, void *owner, void *arg) {
    (void)owner; (void)arg;
    *(unsigned int *)((char *)object + 0x40) = 1;
}

static void fake_done(void *object) { (void)object; }
static void fake_visibility(void *object, int visible) {
    unsigned int *flags = (unsigned int *)((char *)object + 0x40);
    *flags = (*flags & ~1U) | !!visible;
    setter_calls++;
}
static int fake_signals(void) { return 0; }

static int update(void) {
    last_poll_ns = 0;
    return signals_hook();
}

int main(void) {
    const unsigned char id[16] = {
        0x54, 0x32, 0x3d, 0xa7, 0x52, 0x60, 0x04, 0xb9,
        0xe5, 0xc3, 0xd7, 0x5d, 0x17, 0x25, 0x39, 0xe5
    };
    const size_t image_size = 0x6ac8000;
    image = mmap(NULL, image_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    assert(image != MAP_FAILED);
    unsigned long base = (unsigned long)image - 0x218000UL;
    void **init_slot = (void **)(base + 0x6ccf6f8UL);
    void **done_slot = (void **)(base + 0x6ccf708UL);
    void **signals_slot = (void **)(base + 0x6cdff50UL);
    void **visible_slot = (void **)(image + 0x6a06268 + 0xd8);
    memcpy(image + 0x228, id, sizeof(id));
    *init_slot = (void *)(base + 0x79d4b0UL);
    *done_slot = (void *)(base + 0x79d650UL);
    *visible_slot = (void *)(base + 0x72123cUL);
    *signals_slot = (void *)fake_signals;
    *(unsigned int *)(base + 0x968a58UL) = 0x97f2ef64U;
    image[0x228] ^= 1;
    assert(natartos_mod_init(fake_dlsym) == 0);
    assert(*init_slot == (void *)(base + 0x79d4b0UL));
    image[0x228] ^= 1;
    assert(natartos_mod_init(fake_dlsym) == 1);
    assert(*init_slot == (void *)layer_init_hook);
    assert(*done_slot == (void *)layer_done_hook);
    assert(*visible_slot == (void *)visibility_hook);
    assert(*signals_slot == (void *)signals_hook);
    assert(munmap(image, image_size) == 0);

    real_layer_init = fake_init;
    real_layer_done = fake_done;
    real_visibility = fake_visibility;
    real_signals = fake_signals;
    layer_init_hook(layer, 0, 0);

    int fd = open(EYEPIECE_BACKDROP_PATH, O_CREAT | O_EXCL | O_RDWR, 0600);
    assert(fd >= 0);
    assert(flock(fd, LOCK_EX | LOCK_NB) == 0);
    eyepiece_backdrop_request request = { 101, 42 };
    assert(pwrite(fd, &request, sizeof(request), 0) == sizeof(request));
    assert(update() == 1);
    assert((*(unsigned int *)(layer + 0x40) & 1) == 0);
    assert(pread(fd, &request, sizeof(request), 0) == sizeof(request));
    assert(request.acknowledged == 101);

    visibility_hook(layer, 0);
    visibility_hook(layer, 1);
    assert((*(unsigned int *)(layer + 0x40) & 1) == 0);
    int before = setter_calls;
    assert(update() == 0 && setter_calls == before);

    layer_done_hook(layer);
    request.request = 102;
    request.acknowledged = 0;
    assert(pwrite(fd, &request, sizeof(request), 0) == sizeof(request));
    assert(update() == 0);
    assert(pread(fd, &request, sizeof(request), 0) == sizeof(request));
    assert(request.acknowledged == 0);
    layer_init_hook(layer, 0, 0);
    assert((*(unsigned int *)(layer + 0x40) & 1) == 0);
    assert(update() == 1);
    assert(pread(fd, &request, sizeof(request), 0) == sizeof(request));
    assert(request.acknowledged == 102);

    close(fd);
    assert(update() == 1);
    assert((*(unsigned int *)(layer + 0x40) & 1) == 1);

    int ready[2];
    assert(pipe(ready) == 0);
    pid_t child = fork();
    assert(child >= 0);
    if (!child) {
        close(ready[0]);
        int lock_fd = open(EYEPIECE_BACKDROP_PATH, O_RDWR);
        assert(lock_fd >= 0 && flock(lock_fd, LOCK_EX | LOCK_NB) == 0);
        request = (eyepiece_backdrop_request){ 103, 0 };
        assert(pwrite(lock_fd, &request, sizeof(request), 0) == sizeof(request));
        assert(write(ready[1], "x", 1) == 1);
        for (;;) pause();
    }
    close(ready[1]);
    char byte;
    assert(read(ready[0], &byte, 1) == 1);
    close(ready[0]);
    assert(update() == 1);
    visibility_hook(layer, 0);
    assert(kill(child, SIGKILL) == 0);
    assert(waitpid(child, 0, 0) == child);
    assert(update() == 1);
    assert((*(unsigned int *)(layer + 0x40) & 1) == 0);
    assert(unlink(EYEPIECE_BACKDROP_PATH) == 0);
    return 0;
}
