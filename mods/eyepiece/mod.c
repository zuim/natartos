// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * mod_zuim_eyepiece.so — natartos side of Eyepiece (the goggles as a USB PC
 * display with head tracking).
 *
 * Adds the "Eyepiece" entry to the More menu; it launches
 * /vendor/natartos/apps/eyepiece/eyepiece_bridge, which receives H.264/HEVC
 * from the PC over adb-forwarded TCP, shows it, and streams the head pose
 * back. Head tracking comes from the goggles' MCU. The GUI hook suppresses
 * the background layer while the bridge displays video below the GUI.
 */
#define NTLOG_PREFIX "eyepiece"
#include "natartos_common.h"
#include "backdrop.h"

#define O_RDWR 2
#define O_CLOEXEC 02000000
#define LOCK_EX 2
#define LOCK_NB 4
#define EWOULDBLOCK 11
#define __NR_flock 32
#define __NR_pread64 67
#define __NR_pwrite64 68
#define __NR_clock_gettime 113

static long backdrop_flock(int fd, int operation) {
    register long x0 __asm__("x0") = fd;
    register long x1 __asm__("x1") = operation;
    register long x8 __asm__("x8") = __NR_flock;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x8) : "memory");
    return x0;
}

static long backdrop_io(int fd, void *buf, long len, long offset, long nr) {
    register long x0 __asm__("x0") = fd;
    register long x1 __asm__("x1") = (long)buf;
    register long x2 __asm__("x2") = len;
    register long x3 __asm__("x3") = offset;
    register long x8 __asm__("x8") = nr;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x3), "r"(x8) : "memory");
    return x0;
}

static long backdrop_time(struct timespec_t *time) {
    register long x0 __asm__("x0") = 1;
    register long x1 __asm__("x1") = (long)time;
    register long x8 __asm__("x8") = __NR_clock_gettime;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x8) : "memory");
    return x0;
}

typedef void (*layer_init_fn)(void *, void *, void *);
typedef void (*layer_done_fn)(void *);
typedef void (*visibility_fn)(void *, int);
typedef int (*signals_fn)(void);

static layer_init_fn real_layer_init;
static layer_done_fn real_layer_done;
static visibility_fn real_visibility;
static signals_fn real_signals;
static eyepiece_backdrop_state backdrop;
static unsigned long long last_poll_ns;
static long last_open_error;
static int redraw_pending;

static void layer_init_hook(void *layer, void *owner, void *arg) {
    real_layer_init(layer, owner, arg);
    eyepiece_backdrop_init_layer(&backdrop, layer,
                                 (*(unsigned int *)((char *)layer + 0x40)) & 1);
    if (backdrop.suppressed) {
        real_visibility(layer, 0);
        redraw_pending = 1;
    }
}

static void layer_done_hook(void *layer) {
    if (backdrop.layer == layer) backdrop.layer = 0;
    real_layer_done(layer);
}

static void visibility_hook(void *layer, int visible) {
    real_visibility(layer, eyepiece_backdrop_stock_visibility(&backdrop, layer, visible));
}

static int backdrop_update(void) {
    struct timespec_t ts;
    if (backdrop_time(&ts) == 0) {
        unsigned long long now = (unsigned long long)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        if (now >= last_poll_ns && now - last_poll_ns < 20000000ULL) return 0;
        last_poll_ns = now;
    }

    int fd = (int)sys_openat(AT_FDCWD, EYEPIECE_BACKDROP_PATH, O_RDWR | O_CLOEXEC, 0);
    if (fd < 0 && fd != -2 && fd != last_open_error)
        NTLOG_INT("backdrop: request open failed: ", fd);
    last_open_error = fd < 0 ? fd : 0;
    long lock = fd < 0 ? 0 : backdrop_flock(fd, LOCK_EX | LOCK_NB);
    if (lock != 0 && lock != -EWOULDBLOCK) {
        NTLOG_INT("backdrop: flock failed: ", (int)lock);
        sys_close(fd);
        return 0;
    }
    if (lock == -EWOULDBLOCK) {
        eyepiece_backdrop_request request;
        long n = backdrop_io(fd, &request, sizeof(request), 0, __NR_pread64);
        if (n != sizeof(request) || !request.request || !backdrop.layer) {
            sys_close(fd);
            return 0;
        }
        int changed = !backdrop.suppressed;
        if (changed) {
            backdrop.suppressed = 1;
            real_visibility(backdrop.layer, 0);
        }
        if ((*(unsigned int *)((char *)backdrop.layer + 0x40)) & 1) {
            NTLOG("backdrop: layer remained visible");
            sys_close(fd);
            return changed;
        }
        if (request.acknowledged != request.request &&
            backdrop_io(fd, &request.request, sizeof(request.request),
                        __builtin_offsetof(eyepiece_backdrop_request, acknowledged),
                        __NR_pwrite64) != sizeof(request.request))
            NTLOG("backdrop: acknowledgement write failed");
        sys_close(fd);
        return changed;
    }
    if (fd >= 0) sys_close(fd);
    if (!backdrop.suppressed) return 0;
    backdrop.suppressed = 0;
    if (backdrop.layer) {
        real_visibility(backdrop.layer, eyepiece_backdrop_visibility(&backdrop));
        return 1;
    }
    return 0;
}

static int signals_hook(void) {
    int result = real_signals();
    int changed = backdrop_update() | redraw_pending;
    redraw_pending = 0;
    return result | changed;
}

int natartos_mod_init(void *(*loader_dlsym)(void *, const char *)) {
    static const unsigned char build_id[16] = {
        0x54, 0x32, 0x3d, 0xa7, 0x52, 0x60, 0x04, 0xb9,
        0xe5, 0xc3, 0xd7, 0x5d, 0x17, 0x25, 0x39, 0xe5
    };
    typedef struct {
        const char *name;
        void *base;
        const char *symbol;
        void *address;
    } dl_info;
    typedef int (*dladdr_fn)(const void *, dl_info *);
    if (!loader_dlsym) { NTLOG("backdrop: dlsym missing"); return 0; }
    dladdr_fn get_info = (dladdr_fn)loader_dlsym(RTLD_DEFAULT, "dladdr");
    void **vmt = (void **)loader_dlsym(RTLD_DEFAULT, "__vmt_DuBackgroundBackgroundLayer");
    dl_info info;
    if (!get_info || !vmt || !get_info(vmt, &info) || !info.base ||
        (unsigned long)vmt - (unsigned long)info.base != 0x6a06268UL) {
        NTLOG("backdrop: unsupported background layer mapping");
        return 0;
    }
    unsigned long base = (unsigned long)info.base - 0x218000UL;
    const unsigned char *id = (const unsigned char *)info.base + 0x228;
    for (int i = 0; i < 16; i++) {
        if (id[i] != build_id[i]) {
            NTLOG("backdrop: unsupported libew_gls build");
            return 0;
        }
    }
    void **init_slot = (void **)(base + 0x6ccf6f8UL);
    void **done_slot = (void **)(base + 0x6ccf708UL);
    void **signals_slot = (void **)(base + 0x6cdff50UL);
    void **visible_slot = &vmt[0xd8 / sizeof(void *)];
    if (*init_slot != (void *)(base + 0x79d4b0UL) ||
        *done_slot != (void *)(base + 0x79d650UL) ||
        *visible_slot != (void *)(base + 0x72123cUL) ||
        *signals_slot != loader_dlsym(RTLD_DEFAULT, "EwProcessSignals") ||
        !*signals_slot ||
        *(unsigned int *)(base + 0x968a58UL) != 0x97f2ef64U) {
        NTLOG("backdrop: incompatible or already patched GUI");
        return 0;
    }
    real_layer_init = (layer_init_fn)*init_slot;
    real_layer_done = (layer_done_fn)*done_slot;
    real_visibility = (visibility_fn)*visible_slot;
    real_signals = (signals_fn)*signals_slot;

    unsigned long pages[3] = {
        (unsigned long)init_slot & ~(PAGE_SIZE - 1UL),
        (unsigned long)signals_slot & ~(PAGE_SIZE - 1UL),
        (unsigned long)visible_slot & ~(PAGE_SIZE - 1UL)
    };
    int writable = 0;
    while (writable < 3 &&
           sys_mprotect((void *)pages[writable], PAGE_SIZE, PROT_READ | PROT_WRITE) == 0)
        writable++;
    if (writable != 3) {
        NTLOG("backdrop: cannot unprotect GUI hook pages");
        while (writable > 0) {
            writable--;
            if (sys_mprotect((void *)pages[writable], PAGE_SIZE, PROT_READ) < 0)
                NTLOG("backdrop: cannot restore GUI page protection");
        }
        return 0;
    }
    *init_slot = (void *)layer_init_hook;
    *done_slot = (void *)layer_done_hook;
    *visible_slot = (void *)visibility_hook;
    *signals_slot = (void *)signals_hook;
    for (int i = 0; i < 3; i++)
        if (sys_mprotect((void *)pages[i], PAGE_SIZE, PROT_READ) < 0)
            NTLOG("backdrop: cannot restore GUI page protection");
    NTLOG("backdrop: GUI hook installed");
    return 1;
}

int natartos_mod_register(void *(*loader_dlsym)(void *, const char *)) {
    natartos_menu_register_fn reg =
        (natartos_menu_register_fn)loader_dlsym(RTLD_DEFAULT, "natartos_menu_register");
    if (!reg) { NTLOG("menu_register not found"); return 0; }
    natartos_menu_entry e = {
        .id        = "zuim_eyepiece",
        .name      = "Eyepiece",
        .subtitle  = "PC display + head tracking over USB",
        .icon_path = "/vendor/natartos/icons/natartos_icon_stream.rgba",
        .command   = "nohup /vendor/natartos/apps/eyepiece/eyepiece_bridge -b >/dev/null 2>&1 &",
        .priority  = 210,
    };
    reg(&e);
    NTLOG("mod_zuim_eyepiece: menu entry registered");
    return 1;
}
