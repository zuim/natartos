/*
 * hook_overlay.so — display overlay + screencap + health signal for DJI Goggles 3.
 *
 * GOT-patches duss_hal_display_push_frame in libdisp_eng.so.
 * Provides:
 *   - Frame injection via fb_shared (Doom, PC Stream)
 *   - DMA-buf screencap (SIGWINCH → /data/local/tmp/screencap.bin)
 *   - Health signal for loader anti-brick
 *   - ioctl EVIOCGRAB block (so input mod can work)
 */

#define NTLOG_PREFIX "overlay"
#include "natartos_common.h"
#include "gui_alpha.h"

#define sys_write       sys_write_raw
#define my_strlen       natartos_strlen
#define my_strncmp      natartos_strncmp
#define my_strcpy       natartos_strcpy
#define my_memcpy       natartos_memcpy

#define SIGUSR1    10
#define SIGURG     23
#define SIGWINCH   28

#define PROT_EXEC   4

#define __NR_rt_sigaction 134
#define __NR_mmap       222
#define __NR_ftruncate   46
#define __NR_pread64     67
#define __NR_pwrite64    68
#define __NR_munmap     215
#define MS_INVALIDATE   2
#define __NR_msync      227
#define __NR_ioctl       29
#define EVIOCGRAB_VAL    0x40044590
#define __NR_clock_gettime 113

#define MAP_SHARED  1
#define MAP_PRIVATE 2
#define MAP_ANONYMOUS 0x20
#define MAP_FAILED  ((void *)-1)
#define CLOCK_MONOTONIC 1

#define EV_KEY   1
#define KEY_BACK 158
/* omc_keys directions; the EW input manager (system/etc/gui/zv902_gui_input.json)
 * maps the same Linux codes, so they reach dji_glasses' scene. */
#define KEY_UP    103
#define KEY_DOWN  108
#define KEY_LEFT  105
#define KEY_RIGHT 106
#define EVIOCGNAME_64 0x80404506UL   /* _IOC(_IOC_READ, 'E', 0x06, 64) */
struct input_event_t { long sec, usec; unsigned short type, code; int value; };

#define FRAME_SIZE 8294400L
static const char MAPS_PATH[] = "/proc/self/maps";

/* ---- Extra syscalls ---- */

static long sys_pread64(int fd, void *buf, long count, long offset) {
    register long x0 __asm__("x0") = fd;
    register long x1 __asm__("x1") = (long)buf;
    register long x2 __asm__("x2") = count;
    register long x3 __asm__("x3") = offset;
    register long x8 __asm__("x8") = __NR_pread64;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x3), "r"(x8) : "memory");
    return x0;
}

static long sys_pwrite64(int fd, const void *buf, long count, long offset) {
    register long x0 __asm__("x0") = fd;
    register long x1 __asm__("x1") = (long)buf;
    register long x2 __asm__("x2") = count;
    register long x3 __asm__("x3") = offset;
    register long x8 __asm__("x8") = __NR_pwrite64;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x3), "r"(x8) : "memory");
    return x0;
}

static void *sys_mmap(void *addr, long len, int prot, int flags, int fd, long offset) {
    register long x0 __asm__("x0") = (long)addr;
    register long x1 __asm__("x1") = len;
    register long x2 __asm__("x2") = prot;
    register long x3 __asm__("x3") = flags;
    register long x4 __asm__("x4") = fd;
    register long x5 __asm__("x5") = offset;
    register long x8 __asm__("x8") = __NR_mmap;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x3), "r"(x4), "r"(x5), "r"(x8) : "memory");
    return (void *)x0;
}

static long sys_ioctl(int fd, unsigned long req, unsigned long arg) {
    register long x0 __asm__("x0") = fd;
    register long x1 __asm__("x1") = req;
    register long x2 __asm__("x2") = arg;
    register long x8 __asm__("x8") = __NR_ioctl;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x8) : "memory");
    return x0;
}

static long sys_clock_gettime(int clk, struct timespec_t *tp) {
    register long x0 __asm__("x0") = clk;
    register long x1 __asm__("x1") = (long)tp;
    register long x8 __asm__("x8") = __NR_clock_gettime;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x8) : "memory");
    return x0;
}

static long sys_rt_sigaction(int sig, const void *act, void *oldact, long sigsetsize) {
    register long x0 __asm__("x0") = sig;
    register long x1 __asm__("x1") = (long)act;
    register long x2 __asm__("x2") = (long)oldact;
    register long x3 __asm__("x3") = sigsetsize;
    register long x8 __asm__("x8") = __NR_rt_sigaction;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x3), "r"(x8) : "memory");
    return x0;
}

/* ---- Helpers ---- */

static unsigned long parse_hex(const char **p) {
    unsigned long v = 0;
    while (1) {
        char c = **p;
        if (c >= '0' && c <= '9') v = (v << 4) | (c - '0');
        else if (c >= 'a' && c <= 'f') v = (v << 4) | (c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') v = (v << 4) | (c - 'A' + 10);
        else break;
        (*p)++;
    }
    return v;
}

static void fmt_ptr(char *out, unsigned long v) {
    out[0] = '0'; out[1] = 'x';
    for (int i = 15; i >= 0; i--) {
        int n = (v >> (i * 4)) & 0xf;
        out[2 + (15 - i)] = "0123456789abcdef"[n];
    }
    out[18] = 0;
}

static void fmt_dec(char *out, int v) {
    if (v < 0) { *out++ = '-'; v = -v; }
    char tmp[16]; int i = 0;
    if (v == 0) { *out++ = '0'; *out = 0; return; }
    while (v > 0) { tmp[i++] = '0' + (v % 10); v /= 10; }
    while (i > 0) *out++ = tmp[--i];
    *out = 0;
}

static void fmt_u32(char *out, unsigned int v) {
    out[0] = '0'; out[1] = 'x';
    for (int i = 7; i >= 0; i--) {
        int n = (v >> (i * 4)) & 0xf;
        out[2 + (7 - i)] = "0123456789abcdef"[n];
    }
    out[10] = 0;
}

/* ---- Signal structs ---- */

struct kernel_sigaction {
    void (*handler)(int);
    unsigned long flags;
    void (*restorer)(void);
    unsigned long mask[2];
};
#define SA_RESTART 0x10000000

/* ---- Globals ---- */

__attribute__((weak)) void *dlsym(void *, const char *);
#define RTLD_NEXT    ((void *)-1)
#define RTLD_DEFAULT ((void *)0)

typedef void *(*dlsym_fn)(void *, const char *);
static dlsym_fn g_dlsym = 0;

volatile unsigned int natartos_health_signal = 0;

typedef int (*push_frame_fn_t)(void *, int, void *);
static push_frame_fn_t g_real_push_frame;

/* ---- Inline patch helpers ---- */

static void flush_icache(void *addr, long size) {
    unsigned long p = (unsigned long)addr & ~63UL;
    unsigned long end = (unsigned long)addr + size;
    for (; p < end; p += 64) {
        __asm__ volatile("dc cvau, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb ish" ::: "memory");
    p = (unsigned long)addr & ~63UL;
    for (; p < end; p += 64) {
        __asm__ volatile("ic ivau, %0" :: "r"(p) : "memory");
    }
    __asm__ volatile("dsb ish" ::: "memory");
    __asm__ volatile("isb" ::: "memory");
}

/*
 * Entry-point detour with a SINGLE-INSTRUCTION atomic patch.
 *
 * The real push_frame is a tiny leaf dispatcher whose first instruction
 * (ldr x8,[x0,#0x10]) is position-independent. We overwrite only that one
 * 4-byte word with `B <veneer>`. A naturally-aligned 32-bit store is fetched
 * atomically by every core, so a concurrent render thread can never observe a
 * torn instruction. The target is patched ONCE and never rewritten — unlike
 * the old per-frame restore/call/re-patch, which rewrote 16 live bytes 60×/s
 * and SIGSEGV'd dji_glasses when the render thread raced the rewrite.
 *
 *   veneer     : LDR X16,[PC,#8]; BR X16; .quad hook       (placed near the
 *                target so the single `B` can reach it)
 *   call-orig  : <orig insn0>; LDR X16,[PC,#8]; BR X16; .quad target+4
 */
static void *g_patch_target;
static unsigned int g_orig_insn0;

static long sys_munmap(void *addr, long len) {
    register long x0 __asm__("x0") = (long)addr;
    register long x1 __asm__("x1") = len;
    register long x8 __asm__("x8") = __NR_munmap;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x8) : "memory");
    return x0;
}

/* An RWX-then-RX page within ±112MB of `near`, so a `B` reaches it. 0 on fail. */
static void *map_stub_near(unsigned long near) {
    static const long deltas[] = {
        -0x200000L, 0x200000L, -0x2000000L, 0x2000000L, -0x6000000L, 0x6000000L
    };
    for (int i = 0; i < 6; i++) {
        unsigned long hint = (near + deltas[i]) & ~0xFFFUL;
        void *p = sys_mmap((void *)hint, 0x1000, PROT_READ | PROT_WRITE,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        if (p == MAP_FAILED) continue;
        long off = (long)((unsigned long)p - near);
        if (off > -0x7000000L && off < 0x7000000L) return p;
        sys_munmap(p, 0x1000);
    }
    return 0;
}

/* Install the detour; returns the call-original trampoline, or 0 on failure
 * (caller then falls back to LD_PRELOAD interposition only — never a crash). */
static push_frame_fn_t install_hook(void *target, void *hook) {
    unsigned int *t = (unsigned int *)target;
    unsigned int insn0 = t[0];

    void *stub = map_stub_near((unsigned long)target);
    if (!stub) { NTLOG("hook: no stub page in range"); return 0; }

    unsigned int *s = (unsigned int *)stub;
    int i = 0;

    /* call-original trampoline: replay insn0, then continue at target+4 */
    push_frame_fn_t call_orig = (push_frame_fn_t)&s[i];
    s[i++] = insn0;
    s[i++] = 0x58000050;                     /* ldr x16,[pc,#8] */
    s[i++] = 0xD61F0200;                     /* br  x16         */
    *(unsigned long *)&s[i] = (unsigned long)target + 4; i += 2;

    /* entry veneer: reach our hook anywhere in the address space */
    unsigned int *veneer = &s[i];
    s[i++] = 0x58000050;                     /* ldr x16,[pc,#8] */
    s[i++] = 0xD61F0200;                     /* br  x16         */
    *(unsigned long *)&s[i] = (unsigned long)hook; i += 2;

    sys_mprotect(stub, 0x1000, PROT_READ | PROT_EXEC);
    flush_icache(stub, i * 4);

    long off = (long)((unsigned long)veneer - (unsigned long)target);
    if (off <= -0x8000000L || off >= 0x8000000L) { NTLOG("hook: veneer out of B range"); return 0; }
    unsigned int b_insn = 0x14000000u | ((unsigned int)(off >> 2) & 0x03FFFFFFu);

    unsigned long page = (unsigned long)target & ~0xFFFUL;
    if (sys_mprotect((void *)page, 0x2000, PROT_READ | PROT_WRITE | PROT_EXEC) < 0) {
        NTLOG("hook: mprotect target failed"); return 0;
    }
    g_patch_target = target;
    g_orig_insn0 = insn0;
    __atomic_store_n(t, b_insn, __ATOMIC_RELEASE);   /* single aligned store */
    flush_icache(target, 4);

    return call_orig;
}

/* ---- DMA-buf framebuffers ---- */

#define MAX_FRAMEBUFFERS 4
static void *g_framebuffers[MAX_FRAMEBUFFERS];
static int g_fb_count = 0;

static int find_all_framebuffers(void) {
    int fd = (int)sys_openat(AT_FDCWD, MAPS_PATH, O_RDONLY, 0);
    if (fd < 0) return 0;

    void *local_fbs[MAX_FRAMEBUFFERS];
    int local_count = 0;
    char buf[512];
    int buf_len = 0;

    while (1) {
        if (buf_len < (int)sizeof(buf) - 1) {
            long got = sys_read(fd, buf + buf_len, sizeof(buf) - 1 - buf_len);
            if (got <= 0 && buf_len == 0) break;
            if (got > 0) buf_len += got;
        }
        buf[buf_len] = 0;

        int nl = -1;
        for (int i = 0; i < buf_len; i++) {
            if (buf[i] == '\n') { nl = i; break; }
        }
        if (nl < 0) {
            if (buf_len >= (int)sizeof(buf) - 1) { buf_len = 0; continue; }
            break;
        }

        int has_dm = 0;
        for (int i = 0; i + 5 <= nl; i++) {
            if (buf[i]=='d' && buf[i+1]=='m' && buf[i+2]=='a' &&
                buf[i+3]=='b' && buf[i+4]=='u' && buf[i+5]=='f') { has_dm = 1; break; }
        }

        if (has_dm) {
            const char *p = buf;
            unsigned long start = parse_hex(&p);
            if (*p == '-') p++;
            unsigned long end = parse_hex(&p);
            if ((long)(end - start) == FRAME_SIZE && local_count < MAX_FRAMEBUFFERS)
                local_fbs[local_count++] = (void *)start;
        }

        int remaining = buf_len - nl - 1;
        for (int i = 0; i < remaining; i++) buf[i] = buf[nl + 1 + i];
        buf_len = remaining;
    }
    sys_close(fd);

    for (int i = 0; i < local_count; i++)
        g_framebuffers[i] = local_fbs[i];
    __atomic_store_n(&g_fb_count, local_count, __ATOMIC_RELEASE);
    return local_count;
}

/* The inject thread paints app frames straight into the GUI's own scanout
 * buffers. The GUI repaints only what changed, so after an app exits the rest
 * of its last frame stayed visible until enough of the UI was redrawn. Save the
 * GUI's buffers before the first app frame and put them back on exit (anonymous
 * memory, only while an app is shown). */
static void *g_saved_fb[MAX_FRAMEBUFFERS];
static int g_saved_count = 0;

static void save_gui_framebuffers(int fbc) {
    if (g_saved_count) return;
    for (int i = 0; i < fbc; i++) {
        void *m = sys_mmap(0, FRAME_SIZE, PROT_READ | PROT_WRITE,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        if ((long)m < 0 && (long)m > -4096) {
            NTLOG_INT("fbsave: mmap failed, saved=", i);
            break;
        }
        my_memcpy(m, g_framebuffers[i], FRAME_SIZE);
        g_saved_fb[i] = m;
        g_saved_count = i + 1;
    }
    NTLOG_INT("fbsave: saved GUI buffers=", g_saved_count);
}

static void restore_gui_framebuffers(void) {
    int n = g_saved_count;
    for (int i = 0; i < n; i++) {
        my_memcpy(g_framebuffers[i], g_saved_fb[i], FRAME_SIZE);
        sys_munmap(g_saved_fb[i], FRAME_SIZE);
        g_saved_fb[i] = 0;
    }
    g_saved_count = 0;
    if (n) NTLOG_INT("fbsave: restored GUI buffers=", n);
}

/* ---- Screencap ---- */

static void write_status(const char *msg) {
    static const char LOG[] = "/data/local/tmp/screencap.log";
    int fd = (int)sys_openat(AT_FDCWD, LOG, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd >= 0) { sys_write(fd, msg, my_strlen(msg)); sys_close(fd); }
}

static int fb_is_blank(void *fb) {
    unsigned long *p = (unsigned long *)fb;
    for (int i = 0; i < 64; i++) { if (p[i]) return 0; }
    return 1;
}

static void do_capture(void) {
    int count = find_all_framebuffers();
    if (count == 0) { NTLOG("capture: no fb"); write_status("FAIL"); return; }

    /* Pick the first non-blank buffer (back-buffer may be empty) */
    int idx = 0;
    for (int i = 0; i < count; i++) {
        if (!fb_is_blank(g_framebuffers[i])) { idx = i; break; }
    }

    static const char OUT[] = "/data/local/tmp/screencap.bin";
    int ofd = (int)sys_openat(AT_FDCWD, OUT, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (ofd < 0) { NTLOG("capture: open fail"); write_status("FAIL"); return; }
    long written = 0;
    while (written < FRAME_SIZE) {
        long chunk = FRAME_SIZE - written;
        if (chunk > 65536) chunk = 65536;
        long w = sys_write(ofd, (char *)g_framebuffers[idx] + written, chunk);
        if (w <= 0) break;
        written += w;
    }
    sys_close(ofd);
    if (written == FRAME_SIZE) {
        NTLOG("capture: OK"); write_status("OK");
    } else {
        NTLOG("capture: short"); write_status("FAIL");
    }
}

static void do_fb_write(void) {
    int count = find_all_framebuffers();
    if (count == 0) return;
    static const char IN[] = "/data/local/tmp/fb_write.bin";
    int ifd = (int)sys_openat(AT_FDCWD, IN, O_RDONLY, 0);
    if (ifd < 0) return;
    long total = 0;
    while (total < FRAME_SIZE) {
        long chunk = FRAME_SIZE - total;
        if (chunk > 65536) chunk = 65536;
        long r = sys_read(ifd, (char *)g_framebuffers[0] + total, chunk);
        if (r <= 0) break;
        total += r;
    }
    sys_close(ifd);
    if (total != FRAME_SIZE) return;
    for (int i = 1; i < count; i++) my_memcpy(g_framebuffers[i], g_framebuffers[0], FRAME_SIZE);
    NTLOG("fb_write: OK");
}

static void sigwinch_handler(int sig) { (void)sig; do_capture(); }
static void sigurg_handler(int sig) { (void)sig; do_fb_write(); }

/* ---- ioctl intercept: block EVIOCGRAB ---- */

int ioctl(int fd, unsigned long request, unsigned long arg) {
    if (request == EVIOCGRAB_VAL) return 0;
    return (int)sys_ioctl(fd, request, arg);
}

/* ---- Frame injection ---- */

typedef int (*pthread_create_fn)(unsigned long *, const void *, void *(*)(void *), void *);

#define SHM_HEADER_G 64
static int g_inject_fd = -1;
static volatile unsigned int g_inject_last_counter = 0;
static volatile int g_inject_active = 0;
static volatile int g_inject_frames = 0;

/* experiment instrumentation (used by cleanup + hook) */
static volatile int g_exited_once = 0;
static volatile int g_post_exit_pushes = 0;

static int g_push_count = 0;

/* The GUI frame swallowed most recently while injecting, per plane. The GUI
 * does not render again until a pushed frame is handed to the display, so a
 * frame it rendered during the overlay (e.g. navigating on the same Back press
 * that exits an app) leaves it stalled after the overlay stops: no input wakes
 * it. overlay_cleanup submits these to the real push_frame once to release it. */
#define PENDING_SLOTS 8
static void *volatile g_pending_handle[PENDING_SLOTS];
static void *volatile g_pending_buf[PENDING_SLOTS];

static void start_inject_thread(void);

/* GUI display handle and plane, from the GUI's own push_frame calls (for the
 * fb_gui_hide blending switch below). */
static void *volatile g_gui_handle;
static volatile int g_gui_plane = -1;

/* Write one key press+release (with SYN reports) to an evdev fd. */
static void emit_key(int fd, unsigned short code) {
    struct input_event_t ev[4] = {
        { 0, 0, EV_KEY, code, 1 },
        { 0, 0, 0 /*EV_SYN*/, 0, 0 },
        { 0, 0, EV_KEY, code, 0 },
        { 0, 0, 0, 0, 0 },
    };
    sys_write(fd, &ev[0], sizeof(ev[0]));
    sys_write(fd, &ev[1], sizeof(ev[1]));
    struct timespec_t t = { 0, 40000000 };   /* 40ms hold */
    sys_nanosleep(&t, 0);
    sys_write(fd, &ev[2], sizeof(ev[2]));
    sys_write(fd, &ev[3], sizeof(ev[3]));
}

/* Open an input device by name, e.g. "omc_keys". The GUI input config selects
 * devices by name (system/etc/gui/zv902_gui_input.json), while event numbers
 * are assigned by probe order and can change between firmware builds. */
static int open_event_by_name(const char *want) {
    static const char tmpl[] = "/dev/input/event0";
    char path[sizeof(tmpl)];
    my_memcpy(path, tmpl, sizeof(tmpl));
    for (int n = 0; n < 10; n++) {
        path[sizeof(tmpl) - 2] = (char)('0' + n);
        int fd = (int)sys_openat(AT_FDCWD, path, O_RDONLY, 0);
        if (fd < 0) continue;
        char name[64];
        long r = sys_ioctl(fd, EVIOCGNAME_64, (unsigned long)name);
        sys_close(fd);
        if (r <= 0) continue;
        name[sizeof(name) - 1] = 0;
        if (my_strncmp(name, want, sizeof(name)) == 0)
            return (int)sys_openat(AT_FDCWD, path, O_WRONLY, 0);
    }
    return -1;
}

/* Force a UI repaint after an app exits. The GUI is a reactive EW scene that
 * only paints on invalidation, so we feed it input events.
 *
 * Do NOT use ENTER: on the menu the just-exited app is the highlighted item
 * and ENTER re-launches it (observed: Doom relaunched, PC Stream bounced to
 * restart). BACK is safe but only invalidates when there is something to go
 * back from. Apps that do not grab event0 (e.g. the eyepiece bridge) let
 * their own Back press navigate the hidden GUI home, so a BACK sent here is a
 * no-op (the UI beeps), no frame is pushed, and the old code fell back to
 * restarting dji_glasses on every exit.
 *
 * Directions (omc_keys) move the focus on any screen — home included — and
 * never activate an item. Send each pair down+up / right+left so the
 * selection is left where it was. Retry until the GUI pushes a frame
 * (g_post_exit_pushes advances); only then fall back to BACK and, if nothing
 * wakes it, to the restart. */
static int inject_ui_repaint(void) {
    int back = (int)sys_openat(AT_FDCWD, "/dev/input/event0", O_WRONLY, 0);
    int nav  = open_event_by_name("omc_keys");
    if (back < 0 && nav < 0) { NTLOG("repaint: no input device"); return 0; }

    struct timespec_t settle = { 0, 150000000 };   /* app's input device closed */
    sys_nanosleep(&settle, 0);

    /* If the GUI already repainted on its own (happens right after a fresh
     * dji_glasses start), leave it be — don't navigate. */
    struct timespec_t probe = { 0, 300000000 };
    sys_nanosleep(&probe, 0);
    if (g_post_exit_pushes > 0) {
        if (back >= 0) sys_close(back);
        if (nav >= 0) sys_close(nav);
        NTLOG("repaint: self-recovered");
        return 1;
    }

    struct timespec_t wait = { 0, 200000000 };
    unsigned int before = (unsigned int)g_post_exit_pushes;
    int woke = 0;

    if (nav >= 0) {
        static const unsigned short pairs[2][2] = {
            { KEY_DOWN, KEY_UP }, { KEY_RIGHT, KEY_LEFT }
        };
        for (int p = 0; p < 2 && !woke; p++) {
            for (int tries = 0; tries < 2 && !woke; tries++) {
                before = (unsigned int)g_post_exit_pushes;
                emit_key(nav, pairs[p][0]);
                sys_nanosleep(&wait, 0);
                emit_key(nav, pairs[p][1]);   /* restore the selection */
                sys_nanosleep(&wait, 0);
                if (g_post_exit_pushes != before) woke = 1;
            }
        }
    }

    /* Only if no direction woke the scene: BACK can still close an open menu. */
    if (!woke && back >= 0) {
        for (int tries = 0; tries < 2 && !woke; tries++) {
            before = (unsigned int)g_post_exit_pushes;
            emit_key(back, KEY_BACK);
            sys_nanosleep(&wait, 0);
            if (g_post_exit_pushes != before) woke = 1;
        }
    }

    if (back >= 0) sys_close(back);
    if (nav >= 0) sys_close(nav);
    NTLOG_INT("repaint: woke=", woke);
    return woke;
}

/* Set while overlay_cleanup runs (it sleeps ~2 s probing the GUI). The
 * watcher must not start the next app's session meanwhile: the cleanup's
 * input pokes and repaint probe would hit the new session (a newly started
 * app that grabs the keys received the fallback BACK and exited). */
static volatile int g_cleanup_busy = 0;

static void overlay_cleanup_inner(void);
#define CLEANUP_FLAG "/data/local/tmp/fb_cleanup"   /* apps wait while it exists */
static long sys_unlinkat(int dirfd, const char *path, int flags) {
    register long x0 __asm__("x0") = dirfd;
    register long x1 __asm__("x1") = (long)path;
    register long x2 __asm__("x2") = flags;
    register long x8 __asm__("x8") = 35;   /* __NR_unlinkat (arm64) */
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x2), "r"(x8) : "memory");
    return x0;
}
static void overlay_cleanup(void) {
    __atomic_store_n(&g_cleanup_busy, 1, __ATOMIC_RELEASE);
    int fd = (int)sys_openat(AT_FDCWD, CLEANUP_FLAG, O_WRONLY | O_CREAT, 0666);
    if (fd >= 0) sys_close(fd);
    overlay_cleanup_inner();
    sys_unlinkat(AT_FDCWD, CLEANUP_FLAG, 0);
    __atomic_store_n(&g_cleanup_busy, 0, __ATOMIC_RELEASE);
}

static void overlay_cleanup_inner(void) {
    NTLOG("cleanup: stopping overlay");
    g_inject_active = 0;
    if (g_inject_fd >= 0) { sys_close(g_inject_fd); g_inject_fd = -1; }
    g_inject_last_counter = 0;   /* reset so the next launch starts clean */

    /* NOTE: do NOT zero the DMA-bufs here — the display controller does not
     * refill a zeroed buffer on its own after inject stops, which left a black
     * screen on exit. Leaving the last frame in place lets the resuming real
     * push_frame overwrite it with live UI (or, worst case, a frozen last
     * frame — still better than black). */

    typedef int (*system_fn)(const char *);
    system_fn sys_fn = g_dlsym ? (system_fn)g_dlsym(RTLD_DEFAULT, "system") : 0;

    if (!sys_fn) { NTLOG("cleanup: no system()"); return; }

    /* Kill the app and BLOCK until it is really gone: the app runs in its own
     * process and holds an EVIOCGRAB on the input device, so until it dies our
     * synthetic input is swallowed instead of reaching the GUI. */
    sys_fn("kill -9 $(pidof doom_goggles) 2>/dev/null; "
           "kill -9 $(pidof fb_receiver) 2>/dev/null; "
           "rm -f /data/local/tmp/fb_override /data/local/tmp/fb_shared "
           "/data/local/tmp/fb_stop; "
           "i=0; while pidof doom_goggles >/dev/null 2>&1 && [ $i -lt 40 ]; "
           "do usleep 25000; i=$((i+1)); done");
    g_post_exit_pushes = 0;
    g_exited_once = 1;

    restore_gui_framebuffers();

    /* Hand the GUI's held-back frames to the display (see g_pending_buf). */
    int replayed = 0;
    for (int i = 0; i < PENDING_SLOTS && g_real_push_frame; i++) {
        /* load + store, not exchange: an exchange compiles to a libgcc
         * outline-atomic call (__aarch64_swp8_*) that -nostdlib can't resolve,
         * and a preload with an undefined symbol kills every process. */
        void *buf = __atomic_load_n(&g_pending_buf[i], __ATOMIC_ACQUIRE);
        if (!buf) continue;
        __atomic_store_n(&g_pending_buf[i], (void *)0, __ATOMIC_RELEASE);
        g_real_push_frame(g_pending_handle[i], i, buf);
        replayed++;
    }
    NTLOG_INT("cleanup: replayed held frames=", replayed);

    /* Fast path: force the GUI to re-render the now-unoccluded home UI in
     * process. The compositor only paints on scene invalidation and the app's
     * exit invalidates nothing, so we synthesize real input on the GUI's own
     * event thread (in-band, thread-safe — unlike the EW cross-thread call that
     * hung): ENTER wakes/opens the menu, BACK returns to a clean home. ~1-2s. */
    if (inject_ui_repaint()) { NTLOG("cleanup: clean in-process exit OK"); return; }

    /* Fallback: the poke didn't wake the GUI (input-delivery timing is not 100%
     * reliable). Restart dji_glasses — slower (~10s) but always restores the UI,
     * so a failed poke never leaves a frozen screen. inject.sh treats the
     * natartos_restart flag as an intentional (non-crash) respawn. */
    NTLOG("cleanup: poke failed -> restart fallback");
    sys_fn("touch /data/local/tmp/natartos_restart; "
           "(sleep 1; kill $(pidof dji_glasses)) >/dev/null 2>&1 &");
    NTLOG("cleanup: done");
}

static void *inject_thread_fn(void *arg) {
    (void)arg;
    NTLOG("inject thr: start");
    struct timespec_t ts = { 0, 16666666 };
    int logged_fb = 0;

    while (1) {
        sys_nanosleep(&ts, 0);
        if (!g_inject_active) break;
        if (g_inject_fd < 0) continue;

        /* Stop promptly — checked every frame, so exit latency is <100ms:
         *   fb_stop present  → explicit stop (web / BACK button / SIGUSR1)
         *   fb_override gone → the streaming app exited on its own */
        if (sys_faccessat(AT_FDCWD, "/data/local/tmp/fb_stop", 0) == 0) {
            NTLOG("inject thr: fb_stop");
            overlay_cleanup();
            break;
        }
        if (sys_faccessat(AT_FDCWD, "/data/local/tmp/fb_override", 0) != 0) {
            NTLOG("inject thr: fb_override gone");
            overlay_cleanup();
            break;
        }

        int fbc = __atomic_load_n(&g_fb_count, __ATOMIC_ACQUIRE);
        if (fbc == 0) {
            find_all_framebuffers();
            fbc = g_fb_count;
            if (fbc == 0) continue;
        }

        unsigned int counter = 0;
        sys_pread64(g_inject_fd, &counter, 4, 0);
        /* Only lift a frame when the producer published a NEW one. Without this
         * the thread re-copied ~8MB + fan-out every 16.6ms (60Hz) even when
         * fb_receiver sent nothing, wasting ~2GB/s and widening the tear window. */
        if (counter == 0 || counter == g_inject_last_counter) continue;
        g_inject_last_counter = counter;

        /* Read the published FRONT slot (counter & 1); fb_receiver wrote the
         * other slot, so this copy can't catch a frame mid-write. */
        save_gui_framebuffers(fbc);
        long slot_off = SHM_HEADER_G + (long)(counter & 1u) * FRAME_SIZE;
        long total = 0;
        while (total < FRAME_SIZE) {
            long got = sys_pread64(g_inject_fd, (char *)g_framebuffers[0] + total,
                                   FRAME_SIZE - total, slot_off + total);
            if (got <= 0) break;
            total += got;
        }

        for (int i = 1; i < fbc; i++)
            my_memcpy(g_framebuffers[i], g_framebuffers[0], FRAME_SIZE);

        g_inject_frames++;
    }
    NTLOG("inject thr: exit");
    return 0;
}

static void start_inject_thread(void) {
    pthread_create_fn pcreate = g_dlsym ? (pthread_create_fn)g_dlsym(RTLD_DEFAULT, "pthread_create") : 0;
    if (!pcreate) return;
    unsigned long tid;
    pcreate(&tid, 0, inject_thread_fn, 0);
}

/* ---- Watcher thread ---- */

static void *watcher_thread_fn(void *arg) {
    (void)arg;
    static const char OVR[] = "/data/local/tmp/fb_override";
    static const char SHM[] = "/data/local/tmp/fb_shared";
    struct timespec_t ts = { 0, 500000000 };
    NTLOG("watcher: start");
    int cooldown = 0;
    int logged_no_shm = 0;
    while (1) {
        sys_nanosleep(&ts, 0);
        if (g_inject_active || __atomic_load_n(&g_cleanup_busy, __ATOMIC_ACQUIRE)) {
            cooldown = 4; logged_no_shm = 0; continue;
        }
        if (cooldown > 0) { cooldown--; continue; }
        if (sys_faccessat(AT_FDCWD, OVR, 0) != 0) continue;
        int fd = (int)sys_openat(AT_FDCWD, SHM, O_RDONLY, 0);
        if (fd < 0) { if (!logged_no_shm) { NTLOG("watcher: waiting for fb_shared"); logged_no_shm = 1; } continue; }
        NTLOG("watcher: fb_shared opened, starting inject");
        g_inject_fd = fd;
        g_inject_active = 1;
        if (g_fb_count == 0) find_all_framebuffers();
        g_inject_frames = 0;
        g_inject_last_counter = 0;
        start_inject_thread();
    }
    return 0;
}

static void start_watcher_thread(void) {
    pthread_create_fn pcreate = g_dlsym ? (pthread_create_fn)g_dlsym(RTLD_DEFAULT, "pthread_create") : 0;
    if (!pcreate) { NTLOG("watcher: pthread_create not found"); return; }
    unsigned long tid;
    int ret = pcreate(&tid, 0, watcher_thread_fn, 0);
    NTLOG_INT("watcher: pthread_create ret=", ret);
}

/* ---- BACK-button exit: hold BACK to leave an overlay app ----
 *
 * The menu-launched apps (Doom, PC Stream) have no way back to the goggles UI
 * on their own. This thread watches the physical BACK key (gpio_keys, code 158
 * on event0) and, WHILE an app is active (fb_override present), a long hold
 * (>=1.5s) drops fb_stop → the inject thread tears the app down and the UI
 * returns. Short BACK presses still reach the app (Doom USE/ESC) untouched.
 */
#define O_NONBLOCK  0x800
#define BACK_EXIT_MS 1500

static long now_ms_mono(void) {
    struct timespec_t t;
    sys_clock_gettime(CLOCK_MONOTONIC, &t);
    return (long)t.tv_sec * 1000 + t.tv_nsec / 1000000;
}

static void *back_watcher_thread_fn(void *arg) {
    (void)arg;
    NTLOG("back watcher: start");
    struct timespec_t ts = { 0, 50000000 };   /* 50ms idle poll */
    int fd = -1;
    int pressed = 0;
    long press_ms = 0;
    while (1) {
        if (fd < 0) {
            fd = (int)sys_openat(AT_FDCWD, "/dev/input/event0", O_RDONLY | O_NONBLOCK, 0);
            if (fd < 0) { sys_nanosleep(&ts, 0); continue; }
        }
        struct input_event_t ev;
        while (sys_read(fd, &ev, sizeof(ev)) == (long)sizeof(ev)) {
            if (ev.type == EV_KEY && ev.code == KEY_BACK) {
                if (ev.value == 1) { pressed = 1; press_ms = now_ms_mono(); }
                else if (ev.value == 0) { pressed = 0; }
            }
        }
        /* Only fire while an app owns the screen. */
        if (pressed && sys_faccessat(AT_FDCWD, "/data/local/tmp/fb_override", 0) == 0
                    && now_ms_mono() - press_ms >= BACK_EXIT_MS) {
            NTLOG("back watcher: long BACK -> fb_stop");
            int sfd = (int)sys_openat(AT_FDCWD, "/data/local/tmp/fb_stop",
                                      O_WRONLY | O_CREAT | O_TRUNC, 0666);
            if (sfd >= 0) sys_close(sfd);
            pressed = 0;
        }
        sys_nanosleep(&ts, 0);
    }
    return 0;
}

static void start_back_watcher(void) {
    pthread_create_fn pcreate = g_dlsym ? (pthread_create_fn)g_dlsym(RTLD_DEFAULT, "pthread_create") : 0;
    if (!pcreate) return;
    unsigned long tid;
    pcreate(&tid, 0, back_watcher_thread_fn, 0);
}


/*
 * LD_PRELOAD interposition: export this name so the dynamic linker
 * resolves all calls to duss_hal_display_push_frame to our version.
 * The real function is resolved via dlsym(RTLD_NEXT) in mod_init.
 */
int duss_hal_display_push_frame(void *display_handle, int plane_id, void *frame_buf) {
    g_push_count++;
    natartos_health_signal++;

    /* Wake signal for the clean exit: once overlay_cleanup has run, each real
     * push the GUI makes means it has re-rendered → inject_ui_repaint reads
     * this to know the in-process repaint succeeded. */
    if (g_exited_once && !g_inject_active)
        g_post_exit_pushes++;

    if (g_push_count == 1)
        NTLOG("push_frame: HOOK ALIVE");
    if (plane_id >= 0 && plane_id < 8 && g_gui_plane < 0) {
        g_gui_handle = display_handle;
        g_gui_plane = plane_id;
        NTLOG_INT("fb_gui_hide: GUI plane=", plane_id);
    }

    /* While injecting, the inject thread paints the scanout dmabufs directly;
     * skip the real submit so the compositor's own frames don't fight ours
     * (that caused flicker). push_frame returns 0 on success, so 0 is correct. */
    if (g_inject_active) {
        if (plane_id >= 0 && plane_id < PENDING_SLOTS) {
            g_pending_handle[plane_id] = display_handle;
            __atomic_store_n(&g_pending_buf[plane_id], frame_buf, __ATOMIC_RELEASE);
        }
        return 0;
    }
    /* a live push supersedes any frame held back for this plane */
    if (plane_id >= 0 && plane_id < PENDING_SLOTS)
        __atomic_store_n(&g_pending_buf[plane_id], (void *)0, __ATOMIC_RELEASE);

    if (g_real_push_frame)
        return g_real_push_frame(display_handle, plane_id, frame_buf);
    return 0;
}

/* ---- fb_gui_hide: hide the GUI plane at the display, buffers untouched ----
 *
 * For apps that show their picture on their own display plane (eyepiece:
 * VIDEO0 below the GUI). Painting app frames into the GUI's scanout buffers
 * (fb_shared) needs the GUI's picture restored and a forced repaint on exit;
 * the GUI redraws only dirty regions, so leftovers and partly black screens
 * were possible, and the repaint pokes beep. Instead the GUI plane gets
 * global alpha 0 while the app runs: the GUI keeps rendering normally (its
 * frames are not swallowed), so its picture is current when it comes back.
 *
 * Protocol: the app creates /data/local/tmp/fb_gui_hide and holds flock(LOCK_EX)
 * on it for as long as the GUI should stay hidden. The overlay acknowledges
 * with /data/local/tmp/fb_gui_hidden. File removed, or lock free (the app died:
 * the kernel drops the lock) → stock blending restored, ack removed. Polled
 * every 20 ms. The blending must be set from this process: the display
 * driver only lets the plane's owner change it.                          */
#define GUI_HIDE_PATH   "/data/local/tmp/fb_gui_hide"
#define GUI_HIDDEN_PATH "/data/local/tmp/fb_gui_hidden"
#define __NR_flock 32
#define LOCK_SH 1
#define LOCK_NB 4
#define LOCK_UN 8
#define EWOULDBLOCK 11
#define O_RDWR 2

/* duss_hal_display_plane_blending_set config (DispEngine::setBlend layout) */
typedef struct __attribute__((packed)) {
    unsigned char enable, glb_alpha_en, pre_blend, pad0;
    unsigned int  glb_alpha_val;
    unsigned char alg, order, pad1[2];
    unsigned int  pos;
    int hoffset, voffset;
} gui_blend_t;
typedef int (*blending_set_fn)(void *disp, signed char plane, void *cfg);

static long sys_flock(int fd, int op) {
    register long x0 __asm__("x0") = fd;
    register long x1 __asm__("x1") = op;
    register long x8 __asm__("x8") = __NR_flock;
    __asm__ volatile("svc 0" : "+r"(x0) : "r"(x1), "r"(x8) : "memory");
    return x0;
}

static int gui_hide_wanted(void) {
    int fd = (int)sys_openat(AT_FDCWD, GUI_HIDE_PATH, O_RDONLY, 0);
    if (fd < 0) return 0;
    int held = sys_flock(fd, LOCK_SH | LOCK_NB) != 0;   /* owner holds LOCK_EX */
    if (!held) {
        sys_flock(fd, LOCK_UN);
        sys_unlinkat(AT_FDCWD, GUI_HIDE_PATH, 0);         /* stale: owner is gone */
        NTLOG("fb_gui_hide: owner gone, request removed");
    }
    sys_close(fd);
    return held;
}

static int gui_plane_order(int plane) {
    char buf[4096];
    int fd = (int)sys_openat(AT_FDCWD, "/proc/display/plane", O_RDONLY, 0);
    if (fd < 0) return -1;
    long n = sys_read(fd, buf, sizeof(buf));
    sys_close(fd);
    if (n <= 0 || n >= (long)sizeof(buf)) return -1;
    return gui_alpha_blend_order(buf, (int)n, plane);
}

static void *gui_hide_thread_fn(void *arg) {
    (void)arg;
    struct timespec_t ts = { 0, 20000000 };
    blending_set_fn bset = 0;
    int hidden = 0;
    int current_alpha = 255;
    int last_alpha = 255;
    int last_order = -1;
    sys_unlinkat(AT_FDCWD, GUI_HIDDEN_PATH, 0);   /* no ack from a previous instance */
    while (1) {
        sys_nanosleep(&ts, 0);
        int want = gui_hide_wanted();
        int fd = (int)sys_openat(AT_FDCWD, GUI_ALPHA_PATH, O_RDWR, 0);
        gui_alpha_request request = { 0 };
        int requested = 0;
        int valid = 0;
        if (fd >= 0) {
            long lock = sys_flock(fd, LOCK_SH | LOCK_NB);
            if (lock == -EWOULDBLOCK) {
                requested = 1;
                if (sys_pread64(fd, &request, sizeof(request), 0) == sizeof(request) &&
                    request.request && request.alpha <= 255) {
                    valid = 1;
                    last_alpha = (int)request.alpha;
                }
            } else if (lock == 0) {
                sys_flock(fd, LOCK_UN);
            } else {
                NTLOG_INT("gui_alpha: flock failed: ", (int)lock);
                requested = 1;
            }
        } else if (fd != -2) {
            NTLOG_INT("gui_alpha: open failed: ", fd);
            requested = 1;
        }
        if (!requested) last_alpha = 255;
        int alpha = gui_alpha_effective(want, requested, (uint32_t)last_alpha);
        int needs_ack = valid && request.acknowledged != request.request;
        if (want == hidden && alpha == current_alpha && !needs_ack) {
            if (fd >= 0) sys_close(fd);
            continue;
        }
        int plane = g_gui_plane;
        void *h = g_gui_handle;
        if (plane < 0 || !h) { if (fd >= 0) sys_close(fd); continue; }
        if (!bset && g_dlsym)
            bset = (blending_set_fn)g_dlsym(RTLD_DEFAULT, "duss_hal_display_plane_blending_set");
        if (!bset) {
            NTLOG("fb_gui_hide: no blending_set");
            if (fd >= 0) sys_close(fd);
            ts.tv_sec = 1;
            continue;
        }
        int order = gui_plane_order(plane);
        if (order < 0) order = last_order;
        if (order < 0 && !requested) order = plane == 6 ? 1 : 2;
        if (order < 0) {
            NTLOG("gui_alpha: cannot determine GUI plane order");
            if (needs_ack) {
                uint32_t status = 2;
                if (sys_pwrite64(fd, &status, sizeof(status),
                                 __builtin_offsetof(gui_alpha_request, status)) != sizeof(status) ||
                    sys_pwrite64(fd, &request.request, sizeof(request.request),
                                 __builtin_offsetof(gui_alpha_request, acknowledged)) != sizeof(request.request))
                    NTLOG("gui_alpha: acknowledgement write failed");
            }
            if (fd >= 0) sys_close(fd);
            continue;
        }
        last_order = order;
        gui_blend_t b = { 1, (unsigned char)(want || alpha != 255), 0, 0, (unsigned int)alpha, 0,
                          (unsigned char)order, {0, 0}, 0, 0, 0 };
        int r = 0;
        if (want != hidden || alpha != current_alpha || needs_ack) r = bset(h, (signed char)plane, &b);
        if (r != 0) NTLOG_INT("gui_alpha: blending_set failed r=", r);
        if (needs_ack) {
            uint32_t status = (r != 0 || want) ? 2 : 0;
            if (sys_pwrite64(fd, &status, sizeof(status),
                             __builtin_offsetof(gui_alpha_request, status)) != sizeof(status) ||
                sys_pwrite64(fd, &request.request, sizeof(request.request),
                             __builtin_offsetof(gui_alpha_request, acknowledged)) != sizeof(request.request))
                NTLOG("gui_alpha: acknowledgement write failed");
        }
        if (fd >= 0) sys_close(fd);
        if (r != 0) continue;
        current_alpha = alpha;
        if (want == hidden) continue;
        hidden = want;
        if (hidden) {
            int fd = (int)sys_openat(AT_FDCWD, GUI_HIDDEN_PATH, O_WRONLY | O_CREAT | O_TRUNC, 0666);
            if (fd >= 0) sys_close(fd);
            NTLOG("fb_gui_hide: GUI plane hidden");
        } else {
            sys_unlinkat(AT_FDCWD, GUI_HIDDEN_PATH, 0);
            NTLOG("fb_gui_hide: GUI plane restored");
        }
    }
    return 0;
}

static void start_gui_hide_thread(void) {
    pthread_create_fn pcreate = g_dlsym ? (pthread_create_fn)g_dlsym(RTLD_DEFAULT, "pthread_create") : 0;
    if (!pcreate) return;
    unsigned long tid;
    pcreate(&tid, 0, gui_hide_thread_fn, 0);
}

/* ---- Signal handlers ---- */

static void sigusr1_handler(int sig) {
    (void)sig;
    int fd = (int)sys_openat(AT_FDCWD, "/data/local/tmp/fb_stop",
                             O_WRONLY|O_CREAT|O_TRUNC, 0666);
    if (fd >= 0) sys_close(fd);
}

/* ---- Constructor ---- */

__attribute__((constructor))
static void init_hook(void) {
    struct kernel_sigaction sa = { .handler = sigwinch_handler, .flags = SA_RESTART };
    sys_rt_sigaction(SIGWINCH, &sa, 0, 8);

    struct kernel_sigaction sa2 = { .handler = sigurg_handler, .flags = SA_RESTART };
    sys_rt_sigaction(SIGURG, &sa2, 0, 8);

    struct kernel_sigaction sa3 = { .handler = sigusr1_handler, .flags = SA_RESTART };
    sys_rt_sigaction(SIGUSR1, &sa3, 0, 8);

    char exe[256];
    int cmd_fd = (int)sys_openat(AT_FDCWD, "/proc/self/cmdline", O_RDONLY, 0);
    int is_glasses = 0;
    if (cmd_fd >= 0) {
        long r = sys_read(cmd_fd, exe, sizeof(exe) - 1);
        sys_close(cmd_fd);
        if (r > 0) {
            exe[r] = 0;
            for (int i = 0; i + 11 <= r; i++) {
                if (my_strncmp(exe + i, "dji_glasses", 11) == 0) {
                    is_glasses = 1; break;
                }
            }
        }
    }

    if (!is_glasses) return;

    NTLOG("hook_overlay loaded");
}

int natartos_mod_init(void *(*loader_dlsym)(void *, const char *)) {
    g_dlsym = (dlsym_fn)loader_dlsym;

    /* Resolve the REAL push_frame behind our LD_PRELOAD interposition */
    push_frame_fn_t real_fn = (push_frame_fn_t)loader_dlsym(RTLD_NEXT,
                                                              "duss_hal_display_push_frame");
    if (!real_fn) {
        NTLOG("WARN: push_frame not found via RTLD_NEXT");
    } else {
        NTLOG_HEX("real push_frame=", (unsigned long)real_fn);
        /* Atomic single-instruction detour; real called via trampoline. */
        push_frame_fn_t tramp = install_hook((void *)real_fn,
                                             (void *)duss_hal_display_push_frame);
        if (tramp) {
            g_real_push_frame = tramp;
            NTLOG("hook: installed (atomic detour)");
        } else {
            g_real_push_frame = real_fn;   /* LD_PRELOAD interposition only */
            NTLOG("hook: detour unavailable, preload-only");
        }
    }

    NTLOG("init: starting watcher");
    start_watcher_thread();
    start_back_watcher();
    start_gui_hide_thread();
    return 0;
}
