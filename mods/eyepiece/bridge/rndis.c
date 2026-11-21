// SPDX-License-Identifier: AGPL-3.0-or-later
/*
 * rndis.c — USB network link to the PC (RNDIS) next to adb, for a lower-latency stream
 * than TCP through `adb forward`.
 *
 * The goggles' USB gadget is configfs (g1). With root ADB enabled it runs `acm,adb`
 * (f1 = acm.0, f2 = ffs.adb). RNDIS does not bind together with acm (configfs-gadget:
 * "rndis: can't bind, err -19"); DJI's own rndis config does not include acm either.
 * So: f1 = rndis.0, f2 = ffs.adb. acm.0 is DJI's engineering serial port (dji_engpc),
 * not used by eyepiece.
 *
 * Runtime only (configfs + a process); a reboot returns to the stock USB config. The
 * switch is done once and then kept (no switch back on exit): a bridge that is killed
 * can't leave the USB half-configured, and later sessions don't re-enumerate USB.
 *
 * rndis0 already carries 192.168.60.2/24 (stock). A DHCP server (busybox udhcpd) hands
 * the PC 192.168.60.1 — one lease, no router and no DNS option, so the PC's default
 * route and name resolution stay untouched.
 */
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include "bridge.h"

#define GADGET "/sys/kernel/config/usb_gadget/g1"
#define CFG    GADGET "/configs/c.1"
#define DIR_TMP "/data/local/tmp/eyepiece"
#define DHCP_CONF DIR_TMP "/udhcpd.conf"
#define DHCP_PID  DIR_TMP "/udhcpd.pid"
#define RNDIS_HOST_MAC "02:65:79:65:00:01"   /* PC side, "eye" */
#define RNDIS_DEV_MAC  "02:65:79:65:00:02"   /* goggles side */

static int read_file(const char *p, char *buf, size_t n) {
    int fd = open(p, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    ssize_t r = read(fd, buf, n - 1);
    close(fd);
    if (r < 0) return -1;
    buf[r] = 0;
    while (r > 0 && (buf[r - 1] == '\n' || buf[r - 1] == ' ')) buf[--r] = 0;
    return (int)r;
}

static int write_file(const char *p, const char *s) {
    int fd = open(p, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return -errno;
    ssize_t r = write(fd, s, strlen(s));
    int e = r < 0 ? -errno : 0;
    close(fd);
    return e;
}

/* name of the function an fN link points to ("" if none) */
static void link_func(const char *name, char *out, size_t n) {
    char p[128], t[256];
    snprintf(p, sizeof(p), CFG "/%s", name);
    ssize_t r = readlink(p, t, sizeof(t) - 1);
    out[0] = 0;
    if (r <= 0) return;
    t[r] = 0;
    const char *b = strrchr(t, '/');
    snprintf(out, n, "%s", b ? b + 1 : t);
}

/* current links f1..f9 → function names; returns count */
static int read_links(char funcs[9][32]) {
    int n = 0;
    for (int i = 0; i < 9; i++) {
        char nm[8];
        snprintf(nm, sizeof(nm), "f%d", i + 1);
        link_func(nm, funcs[i], sizeof(funcs[i]));
        if (funcs[i][0]) n = i + 1;
    }
    return n;
}

static void unlink_all(void) {
    for (int i = 1; i <= 9; i++) {
        char p[128];
        snprintf(p, sizeof(p), CFG "/f%d", i);
        unlink(p);
    }
}

static int link_funcs(const char *const *funcs, int n) {
    for (int i = 0; i < n; i++) {
        char src[160], dst[128];
        snprintf(src, sizeof(src), GADGET "/functions/%s", funcs[i]);
        snprintf(dst, sizeof(dst), CFG "/f%d", i + 1);
        if (symlink(src, dst) < 0) return -errno;
    }
    return 0;
}

static int rndis_active(void) {
    char f[9][32];
    int n = read_links(f);
    return n == 2 && !strcmp(f[0], "rndis.0") && !strcmp(f[1], "ffs.adb");
}

static int switch_gadget(void) {
    char udc[64];
    if (read_file(GADGET "/UDC", udc, sizeof(udc)) <= 0) {
        vlog("rndis: gadget not bound (UDC empty) — leaving USB alone");
        return -1;
    }
    char old[9][32];
    int nold = read_links(old);
    const char *oldp[9];
    for (int i = 0; i < nold; i++) oldp[i] = old[i];
    vlog("rndis: switching USB %s%s%s → rndis.0 + ffs.adb (UDC %s; adb drops for a few seconds)",
         nold > 0 ? old[0] : "", nold > 1 ? " + " : "", nold > 1 ? old[1] : "", udc);

    int e = write_file(GADGET "/UDC", "\n");
    if (e) { vlog("rndis: unbind: %s", strerror(-e)); return -1; }
    usleep(500000);
    /* fixed, locally administered MACs (the stock ones are random per boot): the PC's
     * interface name (enx02657965…) and any network profile for it stay the same */
    write_file(GADGET "/functions/rndis.0/host_addr", RNDIS_HOST_MAC);
    write_file(GADGET "/functions/rndis.0/dev_addr", RNDIS_DEV_MAC);
    unlink_all();
    static const char *const want[] = { "rndis.0", "ffs.adb" };
    e = link_funcs(want, 2);
    if (!e) e = write_file(GADGET "/UDC", udc);
    if (!e) { vlog("rndis: USB bound with rndis.0 + ffs.adb"); return 0; }

    vlog("rndis: bind failed (%s) → restoring the previous USB config", strerror(-e));
    write_file(GADGET "/UDC", "\n");
    unlink_all();
    link_funcs(oldp, nold);
    e = write_file(GADGET "/UDC", udc);
    vlog("rndis: previous USB config %s", e ? "NOT restored (power-cycle restores stock)" : "restored");
    return -1;
}

static int pid_alive(const char *pidfile) {
    char b[16];
    if (read_file(pidfile, b, sizeof(b)) <= 0) return 0;
    int pid = atoi(b);
    if (pid <= 0 || kill(pid, 0) != 0) return 0;
    char p[64], cmd[128];   /* pid files survive reboots: make sure it is still udhcpd */
    snprintf(p, sizeof(p), "/proc/%d/cmdline", pid);
    int fd = open(p, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return 0;
    ssize_t r = read(fd, cmd, sizeof(cmd) - 1);
    close(fd);
    if (r <= 0) return 0;
    for (ssize_t i = 0; i < r; i++) if (!cmd[i]) cmd[i] = ' ';
    cmd[r] = 0;
    return strstr(cmd, "udhcpd") != NULL;
}

static void start_dhcp(void) {
    if (pid_alive(DHCP_PID)) { vlog("rndis: DHCP server already running"); return; }
    mkdir(DIR_TMP, 0777);
    FILE *f = fopen(DHCP_CONF, "w");
    if (!f) { vlog("rndis: %s: %s", DHCP_CONF, strerror(errno)); return; }
    fprintf(f,
        "start 192.168.60.1\nend 192.168.60.1\ninterface rndis0\nmax_leases 1\n"
        "pidfile " DHCP_PID "\nlease_file " DIR_TMP "/udhcpd.leases\n"
        "option subnet 255.255.255.0\noption lease 86400\n");
    fclose(f);
    close(open(DIR_TMP "/udhcpd.leases", O_WRONLY | O_CREAT | O_CLOEXEC, 0644));
    pid_t p = fork();
    if (p == 0) {
        setsid();
        int nul = open("/dev/null", O_RDWR);
        if (nul >= 0) { dup2(nul, 0); dup2(nul, 1); dup2(nul, 2); }
        execl("/sbin/busybox", "busybox", "udhcpd", "-f", DHCP_CONF, (char *)NULL);
        execlp("busybox", "busybox", "udhcpd", "-f", DHCP_CONF, (char *)NULL);
        _exit(127);
    }
    vlog("rndis: DHCP server %s (offers 192.168.60.1 to the PC, no route/DNS)",
         p > 0 ? "started" : "fork failed");
}

int rndis_setup(void) {
    if (access(GADGET "/functions/rndis.0", F_OK) != 0) {
        vlog("rndis: no rndis.0 gadget function — adb only");
        return -1;
    }
    if (!rndis_active() && switch_gadget() < 0) return -1;
    /* rndis0 comes up with the stock 192.168.60.2/24 once the PC binds the interface */
    start_dhcp();
    return 0;
}
