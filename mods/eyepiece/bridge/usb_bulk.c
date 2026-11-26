// SPDX-License-Identifier: AGPL-3.0-or-later
#include <errno.h>
#include <fcntl.h>
#include <linux/usb/functionfs.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include "bridge.h"

#define GADGET "/sys/kernel/config/usb_gadget/g1"
#define CFG GADGET "/configs/c.1"
#define FUNCTION GADGET "/functions/ffs.eyepiece"
#define MOUNTPOINT "/dev/usb-ffs/eyepiece"
#define UDC GADGET "/UDC"
#define EP0 MOUNTPOINT "/ep0"
#define EP_IN MOUNTPOINT "/ep1"
#define EP_OUT MOUNTPOINT "/ep2"
#define PIPE_BUF_SIZE 65536

static char g_old_udc[64];
static char g_link[128];
static int g_linked;
static int g_changed;
static int g_state_saved;
static int g_link_slot;
static int g_function_created;
static int g_mounted;
static int g_ep0 = -1;
static int g_enabled;
static int g_server_fd = -1;
static int g_pump_fd = -1;
static int g_ep_in = -1;
static int g_ep_out = -1;
static atomic_int g_session_mode; /* 0 = idle, 1 = bulk, 2 = TCP */
static pthread_t g_rx_thread;
static pthread_t g_tx_thread;
static int g_rx_started;
static int g_tx_started;
static atomic_int g_rx_done;
static atomic_int g_tx_done;
static atomic_int g_stop;
static atomic_ullong g_rx_usb_reads;
static atomic_ullong g_rx_usb_bytes;
static atomic_ullong g_rx_relayed_bytes;
static struct sigaction g_old_pump_wake_action;
static int g_pump_wake_installed;

static void pump_wake_handler(int sig) { (void)sig; }

static int read_text(const char *path, char *buf, size_t size) {
    int fd = open(path, O_RDONLY | O_CLOEXEC);
    if (fd < 0) return -1;
    ssize_t n = read(fd, buf, size - 1);
    int saved = errno;
    close(fd);
    if (n < 0) { errno = saved; return -1; }
    buf[n] = 0;
    while (n > 0 && (buf[n - 1] == '\n' || buf[n - 1] == '\r')) buf[--n] = 0;
    return (int)n;
}

static int write_text(const char *path, const char *value) {
    int fd = open(path, O_WRONLY | O_CLOEXEC);
    if (fd < 0) return -errno;
    ssize_t n = write(fd, value, strlen(value));
    int e = n < 0 ? -errno : n == (ssize_t)strlen(value) ? 0 : -EIO;
    close(fd);
    return e;
}

static int save_state(int slot) {
    char tmp[256];
    const char *state = tmp_path("eyepiece.bulk-state");
    snprintf(tmp, sizeof(tmp), "%s.tmp", state);
    FILE *f = fopen(tmp, "w");
    if (!f) return -errno;
    int saved = 0;
    if (fprintf(f, "%s %d\n", g_old_udc, slot) <= 0) saved = errno ? errno : EIO;
    if (!saved && fsync(fileno(f)) < 0) saved = errno;
    if (fclose(f) != 0 && !saved) saved = errno ? errno : EIO;
    if (saved) { unlink(tmp); return -saved; }
    if (rename(tmp, state) < 0) { saved = errno; unlink(tmp); return -saved; }
    g_state_saved = 1;
    g_link_slot = slot;
    return 0;
}

int bulk_recover(void) {
    const char *state_path = tmp_path("eyepiece.bulk-state");
    char state[128], old_udc[64], link[128], target[256];
    int slot;
    if (read_text(state_path, state, sizeof(state)) <= 0) return 0;
    if (sscanf(state, "%63s %d", old_udc, &slot) != 2 || !old_udc[0] || slot < 1 || slot > 9) {
        vlog("bulk: invalid recovery snapshot %s; leaving USB unchanged", state_path);
        return -1;
    }
    snprintf(link, sizeof(link), CFG "/f%d", slot);
    char current_udc[64] = "";
    read_text(UDC, current_udc, sizeof(current_udc));
    if (current_udc[0]) {
        int e = write_text(UDC, "\n");
        if (e) { vlog("bulk: stale recovery unbind failed (%s)", strerror(-e)); return -1; }
    }
    ssize_t n = readlink(link, target, sizeof(target) - 1);
    if (n > 0) {
        target[n] = 0;
        if (!strcmp(target, FUNCTION) && unlink(link) < 0) {
            vlog("bulk: stale recovery unlink %s failed (%s)", link, strerror(errno));
            write_text(UDC, old_udc);
            return -1;
        }
    }
    if (umount(MOUNTPOINT) < 0 && errno != EINVAL && errno != ENOENT) {
        vlog("bulk: stale recovery unmount failed (%s)", strerror(errno));
        write_text(UDC, old_udc);
        return -1;
    }
    rmdir(MOUNTPOINT);
    if (rmdir(FUNCTION) < 0 && errno != ENOENT) {
        vlog("bulk: stale recovery remove function failed (%s)", strerror(errno));
        write_text(UDC, old_udc);
        return -1;
    }
    int e = write_text(UDC, old_udc);
    if (e) {
        vlog("bulk: stale recovery rebind failed (%s); reconnect USB or power-cycle", strerror(-e));
        return -1;
    }
    unlink(state_path);
    vlog("bulk: recovered an interrupted FunctionFS setup; prior UDC binding restored");
    return 0;
}

static void put_le32(unsigned char *p, unsigned value) {
    p[0] = (unsigned char)value;
    p[1] = (unsigned char)(value >> 8);
    p[2] = (unsigned char)(value >> 16);
    p[3] = (unsigned char)(value >> 24);
}

static int write_ep0_blob(const unsigned char *data, size_t size) {
    ssize_t n;
    do { n = write(g_ep0, data, size); } while (n < 0 && errno == EINTR);
    return n == (ssize_t)size ? 0 : -1;
}

static int write_descriptors(void) {
    static const unsigned char fs[] = {
        9, 4, 0, 0, 2, 0xff, 0, 0, 1,
        7, 5, 0x81, 2, 64, 0, 0,
        7, 5, 0x01, 2, 64, 0, 0,
    };
    static const unsigned char hs[] = {
        9, 4, 0, 0, 2, 0xff, 0, 0, 1,
        7, 5, 0x81, 2, 0, 2, 0,
        7, 5, 0x01, 2, 0, 2, 0,
    };
    static const unsigned char ss[] = {
        9, 4, 0, 0, 2, 0xff, 0, 0, 1,
        7, 5, 0x81, 2, 0, 4, 0, 6, 48, 0, 0, 0, 0,
        7, 5, 0x01, 2, 0, 4, 0, 6, 48, 0, 0, 0, 0,
    };
    enum { HEADER_LEN = 24, DESC_LEN = sizeof(fs) + sizeof(hs) + sizeof(ss) };
    unsigned char blob[HEADER_LEN + DESC_LEN] = {0};
    put_le32(blob, 3); /* FUNCTIONFS_DESCRIPTORS_MAGIC_V2 */
    put_le32(blob + 4, sizeof(blob));
    put_le32(blob + 8, 7); /* full, high, and super speed descriptors */
    put_le32(blob + 12, 3);
    put_le32(blob + 16, 3);
    put_le32(blob + 20, 5);
    memcpy(blob + HEADER_LEN, fs, sizeof(fs));
    memcpy(blob + HEADER_LEN + sizeof(fs), hs, sizeof(hs));
    memcpy(blob + HEADER_LEN + sizeof(fs) + sizeof(hs), ss, sizeof(ss));
    if (write_ep0_blob(blob, sizeof(blob)) < 0) return -1;

    static const char interface_name[] = "Eyepiece bulk";
    unsigned char strings[16 + 2 + sizeof(interface_name)] = {0};
    put_le32(strings, 2); /* FUNCTIONFS_STRINGS_MAGIC */
    put_le32(strings + 4, sizeof(strings));
    put_le32(strings + 8, 1);
    put_le32(strings + 12, 1);
    strings[16] = 0x09; /* English (United States) */
    strings[17] = 0x04;
    memcpy(strings + 18, interface_name, sizeof(interface_name));
    return write_ep0_blob(strings, sizeof(strings));
}

static int config_slot(void) {
    char path[128];
    struct stat st;
    for (int i = 1; i <= 9; i++) {
        snprintf(path, sizeof(path), CFG "/f%d", i);
        if (lstat(path, &st) < 0) {
            if (errno == ENOENT) return i;
            return -1;
        }
    }
    return -1;
}

static int restore_gadget(void) {
    if (!g_changed) return 0;
    int e = write_text(UDC, "\n");
    if (e) {
        vlog("bulk: rollback unbind failed (%s); leaving FunctionFS objects intact", strerror(-e));
        return e;
    }
    if (g_linked && unlink(g_link) < 0 && errno != ENOENT) {
        e = -errno;
        vlog("bulk: rollback unlink %s failed (%s)", g_link, strerror(errno));
        write_text(UDC, g_old_udc);
        return e;
    }
    g_linked = 0;
    e = write_text(UDC, g_old_udc);
    if (e) {
        vlog("bulk: rollback rebind failed (%s); reconnect USB or power-cycle", strerror(-e));
        return e;
    }
    g_changed = 0;
    return 0;
}

static void remove_functionfs(void) {
    int clean = 1;
    if (g_ep0 >= 0) { close(g_ep0); g_ep0 = -1; }
    if (g_mounted) {
        if (umount(MOUNTPOINT) < 0) vlog("bulk: unmount %s failed: %s", MOUNTPOINT, strerror(errno));
        else g_mounted = 0;
        if (g_mounted) clean = 0;
    }
    if (!g_mounted && rmdir(MOUNTPOINT) < 0 && errno != ENOENT) {
        vlog("bulk: remove %s failed: %s", MOUNTPOINT, strerror(errno));
        clean = 0;
    }
    if (g_function_created && !g_linked && !g_changed) {
        if (rmdir(FUNCTION) < 0 && errno != ENOENT)
            { vlog("bulk: remove FunctionFS function failed: %s", strerror(errno)); clean = 0; }
        else g_function_created = 0;
    } else if (g_function_created) {
        clean = 0;
    }
    if (clean && !g_mounted && !g_function_created && !g_linked && !g_changed && g_state_saved) {
        unlink(tmp_path("eyepiece.bulk-state"));
        g_state_saved = 0;
    }
}

int bulk_setup(void) {
    if (bulk_recover() < 0) return -1;
    int slot = config_slot();
    if (slot < 0) { vlog("bulk: no free config function link"); return -1; }
    if (read_text(UDC, g_old_udc, sizeof(g_old_udc)) <= 0) {
        vlog("bulk: gadget is not bound; leaving USB unchanged");
        return -1;
    }
    snprintf(g_link, sizeof(g_link), CFG "/f%d", slot);
    int e = save_state(slot);
    if (e) {
        vlog("bulk: cannot save recovery state (%s); leaving USB unchanged", strerror(-e));
        return -1;
    }
    if (mkdir(FUNCTION, 0777) < 0) {
        vlog("bulk: create %s failed: %s", FUNCTION, strerror(errno));
        remove_functionfs();
        return -1;
    }
    g_function_created = 1;
    if (mkdir(MOUNTPOINT, 0777) < 0) {
        vlog("bulk: create %s failed: %s", MOUNTPOINT, strerror(errno));
        remove_functionfs();
        return -1;
    }
    if (mount("eyepiece", MOUNTPOINT, "functionfs", 0, NULL) < 0) {
        vlog("bulk: mount FunctionFS failed: %s", strerror(errno));
        remove_functionfs();
        return -1;
    }
    g_mounted = 1;
    g_ep0 = open(EP0, O_RDWR | O_CLOEXEC | O_NONBLOCK);
    if (g_ep0 < 0 || write_descriptors() < 0) {
        vlog("bulk: FunctionFS descriptor setup failed: %s", strerror(errno));
        remove_functionfs();
        return -1;
    }

    vlog("bulk: adding dedicated interface at %s (USB will re-enumerate)", g_link);
    g_changed = 1;
    e = write_text(UDC, "\n");
    if (e) {
        vlog("bulk: UDC unbind failed: %s", strerror(-e));
        restore_gadget();
        remove_functionfs();
        return -1;
    }
    if (symlink(FUNCTION, g_link) < 0) {
        e = -errno;
        vlog("bulk: link %s failed: %s", g_link, strerror(errno));
        restore_gadget();
        remove_functionfs();
        return -1;
    }
    g_linked = 1;
    e = write_text(UDC, g_old_udc);
    if (e) {
        vlog("bulk: bind with bulk interface failed: %s", strerror(-e));
        restore_gadget();
        remove_functionfs();
        return -1;
    }
    vlog("bulk: FunctionFS interface bound; waiting for host claim");
    return 0;
}

static void stop_pumps(void) {
    atomic_store(&g_stop, 1);
    if (g_pump_fd >= 0) shutdown(g_pump_fd, SHUT_RDWR);
}

static int poll_ready(int fd, short events) {
    struct pollfd p = { .fd = fd, .events = events };
    int r;
    do { r = poll(&p, 1, 100); } while (r < 0 && errno == EINTR && !atomic_load(&g_stop));
    if (r <= 0) return r;
    if (p.revents & events) return 1;
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) { errno = EIO; return -1; }
    return 0;
}

static int write_socket_all(int fd, const unsigned char *buf, size_t size) {
    size_t off = 0;
    while (off < size && !atomic_load(&g_stop)) {
        int ready = poll_ready(fd, POLLOUT);
        if (ready < 0) return -1;
        if (!ready) continue;
        ssize_t n = send(fd, buf + off, size - off, MSG_NOSIGNAL);
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n <= 0) return -1;
        off += (size_t)n;
    }
    return off == size ? 0 : -1;
}

static int write_endpoint_all(int fd, const unsigned char *buf, size_t size) {
    size_t off = 0;
    while (off < size && !atomic_load(&g_stop)) {
        ssize_t n = write(fd, buf + off, size - off);
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n <= 0) return -1;
        off += (size_t)n;
    }
    return off == size ? 0 : -1;
}

static void pump_failed(void) {
    atomic_store(&g_stop, 1);
    if (g_pump_fd >= 0) shutdown(g_pump_fd, SHUT_RDWR);
}

void bulk_log_diagnostics(void) {
    vlog("bulk: OUT totals: reads=%llu usb_bytes=%llu relayed_bytes=%llu",
         (unsigned long long)atomic_load(&g_rx_usb_reads),
         (unsigned long long)atomic_load(&g_rx_usb_bytes),
         (unsigned long long)atomic_load(&g_rx_relayed_bytes));
}

static void *bulk_to_bridge(void *unused) {
    (void)unused;
    unsigned char buf[PIPE_BUF_SIZE];
    int read_armed = 0;
    while (!atomic_load(&g_stop)) {
        if (!read_armed) {
            vlog("bulk: OUT endpoint blocking read armed");
            read_armed = 1;
        }
        ssize_t n = read(g_ep_out, buf, sizeof(buf));
        if (n < 0 && errno == EINTR && !atomic_load(&g_stop)) continue;
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n < 0) {
            if (!atomic_load(&g_stop)) vlog("bulk: OUT endpoint read failed (%s)", strerror(errno));
            break;
        }
        if (n == 0) continue; /* USB ZLP is not stream EOF. */
        atomic_fetch_add(&g_rx_usb_reads, 1);
        atomic_fetch_add(&g_rx_usb_bytes, (unsigned long long)n);
        int mode = atomic_load(&g_session_mode);
        if (mode == 2) continue; /* TCP won arbitration; drain idle bulk input. */
        if (mode == 0) {
            int idle = 0;
            if (!atomic_compare_exchange_strong(&g_session_mode, &idle, 1) && idle == 2)
                continue;
        }
        if (write_socket_all(g_pump_fd, buf, (size_t)n) < 0) break;
        atomic_fetch_add(&g_rx_relayed_bytes, (unsigned long long)n);
    }
    pump_failed();
    atomic_store(&g_rx_done, 1);
    return NULL;
}

static void *bridge_to_bulk(void *unused) {
    (void)unused;
    unsigned char buf[PIPE_BUF_SIZE];
    int write_logged = 0;
    while (!atomic_load(&g_stop)) {
        int ready = poll_ready(g_pump_fd, POLLIN);
        if (ready < 0) break;
        if (!ready) continue;
        ssize_t n = recv(g_pump_fd, buf, sizeof(buf), 0);
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
        if (n <= 0) break;
        if (write_endpoint_all(g_ep_in, buf, (size_t)n) < 0) {
            if (!atomic_load(&g_stop)) vlog("bulk: IN endpoint write failed (%s)", strerror(errno));
            break;
        }
        if (!write_logged) {
            vlog("bulk: IN endpoint write completed (%zd bytes)", n);
            write_logged = 1;
        }
    }
    pump_failed();
    atomic_store(&g_tx_done, 1);
    return NULL;
}

static int start_client(void) {
    int pair[2];
    struct sigaction wake_action = {0};
    wake_action.sa_handler = pump_wake_handler;
    sigemptyset(&wake_action.sa_mask);
    if (sigaction(SIGUSR1, &wake_action, &g_old_pump_wake_action) < 0) return -1;
    g_pump_wake_installed = 1;

    g_ep_in = open(EP_IN, O_WRONLY | O_CLOEXEC);
    if (g_ep_in < 0) goto fail_wake;
    g_ep_out = open(EP_OUT, O_RDONLY | O_CLOEXEC);
    if (g_ep_out < 0) goto fail_endpoints;
    if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, pair) < 0) {
        goto fail_endpoints;
    }
    int flags = fcntl(pair[1], F_GETFL);
    if (flags < 0 || fcntl(pair[1], F_SETFL, flags | O_NONBLOCK) < 0) {
        close(pair[0]); close(pair[1]);
        goto fail_endpoints;
    }
    g_server_fd = pair[0];
    g_pump_fd = pair[1];
    atomic_store(&g_stop, 0);
    atomic_store(&g_rx_done, 0);
    atomic_store(&g_tx_done, 0);
    if (pthread_create(&g_rx_thread, NULL, bulk_to_bridge, NULL) != 0) {
        bulk_client_done();
        return -1;
    }
    g_rx_started = 1;
    if (pthread_create(&g_tx_thread, NULL, bridge_to_bulk, NULL) != 0) {
        bulk_client_done();
        return -1;
    }
    g_tx_started = 1;
    return g_server_fd;

fail_endpoints:
    if (g_ep_in >= 0) close(g_ep_in);
    if (g_ep_out >= 0) close(g_ep_out);
    g_ep_in = g_ep_out = -1;
fail_wake:
    sigaction(SIGUSR1, &g_old_pump_wake_action, NULL);
    g_pump_wake_installed = 0;
    return -1;
}

int bulk_event_fd(void) { return g_ep0; }

void bulk_update_events(void) {
    if (g_ep0 < 0) return;
    struct usb_functionfs_event events[8];
    for (;;) {
        ssize_t n = read(g_ep0, events, sizeof(events));
        if (n < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
            if (errno == EINTR) continue;
            break;
        }
        if (n <= 0) break;
        size_t count = (size_t)n / sizeof(events[0]);
        for (size_t i = 0; i < count; i++) {
            if (events[i].type == FUNCTIONFS_ENABLE) {
                g_enabled = 1;
                vlog("bulk: host enabled interface");
            } else if (events[i].type == FUNCTIONFS_DISABLE || events[i].type == FUNCTIONFS_UNBIND) {
                g_enabled = 0;
                vlog("bulk: host disabled interface");
            }
        }
    }
}

int bulk_pending_client(void) {
    if (g_server_fd >= 0) return g_server_fd;
    if (!g_enabled) return -1;
    int fd = start_client();
    if (fd < 0) vlog("bulk: cannot open enabled endpoints: %s", strerror(errno));
    return fd;
}

int bulk_session_selected(void) { return atomic_load(&g_session_mode) == 1; }

int bulk_tcp_claim(void) {
    int idle = 0;
    return atomic_compare_exchange_strong(&g_session_mode, &idle, 2);
}

void bulk_tcp_done(void) {
    int tcp = 2;
    atomic_compare_exchange_strong(&g_session_mode, &tcp, 0);
}

void bulk_client_done(void) {
    if (g_server_fd < 0 && g_pump_fd < 0) return;
    stop_pumps();
    while ((g_rx_started && !atomic_load(&g_rx_done)) ||
           (g_tx_started && !atomic_load(&g_tx_done))) {
        if (g_rx_started && !atomic_load(&g_rx_done))
            pthread_kill(g_rx_thread, SIGUSR1);
        if (g_tx_started && !atomic_load(&g_tx_done))
            pthread_kill(g_tx_thread, SIGUSR1);
        usleep(10000);
    }
    if (g_rx_started) { pthread_join(g_rx_thread, NULL); g_rx_started = 0; }
    if (g_tx_started) { pthread_join(g_tx_thread, NULL); g_tx_started = 0; }
    if (g_server_fd >= 0) close(g_server_fd);
    if (g_pump_fd >= 0) close(g_pump_fd);
    if (g_ep_in >= 0) close(g_ep_in);
    if (g_ep_out >= 0) close(g_ep_out);
    g_server_fd = g_pump_fd = g_ep_in = g_ep_out = -1;
    atomic_store(&g_session_mode, 0);
    if (g_pump_wake_installed) {
        sigaction(SIGUSR1, &g_old_pump_wake_action, NULL);
        g_pump_wake_installed = 0;
    }
}

void bulk_stop(void) {
    bulk_client_done();
    if (g_changed) {
        int e = restore_gadget();
        if (e) return;
    }
    remove_functionfs();
}
