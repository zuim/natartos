// SPDX-License-Identifier: AGPL-3.0-or-later
#include <dlfcn.h>
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum {
    WIDTH = 1920,
    HEIGHT = 1080,
    STRIDE = WIDTH * 4,
    FRAME_SIZE = STRIDE * HEIGHT,
    FRAME_DESC_SIZE = 0x330,
    PIXFMT_ARGB8_I32 = 20015,
};

typedef struct {
    const char *name;
    void *attach;
    void *detach;
    void *reserved;
} device_desc_t;

typedef struct {
    uint8_t enable;
    uint8_t global_alpha_enable;
    uint8_t pre_blend;
    uint8_t reserved0;
    uint32_t global_alpha;
    uint8_t algorithm;
    uint8_t order;
    uint8_t reserved1[2];
    uint32_t position;
    int32_t x;
    int32_t y;
} blend_config_t;

_Static_assert(sizeof(blend_config_t) == 24, "unexpected blending configuration size");

static volatile sig_atomic_t stopped;
static int (*initialize)(device_desc_t *);
static int (*device_open)(const char *, void *, void **);
static int (*device_start)(void *, void *);
static int (*device_stop)(void *, void *);
static int (*device_close)(void *);
static int (*mem_alloc)(void *, void **, uint32_t, uint32_t, uint32_t, uint32_t);
static int (*mem_map)(void *, void **);
static int (*mem_sync)(void *, uint32_t);
static int (*mem_free)(void *);
static int (*display_open)(void *, void **, uint8_t);
static int (*display_close)(void *, void **);
static int (*acquire_plane)(void *, uint8_t, int8_t *);
static int (*release_plane)(void *, int8_t);
static int (*set_blending)(void *, int8_t, void *);
static int (*push_frame)(void *, int8_t, void *);

static void stop(int signal_number) {
    (void)signal_number;
    stopped = 1;
}

static void rgba_to_argb8_i32(uint8_t *pixels, size_t size) {
    for (size_t i = 0; i < size; i += 4) {
        uint8_t red = pixels[i];
        pixels[i] = pixels[i + 2];
        pixels[i + 2] = red;
    }
}

static void *load_symbol(void *handle, const char *name) {
    void *result = dlsym(handle, name);
    if (!result) fprintf(stderr, "missing %s: %s\n", name, dlerror());
    return result;
}

#define LOAD(var, name) do { \
    *(void **)(&(var)) = load_symbol(hal, (name)); \
    if (!(var)) goto cleanup; \
} while (0)
#define SET32(frame, offset, value) do { \
    uint32_t field_value = (value); \
    memcpy((frame) + (offset), &field_value, sizeof(field_value)); \
} while (0)
#define CHECK(expr) do { \
    result = (expr); \
    if (result != 0) { fprintf(stderr, "%s: %d\n", #expr, result); goto cleanup; } \
} while (0)

int main(int argc, char **argv) {
    if (argc != 3 || (strcmp(argv[1], "20012") && strcmp(argv[1], "20002") &&
                      strcmp(argv[1], "20015"))) {
        fprintf(stderr, "usage: %s 20012|20002|20015 seconds\n", argv[0]);
        return 2;
    }

    errno = 0;
    char *end;
    long seconds = strtol(argv[2], &end, 10);
    if (errno || end == argv[2] || *end || seconds < 1 || seconds > 600) {
        fprintf(stderr, "duration must be 1..600 seconds\n");
        return 2;
    }

    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = stop;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) != 0 || sigaction(SIGTERM, &action, NULL) != 0) {
        perror("sigaction");
        return 1;
    }

    int status = 1;
    int result = 0;
    int ion_started = 0;
    int plane_acquired = 0;
    int frame_pushed = 0;
    int safe_to_free = 1;
    int8_t plane = 6;
    uint32_t zero = 0;
    void *hal = NULL;
    void *ion = NULL;
    void *display_device = NULL;
    void *display = NULL;
    void *memory = NULL;
    void *address = NULL;
    uint8_t *pixels = NULL;
    uint8_t *frame = NULL;
    FILE *input = NULL;

    input = fopen("/data/local/tmp/eyepiece/gfx1-alpha-pattern.rgba", "rb");
    if (!input) {
        perror("open pattern");
        goto cleanup;
    }
    pixels = malloc(FRAME_SIZE);
    if (!pixels) {
        perror("malloc pattern");
        goto cleanup;
    }
    if (fread(pixels, 1, FRAME_SIZE, input) != FRAME_SIZE ||
        fgetc(input) != EOF || ferror(input)) {
        fprintf(stderr, "pattern length/read error\n");
        goto cleanup;
    }
    fclose(input);
    input = NULL;
    uint32_t format = (uint32_t)strtoul(argv[1], NULL, 10);
    if (format == PIXFMT_ARGB8_I32) rgba_to_argb8_i32(pixels, FRAME_SIZE);

    frame = calloc(1, FRAME_DESC_SIZE);
    if (!frame) {
        perror("calloc frame descriptor");
        goto cleanup;
    }

    hal = dlopen("libduml_hal.so", RTLD_NOW | RTLD_GLOBAL);
    if (!hal) {
        fprintf(stderr, "dlopen: %s\n", dlerror());
        goto cleanup;
    }
    LOAD(initialize, "duss_hal_initialize");
    LOAD(device_open, "duss_hal_device_open");
    LOAD(device_start, "duss_hal_device_start");
    LOAD(device_stop, "duss_hal_device_stop");
    LOAD(device_close, "duss_hal_device_close");
    LOAD(mem_alloc, "duss_hal_mem_alloc");
    LOAD(mem_map, "duss_hal_mem_map");
    LOAD(mem_sync, "duss_hal_mem_sync");
    LOAD(mem_free, "duss_hal_mem_free");
    LOAD(display_open, "duss_hal_display_open");
    LOAD(display_close, "duss_hal_display_close");
    LOAD(acquire_plane, "duss_hal_display_aquire_plane");
    LOAD(release_plane, "duss_hal_display_release_plane");
    LOAD(set_blending, "duss_hal_display_plane_blending_set");
    LOAD(push_frame, "duss_hal_display_push_frame");

    device_desc_t devices[] = {
        {"/dev/ion0", load_symbol(hal, "duss_hal_attach_ion_mem"),
         load_symbol(hal, "duss_hal_detach_ion_mem"), NULL},
        {"/dev/dji_display", load_symbol(hal, "duss_hal_attach_disp"),
         load_symbol(hal, "duss_hal_detach_disp"), NULL},
        {NULL, NULL, NULL, NULL},
    };
    if (!devices[0].attach || !devices[0].detach ||
        !devices[1].attach || !devices[1].detach) goto cleanup;

    CHECK(initialize(devices));
    CHECK(device_open("/dev/ion0", &zero, &ion));
    CHECK(device_start(ion, &zero));
    ion_started = 1;
    CHECK(mem_alloc(ion, &memory, FRAME_SIZE, 0x80, 0, 0));
    CHECK(mem_map(memory, &address));
    if (!address) {
        fprintf(stderr, "mem_map returned null\n");
        goto cleanup;
    }
    memcpy(address, pixels, FRAME_SIZE);
    free(pixels);
    pixels = NULL;
    CHECK(mem_sync(memory, 1));

    memcpy(frame + 0x20, &memory, sizeof(memory));
    SET32(frame, 0x28, format);
    SET32(frame, 0x38, WIDTH);
    SET32(frame, 0x3c, HEIGHT);
    SET32(frame, 0x40, STRIDE);
    SET32(frame, 0x4c, HEIGHT);
    SET32(frame, 0x50, FRAME_SIZE);
    SET32(frame, 0xb0, 1);
    if (stopped) goto cleanup;

    CHECK(device_open("/dev/dji_display", &zero, &display_device));
    CHECK(display_open(display_device, &display, 0));
    CHECK(acquire_plane(display, 6, &plane));
    if (plane != 6) {
        fprintf(stderr, "unexpected acquired plane %d\n", plane);
        plane_acquired = 1;
        goto cleanup;
    }
    plane_acquired = 1;

    blend_config_t blend = {
        .enable = 1,
        .global_alpha_enable = 0,
        .global_alpha = 0,
        .algorithm = 0,
        .order = 1,
    };
    CHECK(set_blending(display, plane, &blend));
    if (stopped) goto cleanup;
    CHECK(push_frame(display, plane, frame));
    frame_pushed = 1;

    fprintf(stderr,
            "GFX1: format %s, global alpha disabled, order 1; submitted for %ld seconds\n",
            argv[1], seconds);
    struct timespec delay = {1, 0};
    for (long i = 0; i < seconds && !stopped; i++) nanosleep(&delay, NULL);
    status = 0;

cleanup:
    if (input) fclose(input);
    free(pixels);
    if (plane_acquired) {
        result = release_plane(display, plane);
        if (result != 0) {
            fprintf(stderr, "release_plane: %d; retaining framebuffer resources\n", result);
            if (frame_pushed) safe_to_free = 0;
            status = 1;
        } else {
            frame_pushed = 0;
        }
    }
    if (display && (result = display_close(display_device, &display)) != 0) {
        fprintf(stderr, "display_close: %d\n", result);
        status = 1;
    }
    if (display_device &&
        (result = device_close(display_device)) != 0) {
        fprintf(stderr, "display device_close: %d\n", result);
        status = 1;
    }
    if (safe_to_free && memory && (result = mem_free(memory)) != 0) {
        fprintf(stderr, "mem_free: %d\n", result);
        status = 1;
    }
    if (safe_to_free && ion_started && (result = device_stop(ion, &zero)) != 0) {
        fprintf(stderr, "ion stop: %d\n", result);
        status = 1;
    }
    if (safe_to_free && ion && (result = device_close(ion)) != 0) {
        fprintf(stderr, "ion close: %d\n", result);
        status = 1;
    }
    if (frame_pushed) fprintf(stderr, "framebuffer left allocated after failed plane release\n");
    if (!frame_pushed) free(frame);
    fprintf(stderr, "test finished: %s\n", status ? "error" : "ok");
    return status;
}
