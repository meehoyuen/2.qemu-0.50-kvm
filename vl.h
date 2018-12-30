/*
 * QEMU System Emulator header
 */
#ifndef VL_H
#define VL_H

#include "common.h"

/* vl.c */
extern int reset_requested;

typedef void (IOPortWriteFunc)(void *env, uint32_t address, uint32_t data, uint32_t size);
typedef uint32_t (IOPortReadFunc)(void *env, uint32_t address, uint32_t size);

void *get_mmap_addr(unsigned long size);
int register_ioport_read(int start, int length, IOPortReadFunc *func, int size);
int register_ioport_write(int start, int length, IOPortWriteFunc *func, int size);

void kbd_put_keycode(int keycode);

#define MOUSE_EVENT_LBUTTON 0x01
#define MOUSE_EVENT_RBUTTON 0x02
#define MOUSE_EVENT_MBUTTON 0x04
void kbd_mouse_event(int dx, int dy, int dz, int buttons_state);

/* block.c */
typedef struct BlockDriverState BlockDriverState;

BlockDriverState *bdrv_open(const char *filename, int snapshot);
void bdrv_close(BlockDriverState *bs);
int bdrv_read(BlockDriverState *bs, int64_t sector_num,
              uint8_t *buf, int nb_sectors);
int bdrv_write(BlockDriverState *bs, int64_t sector_num,
               const uint8_t *buf, int nb_sectors);
void bdrv_get_geometry(BlockDriverState *bs, int64_t *nb_sectors_ptr);
int bdrv_commit(BlockDriverState *bs);

/* user mode linux compatible COW file */
#define COW_MAGIC 0x4f4f4f4d  /* MOOO */
#define COW_VERSION 2

struct cow_header_v2
{
    uint32_t magic;
    uint32_t version;
    char backing_file[1024];
    int32_t mtime;
    uint64_t size;
    uint32_t sectorsize;
};

/* vga.c */

#define VGA_RAM_SIZE (8192 * 1024)

typedef struct DisplayState
{
    uint8_t *data;
    int linesize;
    int depth;
    void (*dpy_update)(struct DisplayState *s, int x, int y, int w, int h);
    void (*dpy_resize)(struct DisplayState *s, int w, int h);
    void (*dpy_refresh)(struct DisplayState *s);
} DisplayState;

static inline void dpy_update(DisplayState *s, int x, int y, int w, int h)
{
    s->dpy_update(s, x, y, w, h);
}

static inline void dpy_resize(DisplayState *s, int w, int h)
{
    s->dpy_resize(s, w, h);
}

int vga_init(DisplayState *ds, uint8_t *vga_ram_base,
             unsigned long vga_ram_offset, int vga_ram_size);
void vga_update_display(void);

/* sdl.c */
void sdl_display_init(DisplayState *ds);

int kvm_interrupt(int irq, int level);

#endif /* VL_H */
