#ifndef __COMMON_INCLUDE__
#define __COMMON_INCLUDE__

#include "bswap.h"

#define CPU_INTERRUPT_EXIT 0x01
#define CPU_INTERRUPT_HARD 0x02

#define xglue(x, y) x ## y
#define glue(x, y) xglue(x, y)

extern uint8_t *phys_ram_base;

typedef uint32_t CPUReadMemoryFunc(uint32_t addr);
typedef void CPUWriteMemoryFunc(uint32_t addr, uint32_t value);

int cpu_register_io_memory(uint64_t begin_addr, uint64_t len, CPUReadMemoryFunc **mem_read, CPUWriteMemoryFunc **mem_write);

static inline int lduw_raw(void *ptr)
{
    return *(uint16_t *)ptr;
}

static inline int ldsw_raw(void *ptr)
{
    return *(int16_t *)ptr;
}

static inline int ldl_raw(void *ptr)
{
    return *(uint32_t *)ptr;
}

static inline uint64_t ldq_raw(void *ptr)
{
    return *(uint64_t *)ptr;
}

static inline void stw_raw(void *ptr, int v)
{
    *(uint16_t *)ptr = v;
}

static inline void stl_raw(void *ptr, int v)
{
    *(uint32_t *)ptr = v;
}

static inline void stq_raw(void *ptr, uint64_t v)
{
    *(uint64_t *)ptr = v;
}

/* float access */

static inline float ldfl_raw(void *ptr)
{
    return *(float *)ptr;
}

static inline double ldfq_raw(void *ptr)
{
    return *(double *)ptr;
}

static inline void stfl_raw(void *ptr, float v)
{
    *(float *)ptr = v;
}

static inline void stfq_raw(void *ptr, double v)
{
    *(double *)ptr = v;
}


static inline uint16_t tswap16(uint16_t s)
{
    return s;
}

static inline uint32_t tswap32(uint32_t s)
{
    return s;
}

static inline uint64_t tswap64(uint64_t s)
{
    return s;
}

static inline void tswap16s(uint16_t *s)
{
}

static inline void tswap32s(uint32_t *s)
{
}

static inline void tswap64s(uint64_t *s)
{
}
#endif
