#ifndef DWT_H
#define DWT_H

#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/dwt.h>

inline void dwt_init()
{
    SCS_DEMCR |= SCS_DEMCR_TRCENA;
    DWT_CYCCNT = 0u;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
}

inline uint32_t dwt_get_cycles(void)
{
    return DWT_CYCCNT;
}

#endif // DWT_H