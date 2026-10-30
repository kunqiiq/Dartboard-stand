#ifndef BSP_SYSTEM_H
#define BSP_SYSTEM_H

#include "stm32f4xx_hal.h"

/* Short foreground snapshots only; preserve the caller's interrupt state. */
static inline uint32_t BSP_System_Lock(void)
{
    uint32_t state = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return state;
}

static inline void BSP_System_Unlock(uint32_t state)
{
    __DMB();
    __set_PRIMASK(state);
}

static inline uint32_t BSP_System_GetTick(void)
{
    return HAL_GetTick();
}
#endif
