/**
 * @file    foc_time.c
 * @brief   AT32F413 32-bit DWT CPU-cycle time source.
 * @author  Codex
 * @date    2026-09-10
 */

#include <stdint.h>

#include "at32f413.h"
#include "foc_time.h"
#include "peripheral.h"

#if defined(DWT)

/**
 * @brief Enable the AT32F413 DWT cycle counter.
 * @param None.
 * @return None.
 */
static void foc_time_EnableCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Read the current AT32F413 CPU-cycle counter.
 * @param None.
 * @return Low 32 bits of the DWT cycle counter.
 * @note The first call enables the hardware counter without masking IRQs.
 */
foc_time_tick_t foc_time_now32(void)
{
    uint32_t wControl = DWT->CTRL;

    if ((wControl & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
        foc_time_EnableCounter();
    } else {
        /* The counter is already enabled by the target startup path. */
    }
    return (foc_time_tick_t)DWT->CYCCNT;
}

/**
 * @brief Read the AT32F413 CPU-cycle counter frequency.
 * @param None.
 * @return CPU cycles per second.
 */
uint32_t foc_time_frequency_hz(void)
{
    return get_system_core_clock_hz();
}

#else

/**
 * @brief Report that no 32-bit CPU-cycle counter is available.
 * @param None.
 * @return Zero, which disables cycle-based extrapolation.
 */
foc_time_tick_t foc_time_now32(void)
{
    return 0U;
}

/**
 * @brief Report that no 32-bit CPU-cycle counter is available.
 * @param None.
 * @return Zero, which disables cycle-based extrapolation.
 */
uint32_t foc_time_frequency_hz(void)
{
    return 0U;
}

#endif
