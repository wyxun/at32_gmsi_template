/**
 * @file    foc_time.h
 * @brief   FOC 32-bit CPU-cycle time source contract.
 * @author  Codex
 * @date    2026-09-10
 */

#ifndef FOC_TIME_H
#define FOC_TIME_H

#include <stdint.h>

/** @brief Low 32 bits of the CPU-cycle counter. */
typedef uint32_t foc_time_tick_t;

/**
 * @brief Read the current low 32-bit CPU-cycle counter.
 * @param None.
 * @return Low 32 bits of the CPU-cycle counter, or zero when unavailable.
 */
foc_time_tick_t foc_time_now32(void);

/**
 * @brief Read the CPU-cycle counter frequency.
 * @param None.
 * @return CPU cycles per second, or zero when the counter is unavailable.
 */
uint32_t foc_time_frequency_hz(void);

#endif /* FOC_TIME_H */
