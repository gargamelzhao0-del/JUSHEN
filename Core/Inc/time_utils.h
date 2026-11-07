/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    time_utils.h
  * @brief   Time utility functions for STM32H7 industrial systems
  * @author  Industrial Control System
  * @date    2025-12-16
  ******************************************************************************
  * @attention
  *
  * This module provides time handling utilities that properly handle
  * uint32_t overflow for HAL_GetTick() and other timing functions.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/**
 * @brief Check if a timeout has occurred (handles uint32_t overflow)
 * @param start_tick Starting tick value from HAL_GetTick()
 * @param timeout_ms Timeout in milliseconds
 * @return true if timeout occurred, false otherwise
 *
 * @note This function properly handles the case where HAL_GetTick()
 *       overflows after 49.7 days (2^32 ms)
 */
static inline bool is_timeout(uint32_t start_tick, uint32_t timeout_ms)
{
    uint32_t current_tick = HAL_GetTick();
    /* Use unsigned arithmetic overflow properties */
    return (current_tick - start_tick) >= timeout_ms;
}

/**
 * @brief Get elapsed time since start (handles uint32_t overflow)
 * @param start_tick Starting tick value from HAL_GetTick()
 * @return Elapsed time in milliseconds
 */
static inline uint32_t get_elapsed_ms(uint32_t start_tick)
{
    uint32_t current_tick = HAL_GetTick();
    return (current_tick - start_tick);
}

/**
 * @brief Check if a periodic event should trigger (handles overflow)
 * @param last_tick Pointer to last trigger tick (will be updated)
 * @param period_ms Period in milliseconds
 * @return true if period has elapsed, false otherwise
 *
 * @note Updates *last_tick when returning true
 */
static inline bool is_period_elapsed(uint32_t* last_tick, uint32_t period_ms)
{
    if (last_tick == NULL) return false;

    uint32_t current_tick = HAL_GetTick();
    if ((current_tick - *last_tick) >= period_ms) {
        *last_tick = current_tick;
        return true;
    }
    return false;
}

/**
 * @brief Convert milliseconds to FreeRTOS ticks
 * @param ms Milliseconds
 * @return Number of RTOS ticks
 */
static inline uint32_t ms_to_ticks(uint32_t ms)
{
    return pdMS_TO_TICKS(ms);
}

#ifdef __cplusplus
}
#endif

#endif /* TIME_UTILS_H */
