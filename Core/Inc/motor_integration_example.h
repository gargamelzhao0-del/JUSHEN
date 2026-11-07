/**
 ****************************************************************************************************
 * @file        motor_integration_example.h
 * @version     V1.0
 * @brief       电机控制集成示例代码 - 头文件
 * @note        包含力位混合控制、四足控制器、足端力传感器处理、I2C通信和多电机同步控制的集成示例
 ****************************************************************************************************
 */

#ifndef __MOTOR_INTEGRATION_EXAMPLE_H
#define __MOTOR_INTEGRATION_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ============================================================================
 * 公共函数声明
 * ============================================================================
 */

/**
 * @brief       启动集成示例任务
 * @retval      0=成功, -1=失败
 */
int motor_integration_example_start(void);

/**
 * @brief       获取系统状态
 */
void motor_integration_get_status(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_INTEGRATION_EXAMPLE_H */
