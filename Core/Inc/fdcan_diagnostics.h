/**
  ******************************************************************************
  * @file    fdcan_diagnostics.h
  * @brief   FDCAN硬件诊断工具
  ******************************************************************************
  */

#ifndef __FDCAN_DIAGNOSTICS_H
#define __FDCAN_DIAGNOSTICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 诊断函数 */
void FDCAN_PrintBusStatus(FDCAN_HandleTypeDef *hfdcan, const char* name);
void FDCAN_CheckGPIOPins(void);  /* GPIO引脚状态检测 */
void FDCAN2_GPIO_Test(void);     /* FDCAN2 GPIO连接测试 */
void FDCAN_HardwareDiagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_DIAGNOSTICS_H */
