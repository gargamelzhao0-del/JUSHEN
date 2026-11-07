/**
  ******************************************************************************
  * @file    fdcan_test.h
  * @brief   FDCAN回环测试头文件
  ******************************************************************************
  */

#ifndef __FDCAN_TEST_H
#define __FDCAN_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 测试函数声明 */
void FDCAN_RunLoopbackTest(void);
void FDCAN_ContinuousLoopbackTest(void);
void FDCAN_ManualSendTest(uint8_t channel, uint8_t *data);
void FDCAN_PrintConfiguration(void);
void FDCAN2_PollingReceiveTest(void);  /* 轮询接收测试 */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_TEST_H */
