/**
 ******************************************************************************
 * @file    phy_diagnostic_test.h
 * @brief   LAN8720 PHY硬件诊断测试头文件
 * @note    用于新板子硬件验证测试
 ******************************************************************************
 */

#ifndef __PHY_DIAGNOSTIC_TEST_H
#define __PHY_DIAGNOSTIC_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>

/* 测试结果枚举 */
typedef enum {
    PHY_TEST_OK = 0,
    PHY_TEST_FAIL = 1,
    PHY_TEST_TIMEOUT = 2,
    PHY_TEST_NO_CLOCK = 3,
    PHY_TEST_NO_RESPONSE = 4,
    PHY_TEST_WRONG_ID = 5,
    PHY_TEST_LINK_DOWN = 6
} PHY_TestResult_t;

/* 诊断报告结构 */
typedef struct {
    uint32_t test_timestamp;
    PHY_TestResult_t overall_result;

    /* REF_CLK测试 */
    uint8_t ref_clk_detected;       /* 是否检测到时钟 */
    uint32_t ref_clk_frequency;     /* 时钟频率 (Hz) */

    /* MDIO/MDC测试 */
    uint8_t mdio_mdc_ok;            /* MDIO/MDC通信是否正常 */
    uint32_t phy_id_read;           /* 读到的PHY ID */
    uint32_t phy_id_expected;       /* 预期的PHY ID */

    /* PHY寄存器测试 */
    uint32_t bcr_value;             /* Basic Control Register */
    uint32_t bsr_value;             /* Basic Status Register */
    uint8_t link_status;            /* Link状态 */
    uint8_t auto_neg_complete;      /* 自动协商完成 */

    /* 错误信息 */
    char error_msg[128];
} PHY_DiagnosticReport_t;

/* ==================== 测试函数声明 ==================== */

/**
 * @brief  执行完整的PHY诊断测试
 * @retval 测试结果
 */
PHY_TestResult_t PHY_RunFullDiagnostic(void);

/**
 * @brief  获取诊断报告
 * @param  report: 诊断报告结构指针
 * @retval 无
 */
void PHY_GetDiagnosticReport(PHY_DiagnosticReport_t *report);

/**
 * @brief  打印诊断报告到串口
 * @retval 无
 */
void PHY_PrintDiagnosticReport(void);

/**
 * @brief  测试REF_CLK信号 (需要示波器配合)
 * @note   此函数检查GPIO输入状态，但示波器测量更准确
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestRefClock(void);

/**
 * @brief  测试MDIO/MDC通信
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestMDIOCommunication(void);

/**
 * @brief  测试PHY ID读取
 * @param  phy_addr: PHY地址 (0x00或0x01)
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestPHYIDRead(uint8_t phy_addr);

/**
 * @brief  扫描所有PHY地址，查找响应的地址
 * @note   用于确认PHY地址配置
 * @retval 找到的PHY地址 (0xFF表示未找到)
 */
uint8_t PHY_ScanAllAddresses(void);

/**
 * @brief  测试PHY链路状态
 * @param  phy_addr: PHY地址
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestLinkStatus(uint8_t phy_addr);

/**
 * @brief  执行PHY寄存器读写测试
 * @param  phy_addr: PHY地址
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestRegisterReadWrite(uint8_t phy_addr);

#ifdef __cplusplus
}
#endif

#endif /* __PHY_DIAGNOSTIC_TEST_H */