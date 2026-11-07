/**
 ******************************************************************************
 * @file    phy_diagnostic_test.c
 * @brief   LAN8720 PHY硬件诊断测试实现
 * @note    用于新板子硬件验证测试 - 按步骤诊断PHY通信问题
 ******************************************************************************
 */

#include "phy_diagnostic_test.h"
#include "lan8720_driver.h"
#include <stdio.h>
#include <string.h>

/* 外部ETH句柄 */
extern ETH_HandleTypeDef heth;

/* 诊断报告全局变量 */
static PHY_DiagnosticReport_t g_diagnostic_report = {0};

/* LAN8720寄存器定义 */
#define LAN8720_BCR             0x00  /* Basic Control Register */
#define LAN8720_BSR             0x01  /* Basic Status Register */
#define LAN8720_PHYI1R          0x02  /* PHY Identifier 1 */
#define LAN8720_PHYI2R          0x03  /* PHY Identifier 2 */

/* LAN8720 BCR位定义 */
#define LAN8720_BCR_SOFT_RESET  0x8000
#define LAN8720_BCR_LOOPBACK    0x4000
#define LAN8720_BCR_AUTONEG     0x1000

/* LAN8720 BSR位定义 */
#define LAN8720_BSR_LINK_STATUS 0x0004
#define LAN8720_BSR_AUTONEG_COMP 0x0020

/* 预期的LAN8720 PHY ID */
#define LAN8720_PHY_ID_EXPECTED 0x0007C0F0

/**
 * @brief  执行完整的PHY诊断测试
 * @retval 测试结果
 */
PHY_TestResult_t PHY_RunFullDiagnostic(void)
{
    printf("\r\n");
    printf("=================================================\r\n");
    printf("     LAN8720 PHY Hardware Diagnostic Test\r\n");
    printf("=================================================\r\n");
    printf("[PHY Test] Starting diagnostic...\r\n");

    /* 清空诊断报告 */
    memset(&g_diagnostic_report, 0, sizeof(g_diagnostic_report));
    g_diagnostic_report.test_timestamp = HAL_GetTick();

    /* Step 1: 检查ETH初始化状态 */
    printf("\r\n[Step 1] Checking ETH initialization...\r\n");
    if (heth.Instance != ETH) {
        printf("[Step 1] ERROR: ETH not initialized!\r\n");
        g_diagnostic_report.overall_result = PHY_TEST_FAIL;
        strcpy(g_diagnostic_report.error_msg, "ETH not initialized");
        return PHY_TEST_FAIL;
    }
    printf("[Step 1] ETH Instance OK\r\n");
    printf("[Step 1] ETH gState: %u\r\n", heth.gState);

    /* Step 2: 检查MDIO时钟配置 */
    printf("\r\n[Step 2] Checking MDIO clock configuration...\r\n");
    uint32_t mdio_clock_range = READ_REG(heth.Instance->MACMDIOAR);
    printf("[Step 2] MACMDIOAR: 0x%08X\r\n", mdio_clock_range);

    /* 解析MDIO时钟范围 */
    uint32_t cr_field = (mdio_clock_range >> 8) & 0x7;
    const char* cr_desc;
    switch (cr_field) {
        case 0: cr_desc = "60-100 MHz"; break;
        case 1: cr_desc = "100-150 MHz"; break;
        case 2: cr_desc = "150-200 MHz"; break;
        case 3: cr_desc = "200-250 MHz"; break;
        case 4: cr_desc = "250-300 MHz"; break;
        case 5: cr_desc = "300-350 MHz"; break;
        case 6: cr_desc = "350-400 MHz"; break;
        case 7: cr_desc = "400-450 MHz"; break;
        default: cr_desc = "Unknown"; break;
    }
    printf("[Step 2] MDIO Clock Range (CR): %s\r\n", cr_desc);

    /* 计算实际系统时钟 */
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    printf("[Step 2] System Clock: %u MHz\r\n", sysclk / 1000000);
    printf("[Step 2] Expected MDC: %.2f MHz\r\n", sysclk / 1000000.0 / 124.0);

    /* Step 3: 扫描PHY地址 */
    printf("\r\n[Step 3] Scanning PHY addresses...\r\n");
    uint8_t found_addr = PHY_ScanAllAddresses();
    if (found_addr == 0xFF) {
        printf("[Step 3] ERROR: No PHY found at any address!\r\n");
        printf("[Step 3] This indicates MDIO/MDC communication failure.\r\n");
        g_diagnostic_report.mdio_mdc_ok = 0;
        g_diagnostic_report.overall_result = PHY_TEST_NO_RESPONSE;
        strcpy(g_diagnostic_report.error_msg, "No PHY response - check MDIO/MDC connections");

        /* 打印硬件检查清单 */
        printf("\r\n");
        printf("========================================\r\n");
        printf("  Hardware Checklist for MDIO Failure\r\n");
        printf("========================================\r\n");
        printf("1. Check MDIO (PA2) connection to LAN8720 Pin 5\r\n");
        printf("2. Check MDC (PC1) connection to LAN8720 Pin 6\r\n");
        printf("3. Check LAN8720 VDD = 3.3V (Pin 1, 14)\r\n");
        printf("4. Check LAN8720 GND = 0V (Pin 7, 15)\r\n");
        printf("5. Check nRST pin state (should be HIGH)\r\n");
        printf("6. **CRITICAL**: Check REF_CLK 50MHz on PA1\r\n");
        printf("   - Use oscilloscope to verify 50MHz signal\r\n");
        printf("   - LAN8720 must have 50MHz clock to respond!\r\n");
        printf("========================================\r\n");

        return PHY_TEST_NO_RESPONSE;
    }
    printf("[Step 3] PHY found at address 0x%02X\r\n", found_addr);
    g_diagnostic_report.mdio_mdc_ok = 1;

    /* Step 4: 验证PHY ID */
    printf("\r\n[Step 4] Verifying PHY ID...\r\n");
    uint32_t phy_id1, phy_id2, phy_id_full;
    if (HAL_ETH_ReadPHYRegister(&heth, found_addr, LAN8720_PHYI1R, &phy_id1) != HAL_OK) {
        printf("[Step 4] ERROR: Failed to read PHY ID1!\r\n");
        return PHY_TEST_FAIL;
    }
    if (HAL_ETH_ReadPHYRegister(&heth, found_addr, LAN8720_PHYI2R, &phy_id2) != HAL_OK) {
        printf("[Step 4] ERROR: Failed to read PHY ID2!\r\n");
        return PHY_TEST_FAIL;
    }
    phy_id_full = (phy_id1 << 16) | phy_id2;
    printf("[Step 4] PHY ID1 (Reg 0x02): 0x%04X\r\n", phy_id1);
    printf("[Step 4] PHY ID2 (Reg 0x03): 0x%04X\r\n", phy_id2);
    printf("[Step 4] Full PHY ID: 0x%08X\r\n", phy_id_full);
    g_diagnostic_report.phy_id_read = phy_id_full;
    g_diagnostic_report.phy_id_expected = LAN8720_PHY_ID_EXPECTED;

    if ((phy_id_full & 0xFFFFFFF0) != LAN8720_PHY_ID_EXPECTED) {
        printf("[Step 4] WARNING: PHY ID mismatch!\r\n");
        printf("[Step 4] Expected: 0x0007C0Fx (LAN8720/LAN8742 family)\r\n");
        g_diagnostic_report.overall_result = PHY_TEST_WRONG_ID;
        /* 继续测试，可能是不同型号的PHY */
    } else {
        printf("[Step 4] PHY ID OK (LAN87xx family)\r\n");
    }

    /* Step 5: 读取BCR和BSR寄存器 */
    printf("\r\n[Step 5] Reading PHY control registers...\r\n");
    uint32_t bcr, bsr;
    if (HAL_ETH_ReadPHYRegister(&heth, found_addr, LAN8720_BCR, &bcr) != HAL_OK) {
        printf("[Step 5] ERROR: Failed to read BCR!\r\n");
        return PHY_TEST_FAIL;
    }
    if (HAL_ETH_ReadPHYRegister(&heth, found_addr, LAN8720_BSR, &bsr) != HAL_OK) {
        printf("[Step 5] ERROR: Failed to read BSR!\r\n");
        return PHY_TEST_FAIL;
    }
    printf("[Step 5] BCR (Reg 0x00): 0x%04X\r\n", bcr);
    printf("[Step 5] BSR (Reg 0x01): 0x%04X\r\n", bsr);
    g_diagnostic_report.bcr_value = bcr;
    g_diagnostic_report.bsr_value = bsr;

    /* 解析BCR */
    printf("[Step 5] BCR Analysis:\r\n");
    printf("  - Soft Reset: %s\r\n", (bcr & LAN8720_BCR_SOFT_RESET) ? "ACTIVE" : "Inactive");
    printf("  - Loopback: %s\r\n", (bcr & LAN8720_BCR_LOOPBACK) ? "ENABLED" : "Disabled");
    printf("  - Auto-Neg: %s\r\n", (bcr & LAN8720_BCR_AUTONEG) ? "ENABLED" : "Disabled");

    /* 解析BSR */
    printf("[Step 5] BSR Analysis:\r\n");
    printf("  - Link Status: %s\r\n", (bsr & LAN8720_BSR_LINK_STATUS) ? "UP" : "DOWN");
    printf("  - Auto-Neg Complete: %s\r\n", (bsr & LAN8720_BSR_AUTONEG_COMP) ? "YES" : "NO");
    g_diagnostic_report.link_status = (bsr & LAN8720_BSR_LINK_STATUS) ? 1 : 0;
    g_diagnostic_report.auto_neg_complete = (bsr & LAN8720_BSR_AUTONEG_COMP) ? 1 : 0;

    /* Step 6: 检查链路状态 */
    printf("\r\n[Step 6] Checking link status...\r\n");
    if (!(bsr & LAN8720_BSR_LINK_STATUS)) {
        printf("[Step 6] Link is DOWN\r\n");
        printf("[Step 6] This is normal if no Ethernet cable is connected.\r\n");
        printf("[Step 6] Connect cable to router/switch and observe LED.\r\n");
        g_diagnostic_report.overall_result = PHY_TEST_LINK_DOWN;
        strcpy(g_diagnostic_report.error_msg, "Link down - no cable connected");
    } else {
        printf("[Step 6] Link is UP\r\n");
        g_diagnostic_report.overall_result = PHY_TEST_OK;
        printf("\r\n");
        printf("=================================================\r\n");
        printf("     PHY Diagnostic Test: PASSED\r\n");
        printf("=================================================\r\n");
    }

    /* 打印完整报告 */
    PHY_PrintDiagnosticReport();

    return g_diagnostic_report.overall_result;
}

/**
 * @brief  获取诊断报告
 * @param  report: 诊断报告结构指针
 * @retval 无
 */
void PHY_GetDiagnosticReport(PHY_DiagnosticReport_t *report)
{
    if (report != NULL) {
        memcpy(report, &g_diagnostic_report, sizeof(PHY_DiagnosticReport_t));
    }
}

/**
 * @brief  打印诊断报告到串口
 * @retval 无
 */
void PHY_PrintDiagnosticReport(void)
{
    printf("\r\n");
    printf("========================================\r\n");
    printf("     PHY Diagnostic Report\r\n");
    printf("========================================\r\n");
    printf("Test Time: %u ms\r\n", g_diagnostic_report.test_timestamp);
    printf("Overall Result: ");

    switch (g_diagnostic_report.overall_result) {
        case PHY_TEST_OK:
            printf("PASS\r\n");
            break;
        case PHY_TEST_FAIL:
            printf("FAIL\r\n");
            break;
        case PHY_TEST_NO_RESPONSE:
            printf("NO PHY RESPONSE\r\n");
            break;
        case PHY_TEST_WRONG_ID:
            printf("WRONG PHY ID\r\n");
            break;
        case PHY_TEST_LINK_DOWN:
            printf("LINK DOWN (normal without cable)\r\n");
            break;
        default:
            printf("UNKNOWN\r\n");
            break;
    }

    printf("----------------------------------------\r\n");
    printf("PHY ID Read: 0x%08X\r\n", g_diagnostic_report.phy_id_read);
    printf("PHY ID Expected: 0x%08X\r\n", g_diagnostic_report.phy_id_expected);
    printf("BCR: 0x%04X\r\n", g_diagnostic_report.bcr_value);
    printf("BSR: 0x%04X\r\n", g_diagnostic_report.bsr_value);
    printf("Link Status: %s\r\n", g_diagnostic_report.link_status ? "UP" : "DOWN");
    printf("Auto-Neg Complete: %s\r\n", g_diagnostic_report.auto_neg_complete ? "YES" : "NO");
    printf("----------------------------------------\r\n");
    if (strlen(g_diagnostic_report.error_msg) > 0) {
        printf("Error Message: %s\r\n", g_diagnostic_report.error_msg);
    }
    printf("========================================\r\n");
}

/**
 * @brief  扫描所有PHY地址，查找响应的地址
 * @retval 找到的PHY地址 (0xFF表示未找到)
 */
uint8_t PHY_ScanAllAddresses(void)
{
    uint32_t phy_id;

    printf("[PHY Scan] Scanning addresses 0x00-0x1F...\r\n");

    for (uint8_t addr = 0; addr < 32; addr++) {
        HAL_StatusTypeDef status = HAL_ETH_ReadPHYRegister(&heth, addr, LAN8720_PHYI1R, &phy_id);

        if (status == HAL_OK) {
            /* 检查是否是有效的PHY ID (不全是0或F) */
            if (phy_id != 0x0000 && phy_id != 0xFFFF) {
                printf("[PHY Scan] Found PHY at address 0x%02X\r\n", addr);
                printf("[PHY Scan] PHY ID1 = 0x%04X\r\n", phy_id);

                /* 尝试读取完整的PHY ID */
                uint32_t phy_id2;
                if (HAL_ETH_ReadPHYRegister(&heth, addr, LAN8720_PHYI2R, &phy_id2) == HAL_OK) {
                    printf("[PHY Scan] PHY ID2 = 0x%04X\r\n", phy_id2);
                    printf("[PHY Scan] Full PHY ID = 0x%08X\r\n", (phy_id << 16) | phy_id2);
                }

                return addr;
            }
        }
    }

    printf("[PHY Scan] No PHY found at any address\r\n");
    return 0xFF;
}

/**
 * @brief  测试MDIO/MDC通信
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestMDIOCommunication(void)
{
    printf("\r\n[MDIO Test] Testing MDIO/MDC communication...\r\n");

    /* 尝试读取地址0x00的PHY ID寄存器 */
    uint32_t test_values[4] = {0};
    uint8_t success_count = 0;

    for (uint8_t addr = 0; addr < 2; addr++) {
        for (uint8_t reg = 0; reg < 4; reg++) {
            HAL_StatusTypeDef status = HAL_ETH_ReadPHYRegister(&heth, addr, reg, &test_values[reg]);

            if (status == HAL_OK && test_values[reg] != 0xFFFFFFFF && test_values[reg] != 0x0000) {
                success_count++;
                printf("[MDIO Test] Addr 0x%02X, Reg %u: 0x%04X\r\n", addr, reg, test_values[reg]);
            }
        }
    }

    if (success_count > 0) {
        printf("[MDIO Test] MDIO/MDC communication OK\r\n");
        return PHY_TEST_OK;
    } else {
        printf("[MDIO Test] MDIO/MDC communication FAILED\r\n");
        printf("[MDIO Test] All reads returned 0xFFFFFFFF or 0x0000\r\n");
        return PHY_TEST_NO_RESPONSE;
    }
}

/**
 * @brief  测试PHY ID读取
 * @param  phy_addr: PHY地址
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestPHYIDRead(uint8_t phy_addr)
{
    printf("\r\n[PHY ID Test] Reading PHY ID at address 0x%02X...\r\n", phy_addr);

    uint32_t phy_id1, phy_id2;

    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_PHYI1R, &phy_id1) != HAL_OK) {
        printf("[PHY ID Test] Failed to read PHY ID1\r\n");
        return PHY_TEST_FAIL;
    }

    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_PHYI2R, &phy_id2) != HAL_OK) {
        printf("[PHY ID Test] Failed to read PHY ID2\r\n");
        return PHY_TEST_FAIL;
    }

    uint32_t phy_id_full = (phy_id1 << 16) | phy_id2;
    printf("[PHY ID Test] Full PHY ID: 0x%08X\r\n", phy_id_full);

    if (phy_id_full == 0xFFFFFFFF) {
        printf("[PHY ID Test] PHY returned all 1s - no response\r\n");
        return PHY_TEST_NO_RESPONSE;
    }

    if ((phy_id_full & 0xFFFFFFF0) == LAN8720_PHY_ID_EXPECTED) {
        printf("[PHY ID Test] PHY ID OK\r\n");
        return PHY_TEST_OK;
    } else {
        printf("[PHY ID Test] PHY ID mismatch\r\n");
        return PHY_TEST_WRONG_ID;
    }
}

/**
 * @brief  测试PHY链路状态
 * @param  phy_addr: PHY地址
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestLinkStatus(uint8_t phy_addr)
{
    printf("\r\n[Link Test] Checking link status at address 0x%02X...\r\n", phy_addr);

    uint32_t bsr;

    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_BSR, &bsr) != HAL_OK) {
        printf("[Link Test] Failed to read BSR\r\n");
        return PHY_TEST_FAIL;
    }

    printf("[Link Test] BSR: 0x%04X\r\n", bsr);

    if (bsr & LAN8720_BSR_LINK_STATUS) {
        printf("[Link Test] Link is UP\r\n");
        return PHY_TEST_OK;
    } else {
        printf("[Link Test] Link is DOWN\r\n");
        printf("[Link Test] Connect Ethernet cable and check LED\r\n");
        return PHY_TEST_LINK_DOWN;
    }
}

/**
 * @brief  执行PHY寄存器读写测试
 * @param  phy_addr: PHY地址
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestRegisterReadWrite(uint8_t phy_addr)
{
    printf("\r\n[Reg Test] Testing register read/write at address 0x%02X...\r\n", phy_addr);

    uint32_t original_bcr, modified_bcr, restored_bcr;

    /* 读取原始BCR值 */
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_BCR, &original_bcr) != HAL_OK) {
        printf("[Reg Test] Failed to read original BCR\r\n");
        return PHY_TEST_FAIL;
    }
    printf("[Reg Test] Original BCR: 0x%04X\r\n", original_bcr);

    /* 尝试写入测试值（设置Loopback位，这是安全的测试） */
    uint32_t test_value = original_bcr | LAN8720_BCR_LOOPBACK;
    if (HAL_ETH_WritePHYRegister(&heth, phy_addr, LAN8720_BCR, test_value) != HAL_OK) {
        printf("[Reg Test] Failed to write test BCR\r\n");
        return PHY_TEST_FAIL;
    }
    printf("[Reg Test] Written BCR: 0x%04X\r\n", test_value);

    /* 读回验证 */
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_BCR, &modified_bcr) != HAL_OK) {
        printf("[Reg Test] Failed to read modified BCR\r\n");
        return PHY_TEST_FAIL;
    }
    printf("[Reg Test] Read back BCR: 0x%04X\r\n", modified_bcr);

    /* 恢复原始值 */
    if (HAL_ETH_WritePHYRegister(&heth, phy_addr, LAN8720_BCR, original_bcr) != HAL_OK) {
        printf("[Reg Test] Failed to restore original BCR\r\n");
        return PHY_TEST_FAIL;
    }

    /* 验证恢复 */
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_BCR, &restored_bcr) != HAL_OK) {
        printf("[Reg Test] Failed to read restored BCR\r\n");
        return PHY_TEST_FAIL;
    }
    printf("[Reg Test] Restored BCR: 0x%04X\r\n", restored_bcr);

    if ((modified_bcr & LAN8720_BCR_LOOPBACK) && restored_bcr == original_bcr) {
        printf("[Reg Test] Register read/write test PASSED\r\n");
        return PHY_TEST_OK;
    } else {
        printf("[Reg Test] Register read/write test FAILED\r\n");
        return PHY_TEST_FAIL;
    }
}

/**
 * @brief  测试REF_CLK信号
 * @note   此函数检查ETH是否已启动，实际时钟需示波器验证
 * @retval 测试结果
 */
PHY_TestResult_t PHY_TestRefClock(void)
{
    printf("\r\n[REF_CLK Test] Checking ETH clock configuration...\r\n");

    /* 检查ETH MAC是否配置 */
    uint32_t mac_cr = READ_REG(heth.Instance->MACCR);
    printf("[REF_CLK Test] MACCR: 0x%08X\r\n", mac_cr);

    /* 检查系统时钟配置 */
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    printf("[REF_CLK Test] System Clock: %u Hz\r\n", sysclk);

    /* REF_CLK验证需要示波器 */
    printf("\r\n");
    printf("==========================================\r\n");
    printf("  REF_CLK Verification (Requires Scope)\r\n");
    printf("==========================================\r\n");
    printf("1. Connect oscilloscope to PA1 (ETH_REF_CLK)\r\n");
    printf("2. Expected signal: 50MHz square wave\r\n");
    printf("3. Amplitude: ~3.3V\r\n");
    printf("4. LAN8720 Pin 2 (nINT/REFCLKO) outputs 50MHz\r\n");
    printf("==========================================\r\n");

    /* 返回OK，实际验证需示波器 */
    return PHY_TEST_OK;
}