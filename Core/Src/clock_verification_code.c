/**
 * @file    clock_verification_code.c
 * @brief   STM32H743时钟配置运行时验证代码
 * @note    可以复制到main.c的USER CODE区域执行
 */

#include "main.h"
#include <stdio.h>

/**
 * @brief  打印详细的时钟配置信息
 * @note   在main()初始化后调用
 */
void print_detailed_clock_info(void)
{
    RCC_ClkInitTypeDef clkconfig;
    uint32_t flash_latency;
    uint32_t pll_source;

    printf("\r\n");
    printf("=============================================\r\n");
    printf("   STM32H743 Clock Configuration Report\r\n");
    printf("=============================================\r\n\r\n");

    /* 1. 系统时钟频率 */
    printf("1. SYSTEM CLOCKS:\r\n");
    printf("   SYSCLK:  %u MHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);
    printf("   HCLK:    %u MHz (AHB bus, CPU, DMA, memory)\r\n", HAL_RCC_GetHCLKFreq() / 1000000);

    /* 2. 外设时钟 */
    printf("\r\n2. PERIPHERAL CLOCKS:\r\n");
    printf("   PCLK1:   %u MHz (APB1: UART2-8, I2C, SPI2/3, TIM2-7)\r\n", HAL_RCC_GetPCLK1Freq() / 1000000);
    printf("   PCLK2:   %u MHz (APB2: UART1/6, SPI1/4-6, TIM1/8)\r\n", HAL_RCC_GetPCLK2Freq() / 1000000);

    /* 3. PLL配置 */
    printf("\r\n3. PLL CONFIGURATION:\r\n");

    /* 读取PLL配置寄存器 - STM32H7 specific */
    uint32_t pllckselr = RCC->PLLCKSELR;
    uint32_t pllm = (pllckselr & RCC_PLLCKSELR_DIVM1_Msk) >> RCC_PLLCKSELR_DIVM1_Pos;

    uint32_t pll1divr = RCC->PLL1DIVR;
    uint32_t plln = ((pll1divr & RCC_PLL1DIVR_N1_Msk) >> RCC_PLL1DIVR_N1_Pos) + 1;
    uint32_t pllp = ((pll1divr & RCC_PLL1DIVR_P1_Msk) >> RCC_PLL1DIVR_P1_Pos) + 1;
    uint32_t pllq = ((pll1divr & RCC_PLL1DIVR_Q1_Msk) >> RCC_PLL1DIVR_Q1_Pos) + 1;
    uint32_t pllr = ((pll1divr & RCC_PLL1DIVR_R1_Msk) >> RCC_PLL1DIVR_R1_Pos) + 1;

    /* 判断PLL时钟源 - STM32H7 uses PLLCKSELR register */
    pll_source = (pllckselr & RCC_PLLCKSELR_PLLSRC_Msk) >> RCC_PLLCKSELR_PLLSRC_Pos;
    const char *pll_src_name;
    uint32_t pll_input_freq;

    switch(pll_source) {
        case 0: pll_src_name = "None"; pll_input_freq = 0; break;
        case 1: pll_src_name = "HSI (64MHz)"; pll_input_freq = 64000000; break;
        case 2: pll_src_name = "CSI (4MHz)"; pll_input_freq = 4000000; break;
        case 3: pll_src_name = "HSE (25MHz)"; pll_input_freq = 25000000; break;
        default: pll_src_name = "Unknown"; pll_input_freq = 0; break;
    }

    printf("   PLL Source: %s\r\n", pll_src_name);
    printf("   PLLM (divider):    %lu\r\n", (unsigned long)pllm);
    printf("   PLLN (multiplier): %lu\r\n", (unsigned long)plln);
    printf("   PLLP (divider):    %lu\r\n", (unsigned long)pllp);
    printf("   PLLQ (divider):    %lu\r\n", (unsigned long)pllq);
    printf("   PLLR (divider):    %lu\r\n", (unsigned long)pllr);

    /* 计算PLL频率 */
    if (pll_input_freq > 0 && pllm > 0) {
        uint32_t vco_input = pll_input_freq / pllm;
        uint32_t vco_output = vco_input * plln;
        uint32_t pll_p_out = vco_output / pllp;
        uint32_t pll_q_out = vco_output / pllq;

        printf("\r\n   PLL Calculation:\r\n");
        printf("   VCO Input:  %u MHz (must be 8-16 MHz)\r\n", vco_input / 1000000);
        printf("   VCO Output: %u MHz (must be 192-960 MHz)\r\n", vco_output / 1000000);
        printf("   PLL_P Out:  %u MHz (SYSCLK)\r\n", pll_p_out / 1000000);
        printf("   PLL_Q Out:  %u MHz\r\n", pll_q_out / 1000000);

        /* 验证范围 */
        if (vco_input < 8000000 || vco_input > 16000000) {
            printf("   ⚠️  WARNING: VCO input out of range!\r\n");
        } else {
            printf("   ✅ VCO input in valid range\r\n");
        }

        if (vco_output < 192000000 || vco_output > 960000000) {
            printf("   ⚠️  WARNING: VCO output out of range!\r\n");
        } else {
            printf("   ✅ VCO output in valid range\r\n");
        }
    }

    /* 4. 时钟分频器配置 */
    printf("\r\n4. CLOCK DIVIDERS:\r\n");
    HAL_RCC_GetClockConfig(&clkconfig, &flash_latency);

    printf("   SYSCLK Divider: %u\r\n", clkconfig.SYSCLKDivider);
    printf("   AHB Divider:    %u\r\n",
           (clkconfig.AHBCLKDivider == RCC_HCLK_DIV2) ? 2 : 1);
    printf("   APB1 Divider:   %u\r\n",
           (clkconfig.APB1CLKDivider == RCC_APB1_DIV2) ? 2 : 1);
    printf("   APB2 Divider:   %u\r\n",
           (clkconfig.APB2CLKDivider == RCC_APB2_DIV2) ? 2 : 1);
    printf("   APB3 Divider:   %u\r\n",
           (clkconfig.APB3CLKDivider == RCC_APB3_DIV2) ? 2 : 1);
    printf("   APB4 Divider:   %u\r\n",
           (clkconfig.APB4CLKDivider == RCC_APB4_DIV2) ? 2 : 1);

    /* 5. Flash延迟 */
    printf("\r\n5. FLASH CONFIGURATION:\r\n");
    printf("   Flash Latency: %u wait states\r\n", flash_latency);

    /* Flash延迟表 (VOS0) */
    const char *latency_status;
    uint32_t hclk_mhz = HAL_RCC_GetHCLKFreq() / 1000000;

    if (hclk_mhz <= 70 && flash_latency == 0) latency_status = "✅ Correct";
    else if (hclk_mhz <= 140 && flash_latency == 1) latency_status = "✅ Correct";
    else if (hclk_mhz <= 185 && flash_latency == 2) latency_status = "✅ Correct";
    else if (hclk_mhz <= 210 && flash_latency == 3) latency_status = "✅ Correct";
    else if (hclk_mhz <= 240 && flash_latency == 4) latency_status = "✅ Correct";
    else latency_status = "⚠️  Check configuration";

    printf("   Status: %s for %u MHz HCLK\r\n", latency_status, hclk_mhz);

    /* 6. 电压标度 */
    printf("\r\n6. POWER CONFIGURATION:\r\n");
    uint32_t vos = (PWR->D3CR & PWR_D3CR_VOS) >> PWR_D3CR_VOS_Pos;
    const char *vos_name;
    const char *vos_max_freq;

    switch(vos) {
        case 3: vos_name = "VOS0 (Highest performance)"; vos_max_freq = "480 MHz"; break;
        case 2: vos_name = "VOS1 (High performance)"; vos_max_freq = "400 MHz"; break;
        case 1: vos_name = "VOS2 (Medium performance)"; vos_max_freq = "300 MHz"; break;
        case 0: vos_name = "VOS3 (Low power)"; vos_max_freq = "200 MHz"; break;
        default: vos_name = "Unknown"; vos_max_freq = "Unknown"; break;
    }

    printf("   Voltage Scale: %s\r\n", vos_name);
    printf("   Max Frequency: %s\r\n", vos_max_freq);

    /* 验证频率是否超出VOS限制 */
    uint32_t sysclk_mhz = HAL_RCC_GetSysClockFreq() / 1000000;
    if ((vos == 3 && sysclk_mhz <= 480) ||
        (vos == 2 && sysclk_mhz <= 400) ||
        (vos == 1 && sysclk_mhz <= 300) ||
        (vos == 0 && sysclk_mhz <= 200)) {
        printf("   ✅ SYSCLK within VOS limit\r\n");
    } else {
        printf("   ⚠️  WARNING: SYSCLK exceeds VOS limit!\r\n");
    }

    /* 7. SystemCoreClock变量 */
    printf("\r\n7. SYSTEM VARIABLES:\r\n");
    printf("   SystemCoreClock: %u MHz\r\n", SystemCoreClock / 1000000);
    printf("   SystemD2Clock:   %u MHz\r\n", SystemD2Clock / 1000000);

    /* 验证SystemCoreClock是否正确 */
    if (SystemCoreClock == HAL_RCC_GetSysClockFreq()) {
        printf("   ✅ SystemCoreClock matches SYSCLK\r\n");
    } else {
        printf("   ⚠️  WARNING: SystemCoreClock mismatch!\r\n");
        printf("   Call SystemCoreClockUpdate() to fix\r\n");
    }

    printf("\r\n=============================================\r\n");
    printf("   Clock Verification Complete\r\n");
    printf("=============================================\r\n\r\n");
}

/**
 * @brief  使用SysTick测试时钟精度
 * @note   测试1秒延时的精度
 */
void test_systick_accuracy(void)
{
    printf("\r\n=== SysTick Accuracy Test ===\r\n");
    printf("Testing 1000ms delay...\r\n");

    uint32_t start_tick = HAL_GetTick();
    HAL_Delay(1000);  // 延时1秒
    uint32_t end_tick = HAL_GetTick();
    uint32_t elapsed = end_tick - start_tick;

    printf("Expected: 1000 ms\r\n");
    printf("Actual:   %u ms\r\n", elapsed);
    printf("Error:    %d ms (%.2f%%)\r\n",
           (int32_t)elapsed - 1000,
           ((float)elapsed - 1000.0f) / 10.0f);

    if (elapsed >= 999 && elapsed <= 1001) {
        printf("✅ SysTick accuracy is good!\r\n");
    } else {
        printf("⚠️  WARNING: SysTick may have timing issues\r\n");
    }
    printf("\r\n");
}

/**
 * @brief  计算并显示理论最大性能
 * @note   基于当前时钟配置
 */
void print_performance_metrics(void)
{
    uint32_t sysclk = HAL_RCC_GetSysClockFreq();
    uint32_t hclk = HAL_RCC_GetHCLKFreq();

    printf("\r\n=== Performance Metrics ===\r\n");
    printf("CPU Clock:      %u MHz\r\n", hclk / 1000000);
    printf("CPU Performance:\r\n");
    printf("  - Max DMIPS:   ~%u DMIPS (estimated)\r\n",
           (hclk / 1000000) * 5 / 4);  // Cortex-M7约1.25 DMIPS/MHz
    printf("  - Instructions: ~%u MIPS\r\n", hclk / 1000000);

    printf("\r\nMemory Bandwidth:\r\n");
    printf("  - AXI SRAM:    ~%u MB/s (estimated)\r\n", hclk / 1000000 * 4);
    printf("  - DTCM:        ~%u MB/s (zero wait state)\r\n", hclk / 1000000 * 8);

    printf("\r\nPower Consumption (typical):\r\n");
    printf("  - VOS0 @ 480MHz: ~350-400 mA\r\n");
    printf("  - VOS1 @ 400MHz: ~300-350 mA\r\n");
    printf("  - VOS2 @ 300MHz: ~250-300 mA\r\n");
    printf("\r\n");
}

/**
 * @brief  完整的时钟验证测试
 * @note   在main()的USER CODE BEGIN 2区域调用
 */
void run_complete_clock_verification(void)
{
    printf("\r\n");
    printf("*************************************************\r\n");
    printf("*  STM32H743 CLOCK VERIFICATION SUITE          *\r\n");
    printf("*************************************************\r\n");

    /* 1. 打印详细时钟配置 */
    print_detailed_clock_info();

    /* 2. 测试SysTick精度 */
    test_systick_accuracy();

    /* 3. 显示性能指标 */
    print_performance_metrics();

    printf("*************************************************\r\n");
    printf("*  Verification Complete                        *\r\n");
    printf("*************************************************\r\n\r\n");
}

/* ===== 使用示例 =====

在main.c中添加:

// USER CODE BEGIN 2

  // 等待串口稳定
  HAL_Delay(100);

  // 运行完整的时钟验证
  run_complete_clock_verification();

// USER CODE END 2

*/
