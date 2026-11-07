/**
  ******************************************************************************
  * @file    fdcan_diagnostics.c
  * @brief   FDCAN硬件诊断工具实现
  ******************************************************************************
  */

#include "fdcan_diagnostics.h"
#include "fdcan.h"
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/**
  * @brief  打印FDCAN总线状态
  */
void FDCAN_PrintBusStatus(FDCAN_HandleTypeDef *hfdcan, const char* name)
{
  uint32_t psr = hfdcan->Instance->PSR;
  uint32_t ecr = hfdcan->Instance->ECR;
  uint32_t cccr = hfdcan->Instance->CCCR;

  printf("\r\n========== %s Bus Status ==========\r\n", name);

  /* 协议状态 */
  uint8_t lec = (psr >> 0) & 0x7;
  uint8_t act = (psr >> 3) & 0x3;
  uint8_t ep = (psr >> 5) & 0x1;
  uint8_t ew = (psr >> 6) & 0x1;
  uint8_t bo = (psr >> 7) & 0x1;

  printf("Protocol Status (PSR=0x%08X):\r\n", psr);
  printf("  LEC (Last Error): %u - ", lec);
  switch(lec) {
    case 0: printf("No Error\r\n"); break;
    case 1: printf("Stuff Error\r\n"); break;
    case 2: printf("Form Error\r\n"); break;
    case 3: printf("Ack Error ⚠️\r\n"); break;
    case 4: printf("Bit1 Error\r\n"); break;
    case 5: printf("Bit0 Error\r\n"); break;
    case 6: printf("CRC Error\r\n"); break;
    case 7: printf("No Change\r\n"); break;
  }

  printf("  Activity: %u - ", act);
  switch(act) {
    case 0: printf("Synchronizing\r\n"); break;
    case 1: printf("Idle\r\n"); break;
    case 2: printf("Receiver\r\n"); break;
    case 3: printf("Transmitter\r\n"); break;
  }

  if (ep) printf("  ⚠️  Error Passive Mode\r\n");
  if (ew) printf("  ⚠️  Error Warning\r\n");
  if (bo) printf("  ❌ Bus Off!\r\n");

  /* 错误计数器 */
  uint8_t tec = (ecr >> 0) & 0xFF;
  uint8_t rec = (ecr >> 8) & 0xFF;

  printf("\r\nError Counters (ECR=0x%08X):\r\n", ecr);
  printf("  TEC (TX Error): %u", tec);
  if (tec > 127) printf(" ⚠️  HIGH");
  printf("\r\n");
  printf("  REC (RX Error): %u", rec);
  if (rec > 127) printf(" ⚠️  HIGH");
  printf("\r\n");

  /* 控制寄存器 */
  uint8_t init = (cccr >> 0) & 0x1;
  uint8_t asm_bit = (cccr >> 2) & 0x1;
  uint8_t mon = (cccr >> 5) & 0x1;

  printf("\r\nControl Status (CCCR=0x%08X):\r\n", cccr);
  if (init) printf("  ⚠️  INIT Mode (not started)\r\n");
  if (asm_bit) printf("  ⚠️  Restricted Operation Mode (ASM=1)\r\n");
  if (mon) printf("  ⚠️  Bus Monitoring Mode (read-only)\r\n");

  /* TX/RX FIFO状态 */
  uint32_t txfqs = hfdcan->Instance->TXFQS;
  uint32_t rxf0s = hfdcan->Instance->RXF0S;

  uint32_t tx_free = (txfqs >> 0) & 0x3F;
  uint32_t rx_fill = (rxf0s >> 0) & 0x7F;

  printf("\r\nFIFO Status:\r\n");
  printf("  TX FIFO Free: %u/8", tx_free);
  if (tx_free == 0) printf(" ❌ FULL!");
  printf("\r\n");
  printf("  RX FIFO Fill: %u/8\r\n", rx_fill);

  printf("=====================================\r\n\r\n");
}

/**
  * @brief  检查FDCAN GPIO引脚状态
  * @note   用于诊断收发器电平不匹配问题
  */
void FDCAN_CheckGPIOPins(void)
{
  printf("\r\n========== FDCAN GPIO Pin Diagnostics ==========\r\n");

  /* FDCAN1 GPIO状态检查 */
  printf("\r\n[FDCAN1] PA11(RX), PA12(TX):\r\n");

  /* 读取PA11 (FDCAN1_RX) 状态 */
  GPIO_PinState rx1_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
  printf("  PA11 (RX): %s", rx1_state == GPIO_PIN_SET ? "HIGH" : "LOW");
  if (rx1_state == GPIO_PIN_RESET) {
    printf(" ⚠️  RX pin stuck LOW - possible issues:\r\n");
    printf("      1. MCP2562FD VIO not connected to 3.3V\r\n");
    printf("      2. MCP2562FD in Standby (STBY pin HIGH)\r\n");
    printf("      3. CAN bus shorted to GND\r\n");
    printf("      4. No bus activity (expected during idle)\r\n");
  } else {
    printf(" ✅ (recessive state)\r\n");
  }

  /* 读取PA12 (FDCAN1_TX) 状态 */
  GPIO_PinState tx1_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
  printf("  PA12 (TX): %s", tx1_state == GPIO_PIN_SET ? "HIGH" : "LOW");
  if (tx1_state == GPIO_PIN_RESET) {
    printf(" ⚠️  TX pin stuck LOW - check STM32 GPIO config\r\n");
  } else {
    printf(" ✅ (recessive state)\r\n");
  }

  /* FDCAN2 GPIO状态检查 */
  printf("\r\n[FDCAN2] PB5(RX), PB6(TX):\r\n");

  /* 读取PB5 (FDCAN2_RX) 状态 */
  GPIO_PinState rx2_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
  printf("  PB5 (RX): %s", rx2_state == GPIO_PIN_SET ? "HIGH" : "LOW");
  if (rx2_state == GPIO_PIN_RESET) {
    printf(" ⚠️  RX pin stuck LOW - possible issues:\r\n");
    printf("      1. MCP2562FD VIO not connected to 3.3V\r\n");
    printf("      2. MCP2562FD in Standby (STBY pin HIGH)\r\n");
    printf("      3. CAN bus shorted to GND\r\n");
    printf("      4. No bus activity (expected during idle)\r\n");
  } else {
    printf(" ✅ (recessive state)\r\n");
  }

  /* 读取PB6 (FDCAN2_TX) 状态 */
  GPIO_PinState tx2_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
  printf("  PB6 (TX): %s", tx2_state == GPIO_PIN_SET ? "HIGH" : "LOW");
  if (tx2_state == GPIO_PIN_RESET) {
    printf(" ⚠️  TX pin stuck LOW - check STM32 GPIO config\r\n");
  } else {
    printf(" ✅ (recessive state)\r\n");
  }

  /* VIO电平检查提示 */
  printf("\r\n[硬件检查清单]:\r\n");
  printf("  1. 检查MCP2562FD板上P1跳线帽位置:\r\n");
  printf("     Pin 1 (VIO) 必须连接到 Pin 3 (VCC3.3V)\r\n");
  printf("     ⚠️  如果VIO=5V会导致电平不匹配!\r\n");
  printf("\r\n");
  printf("  2. 使用万用表测量VIO引脚电压:\r\n");
  printf("     期望值: 3.3V (±0.15V)\r\n");
  printf("     错误值: 5V 或 0V (悬空)\r\n");
  printf("\r\n");
  printf("  3. 测量CANH-CANL电阻 (断电状态):\r\n");
  printf("     期望值: 60-70Ω (两个120Ω并联)\r\n");
  printf("     如果是120Ω说明只有一个终端电阻有效\r\n");
  printf("\r\n");
  printf("  4. 确认STBY引脚接GND (通过R3=0Ω):\r\n");
  printf("     STBY=0 → Normal Mode (正常工作)\r\n");
  printf("     STBY=1 → Standby Mode (待机,无法通信)\r\n");

  printf("=================================================\r\n\r\n");
}

/**
  * @brief  测试FDCAN2 GPIO连接
  * @note   手动切换GPIO来验证硬件连接
  */
void FDCAN2_GPIO_Test(void)
{
  printf("\r\n========== FDCAN2 GPIO Connection Test ==========\r\n");
  printf("This test will manually toggle PB6 (TX) to verify hardware connection\r\n\r\n");

  /* 临时配置PB6为普通GPIO输出模式 */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  printf("请使用示波器或万用表监测FDCAN2板的TXD引脚...\r\n");
  printf("PB6将以1Hz频率切换HIGH/LOW...\r\n\r\n");

  for (int i = 0; i < 10; i++) {
    /* 输出HIGH */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    printf("[%d] PB6 = HIGH (期望TXD引脚为HIGH)\r\n", i);
    HAL_Delay(500);

    /* 输出LOW */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    printf("[%d] PB6 = LOW  (期望TXD引脚为LOW)\r\n", i);
    HAL_Delay(500);
  }

  /* 恢复PB6为FDCAN复用功能 */
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  printf("\r\nGPIO测试完成，PB6已恢复为FDCAN2_TX功能\r\n");
  printf("=================================================\r\n\r\n");
}

/**
  * @brief  完整的FDCAN硬件诊断
  */
void FDCAN_HardwareDiagnostics(void)
{
  printf("\r\n");
  printf("╔════════════════════════════════════════════╗\r\n");
  printf("║  FDCAN Hardware Diagnostics v2.0           ║\r\n");
  printf("╚════════════════════════════════════════════╝\r\n");

  /* 打印两个FDCAN的状态 */
  FDCAN_PrintBusStatus(&hfdcan1, "FDCAN1");
  FDCAN_PrintBusStatus(&hfdcan2, "FDCAN2");

  /* 硬件连接检查清单 */
  printf("╔════════════════════════════════════════════╗\r\n");
  printf("║  Hardware Checklist                        ║\r\n");
  printf("╚════════════════════════════════════════════╝\r\n");
  printf("\r\n");
  printf("请检查以下硬件连接:\r\n");
  printf("\r\n");
  printf("1. 收发器供电:\r\n");
  printf("   □ 收发器1供电正常 (3.3V或5V)\r\n");
  printf("   □ 收发器2供电正常 (3.3V或5V)\r\n");
  printf("\r\n");
  printf("2. CAN总线连接:\r\n");
  printf("   □ FDCAN1_TX(PA12) → 收发器1 TXD\r\n");
  printf("   □ FDCAN1_RX(PA11) ← 收发器1 RXD\r\n");
  printf("   □ FDCAN2_TX(PB6)  → 收发器2 TXD\r\n");
  printf("   □ FDCAN2_RX(PB5)  ← 收发器2 RXD\r\n");
  printf("   □ 收发器1 CAN_H  ↔  收发器2 CAN_H\r\n");
  printf("   □ 收发器1 CAN_L  ↔  收发器2 CAN_L\r\n");
  printf("\r\n");
  printf("3. 终端电阻:\r\n");
  printf("   □ 收发器1端: 120Ω 在CAN_H和CAN_L之间\r\n");
  printf("   □ 收发器2端: 120Ω 在CAN_H和CAN_L之间\r\n");
  printf("   □ 用万用表测量CAN_H-CAN_L电阻应为60Ω\r\n");
  printf("\r\n");
  printf("4. 常见问题:\r\n");
  printf("   ⚠️  Ack Error: 总线上只有一个节点,需要另一端应答\r\n");
  printf("   ⚠️  ASM模式: 总线错误导致进入保护模式\r\n");
  printf("   ⚠️  TX FIFO满: 数据无法发送到总线上\r\n");
  printf("   ⚠️  没有终端电阻: 信号反射导致通信失败\r\n");
  printf("\r\n");
  printf("5. 测量建议:\r\n");
  printf("   • 用示波器测量CAN_H: 空闲时约2.5V\r\n");
  printf("   • 用示波器测量CAN_L: 空闲时约2.5V\r\n");
  printf("   • 差分电压CAN_H-CAN_L: 空闲时约0V\r\n");
  printf("   • 发送时CAN_H升高,CAN_L降低\r\n");
  printf("\r\n");
  printf("════════════════════════════════════════════\r\n");
}
