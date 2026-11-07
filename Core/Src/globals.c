#include "main.h"

/* 全局变量定义 */
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
UART_HandleTypeDef huart1;
/* OLD: MotorHandle g_motor; - No longer used with new quadruped system */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;