/**
 ****************************************************************************************************
 * @file        fdcan.h
 * @version     V1.0
 * @brief       FDCAN 驱动代码
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 ****************************************************************************************************
 */

#ifndef __CAN_H
#define __CAN_H

#include "main.h"


/******************************************************************************************/
/* FDCAN 引脚 定义 */

/* FDCAN1引脚定义 (PB8/PB9) - ✅ V3.0: 更换引脚 */
#define FDCAN1_RX_GPIO_PORT                GPIOB
#define FDCAN1_RX_GPIO_PIN                 GPIO_PIN_8
#define FDCAN1_RX_GPIO_AF                  GPIO_AF9_FDCAN1
#define FDCAN1_RX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define FDCAN1_TX_GPIO_PORT                GPIOB
#define FDCAN1_TX_GPIO_PIN                 GPIO_PIN_9
#define FDCAN1_TX_GPIO_AF                  GPIO_AF9_FDCAN1
#define FDCAN1_TX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* FDCAN2引脚定义 (PB5/PB6) - ✅ 使用备用引脚 */
#define FDCAN2_RX_GPIO_PORT                GPIOB
#define FDCAN2_RX_GPIO_PIN                 GPIO_PIN_5
#define FDCAN2_RX_GPIO_AF                  GPIO_AF9_FDCAN2
#define FDCAN2_RX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

#define FDCAN2_TX_GPIO_PORT                GPIOB
#define FDCAN2_TX_GPIO_PIN                 GPIO_PIN_6
#define FDCAN2_TX_GPIO_AF                  GPIO_AF9_FDCAN2
#define FDCAN2_TX_GPIO_CLK_ENABLE()        do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)

/* FDCAN接收RX0中断使能 */
#define FDCAN1_RX0_INT_ENABLE   1         /* ✅ 1,使能接收中断 */
#define FDCAN2_RX0_INT_ENABLE   1         /* ✅ 1,使能接收中断 */

/* ============== 修复: FDCAN Message RAM配置 ============== */
/* STM32H7 Message RAM总大小: 2560 WORDS (10240 bytes) */
#define FDCAN_MESSAGE_RAM_SIZE_WORDS   2560

/* FDCAN1配置参数 */
#define FDCAN1_STD_FILTERS_NBR         1
#define FDCAN1_EXT_FILTERS_NBR         1
#define FDCAN1_RX_FIFO0_ELMTS_NBR      3
#define FDCAN1_RX_FIFO0_ELMT_SIZE      18   /* 64字节 = 18 WORDS */
#define FDCAN1_TX_FIFO_ELMTS_NBR       8
#define FDCAN1_TX_ELMT_SIZE            2    /* 8字节 = 2 WORDS */

/* FDCAN2配置参数 */
#define FDCAN2_STD_FILTERS_NBR         1
#define FDCAN2_EXT_FILTERS_NBR         1
#define FDCAN2_RX_FIFO0_ELMTS_NBR      3
#define FDCAN2_RX_FIFO0_ELMT_SIZE      18   /* 64字节 = 18 WORDS */
#define FDCAN2_TX_FIFO_ELMTS_NBR       8
#define FDCAN2_TX_ELMT_SIZE            2    /* 8字节 = 2 WORDS */

/* 计算FDCAN1 Message RAM使用量 (WORDS) */
#define FDCAN1_RAM_FILTERS_STD         (FDCAN1_STD_FILTERS_NBR * 1)
#define FDCAN1_RAM_FILTERS_EXT         (FDCAN1_EXT_FILTERS_NBR * 2)
#define FDCAN1_RAM_RX_FIFO0            (FDCAN1_RX_FIFO0_ELMTS_NBR * FDCAN1_RX_FIFO0_ELMT_SIZE)
#define FDCAN1_RAM_TX_FIFO             (FDCAN1_TX_FIFO_ELMTS_NBR * FDCAN1_TX_ELMT_SIZE)
#define FDCAN1_RAM_TOTAL               (FDCAN1_RAM_FILTERS_STD + FDCAN1_RAM_FILTERS_EXT + \
                                        FDCAN1_RAM_RX_FIFO0 + FDCAN1_RAM_TX_FIFO)

/* 计算FDCAN2 Message RAM使用量 (WORDS) */
#define FDCAN2_RAM_FILTERS_STD         (FDCAN2_STD_FILTERS_NBR * 1)
#define FDCAN2_RAM_FILTERS_EXT         (FDCAN2_EXT_FILTERS_NBR * 2)
#define FDCAN2_RAM_RX_FIFO0            (FDCAN2_RX_FIFO0_ELMTS_NBR * FDCAN2_RX_FIFO0_ELMT_SIZE)
#define FDCAN2_RAM_TX_FIFO             (FDCAN2_TX_FIFO_ELMTS_NBR * FDCAN2_TX_ELMT_SIZE)
#define FDCAN2_RAM_TOTAL               (FDCAN2_RAM_FILTERS_STD + FDCAN2_RAM_FILTERS_EXT + \
                                        FDCAN2_RAM_RX_FIFO0 + FDCAN2_RAM_TX_FIFO)

/* FDCAN2偏移量 (带安全边距) */
#define FDCAN2_MESSAGE_RAM_OFFSET      ((FDCAN1_RAM_TOTAL + 64) & ~31)  /* 32字对齐，64字安全边距 */

/* 编译时验证: 确保Message RAM不溢出 */
#if (FDCAN1_RAM_TOTAL + FDCAN2_RAM_TOTAL + 128) > FDCAN_MESSAGE_RAM_SIZE_WORDS
    #error "FDCAN Message RAM overflow! Check configuration."
#endif

/* 编译时验证: 确保FDCAN2偏移量足够 */
#if FDCAN2_MESSAGE_RAM_OFFSET < FDCAN1_RAM_TOTAL
    #error "FDCAN2 Message RAM offset overlaps with FDCAN1!"
#endif

/******************************************************************************************/ 

/* FDCAN发送参数配置结构体 */
typedef struct {
    uint32_t identifier;          /* CAN ID (标准帧: 0x000~0x7FF, 扩展帧: 0x00000000~0x1FFFFFFF) */
    uint32_t id_type;             /* ID类型: FDCAN_STANDARD_ID 或 FDCAN_EXTENDED_ID */
    uint32_t frame_type;          /* 帧类型: FDCAN_DATA_FRAME 或 FDCAN_REMOTE_FRAME */
    uint32_t data_length;         /* 数据长度: FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_64 */
    uint32_t fd_format;           /* FD格式: FDCAN_CLASSIC_CAN 或 FDCAN_FD_CAN */
    uint32_t bit_rate_switch;     /* 位速率切换: FDCAN_BRS_OFF 或 FDCAN_BRS_ON */
} FDCAN_TxConfigTypeDef;

/* FDCAN1函数 */
uint8_t fdcan1_init(uint16_t presc, uint8_t tsjw, uint16_t ntseg1, uint8_t ntseg2, uint32_t mode);  /* FDCAN1初始化 */
uint8_t fdcan1_send_msg(uint8_t *msg, uint32_t len);                                                /* FDCAN1发送数据(默认配置) */
uint8_t fdcan1_send_msg_ex(uint8_t *msg, FDCAN_TxConfigTypeDef *config);                            /* FDCAN1发送数据(灵活配置) */
uint8_t fdcan1_send_batch(uint8_t *msgs[], uint32_t lens[], uint8_t count);                          /* FDCAN1批量发送多个消息 */
uint8_t fdcan1_receive_msg(uint8_t *buf);                                                           /* FDCAN1接收数据 */

/* FDCAN2函数 */
uint8_t fdcan2_init(uint16_t presc, uint8_t tsjw, uint16_t ntseg1, uint8_t ntseg2, uint32_t mode);  /* FDCAN2初始化 */
uint8_t fdcan2_send_msg(uint8_t *msg, uint32_t len);                                                /* FDCAN2发送数据(默认配置) */
uint8_t fdcan2_send_msg_ex(uint8_t *msg, FDCAN_TxConfigTypeDef *config);                            /* FDCAN2发送数据(灵活配置) */
uint8_t fdcan2_send_batch(uint8_t *msgs[], uint32_t lens[], uint8_t count);                          /* FDCAN2批量发送多个消息 */
uint8_t fdcan2_receive_msg(uint8_t *buf);                                                           /* FDCAN2接收数据 */

/* 兼容性函数 (默认使用FDCAN1) */
uint8_t fdcan_init(uint16_t presc, uint8_t tsjw, uint16_t ntseg1, uint8_t ntseg2, uint32_t mode);
uint8_t fdcan_send_msg(uint8_t *msg, uint32_t len);
uint8_t fdcan_send_batch(uint8_t *msgs[], uint32_t lens[], uint8_t count);                          /* 批量发送多个消息 */
uint8_t fdcan_receive_msg(uint8_t *buf);

/* 诊断和恢复函数 */
void fdcan_diagnose_status(void);
uint8_t fdcan_force_reset(uint8_t can_num);
uint8_t fdcan_check_hardware(uint8_t can_num);
void fdcan_show_statistics(void);
void fdcan_reset_statistics(void);
uint8_t fdcan_check_system_resources(void);
void fdcan_system_maintenance(void);

/* 总线恢复函数 */
uint8_t fdcan_recover_busoff(uint8_t can_num);
void fdcan_check_and_recover(void);

/* FDCAN2监控和保护函数 */
uint8_t fdcan2_monitor_and_protect(void);  /* 监控FDCAN2状态，自动禁用以保护FDCAN1 */

/* 全局控制标志（外部可访问） */
extern volatile uint32_t g_fdcan1_send_count;             /* FDCAN1发送计数 */
extern volatile uint32_t g_fdcan1_error_count;           /* FDCAN1错误计数 */
extern volatile uint32_t g_fdcan2_send_count;             /* FDCAN2发送计数 */
extern volatile uint32_t g_fdcan2_error_count;           /* FDCAN2错误计数 */

extern volatile uint8_t g_fdcan2_disabled;              /* FDCAN2禁用标志 */
extern volatile uint32_t g_fdcan2_busoff_count;         /* FDCAN2 Bus-Off次数 */
extern volatile uint32_t g_fdcan2_error_passive_count;  /* FDCAN2 Error Passive次数 */

#endif





