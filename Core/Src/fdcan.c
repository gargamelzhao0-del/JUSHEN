/**
 ****************************************************************************************************
 * @file        fdcan.c
 * @version     V2.0 - 工业级优化版本
 * @brief       FDCAN 驱动代码
 * @note        V2.0: 添加发送超时保护，改进错误处理
 ****************************************************************************************************
 * @attention   Waiken-Smart 慧勤智远
 *
 * 实验平台:    STM32H743IIT6小系统板
 *
 ****************************************************************************************************
 */
 
#include "fdcan.h"
#include "system_config.h"
#include "cmsis_os.h"
#include "log_system.h"
#include <stdio.h>
#include <stdlib.h>

/* ============================================================================
 * V2.0: 超时保护配置
 * ============================================================================ */

#define FDCAN_TX_TIMEOUT_MS           10      /* 发送超时时间 */
#define FDCAN_FIFO_WAIT_MAX_RETRIES   5       /* FIFO等待最大重试次数 */
#define FDCAN_FIFO_WAIT_DELAY_US      100     /* FIFO等待延时(微秒) */

/* ============================================================================
 * 外部变量声明
 * ============================================================================ */

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;


/* FDCAN1句柄和消息结构 */
/* extern FDCAN_HandleTypeDef       hfdcan1;                     FDCAN1句柄 (在globals.c中定义) */
FDCAN_TxHeaderTypeDef     g_fdcan1_txheade;            /* FDCAN1发送消息 */
FDCAN_RxHeaderTypeDef     g_fdcan1_rxheade;            /* FDCAN1接收消息 */

/* FDCAN2句柄和消息结构 */
/* extern FDCAN_HandleTypeDef       hfdcan2;                     FDCAN2句柄 (在globals.c中定义) */
FDCAN_TxHeaderTypeDef     g_fdcan2_txheade;            /* FDCAN2发送消息 */
FDCAN_RxHeaderTypeDef     g_fdcan2_rxheade;            /* FDCAN2接收消息 */

/* 通信统计和监控 */
volatile uint32_t g_fdcan1_send_count = 0;             /* FDCAN1发送计数 */
volatile uint32_t g_fdcan2_send_count = 0;             /* FDCAN2发送计数 */
volatile uint32_t g_fdcan1_error_count = 0;           /* FDCAN1错误计数 */
volatile uint32_t g_fdcan2_error_count = 0;           /* FDCAN2错误计数 */
volatile uint32_t g_last_successful_send = 0;         /* 最后成功发送时间 */
volatile uint32_t g_total_operation_time = 0;         /* 总运行时间 */
volatile uint32_t g_system_start_time = 0;            /* 系统启动时间 */
volatile uint32_t g_memory_usage_check = 0;           /* 内存使用检查 */

/* ✅ FDCAN2控制标志 */
volatile uint8_t g_fdcan2_disabled = 0;               /* 1=完全禁用FDCAN2发送（避免影响FDCAN1） */
volatile uint32_t g_fdcan1_busoff_count = 0;          /* FDCAN1进入Bus-Off次数 */
volatile uint32_t g_fdcan2_busoff_count = 0;          /* FDCAN2进入Bus-Off次数 */
volatile uint32_t g_fdcan2_error_passive_count = 0;   /* FDCAN2进入Error Passive次数 */

/* 总线恢复控制 */
static volatile uint8_t g_fdcan1_in_busoff = 0;        /* FDCAN1 BUS-OFF状态 */
static volatile uint8_t g_fdcan2_in_busoff = 0;        /* FDCAN2 BUS-OFF状态 */
static volatile uint32_t g_fdcan1_recover_count = 0;   /* FDCAN1恢复次数 */
static volatile uint32_t g_fdcan2_recover_count = 0;   /* FDCAN2恢复次数 */

/* Bus-Off恢复配置 */
#define FDCAN_BUSOFF_RECOVERY_DELAY_MS   100    /* Bus-Off恢复延迟时间 */
#define FDCAN_BUSOFF_MAX_RECOVERY_ATTEMPTS 5    /* 单轮最大恢复尝试次数 */
#define FDCAN_BUSOFF_RETRY_INTERVAL_MS    1000  /* 放弃后重新触发恢复的间隔 */

/* V2.0: 非阻塞Bus-Off恢复状态机 */
typedef struct {
    uint8_t in_recovery;                    /* 是否正在恢复中 */
    uint8_t attempts;                       /* 当前尝试次数 */
    uint32_t recovery_start_tick;           /* 恢复开始时间戳 */
    uint32_t last_check_tick;               /* 上次检查时间戳 */
    uint8_t software_reset_done;            /* 是否已执行软件复位 */
    uint8_t exhausted;                      /* 本轮已用完尝试次数 */
    uint32_t exhaust_tick;                  /* 本轮放弃时间戳 */
} BusOffRecoveryState_t;

static BusOffRecoveryState_t g_fdcan1_recovery_state = {0};
static BusOffRecoveryState_t g_fdcan2_recovery_state = {0};

/**
 * @brief       V2.0: 非阻塞FDCAN Bus-Off恢复状态机
 * @param       hfdcan  : FDCAN句柄指针
 * @param       fdcan_id: FDCAN编号 (1或2)
 * @retval      0 恢复成功/正在恢复中, 1 恢复失败
 * @note        非阻塞实现，每次调用只检查状态，不阻塞等待
 */
static uint8_t fdcan_busoff_recovery_nonblocking(FDCAN_HandleTypeDef *hfdcan, uint8_t fdcan_id)
{
    BusOffRecoveryState_t *state = (fdcan_id == 1) ? &g_fdcan1_recovery_state : &g_fdcan2_recovery_state;
    uint32_t current_tick = HAL_GetTick();

    /* --- 第一阶段：未在恢复中，判断是否需要启动新一轮 --- */
    if (!state->in_recovery) {
        /* 如果上一轮已耗尽尝试次数，等待冷却后重新触发 */
        if (state->exhausted) {
            if ((current_tick - state->exhaust_tick) < FDCAN_BUSOFF_RETRY_INTERVAL_MS) {
                return 1;  /* 冷却期内，仍返回失败 */
            }
            /* 冷却期结束，重新开始 */
            LOG_WARN("[FDCAN%d] Bus-Off retry cooldown elapsed, restarting recovery", fdcan_id);
            state->exhausted = 0;
        }

        /* 启动新一轮恢复 */
        state->in_recovery = 1;
        state->attempts = 0;
        state->recovery_start_tick = current_tick;
        state->last_check_tick = current_tick;
        state->software_reset_done = 0;
        LOG_WARN("[FDCAN%d] Bus-Off recovery started (non-blocking)", fdcan_id);
        return 0;
    }

    /* --- 第二阶段：恢复中，按间隔检查总线状态 --- */
    if ((current_tick - state->last_check_tick) < FDCAN_BUSOFF_RECOVERY_DELAY_MS) {
        return 0;
    }

    state->last_check_tick = current_tick;

    /* 检查Bus-Off是否已解除 */
    uint32_t psr = hfdcan->Instance->PSR;
    uint8_t bo = (psr >> 7) & 0x1;

    if (!bo) {
        LOG_INFO("[FDCAN%d] Bus-Off recovery successful (attempt %d)", fdcan_id, state->attempts + 1);
        state->in_recovery = 0;
        state->exhausted = 0;

        if (fdcan_id == 1) {
            g_fdcan1_in_busoff = 0;
            g_fdcan1_recover_count++;
        } else {
            g_fdcan2_in_busoff = 0;
            g_fdcan2_recover_count++;
        }
        return 0;
    }

    state->attempts++;

    /* 单轮尝试次数用完 → 进入冷却期，后续发送会重新触发 */
    if (state->attempts >= FDCAN_BUSOFF_MAX_RECOVERY_ATTEMPTS) {
        LOG_WARN("[FDCAN%d] Bus-Off recovery round failed (%d attempts), will retry in %d ms",
                 fdcan_id, state->attempts, FDCAN_BUSOFF_RETRY_INTERVAL_MS);
        state->in_recovery = 0;
        state->exhausted = 1;
        state->exhaust_tick = current_tick;
        return 1;
    }

    /* 第2次尝试时执行软件复位 */
    if (state->attempts == 2 && !state->software_reset_done) {
        LOG_WARN("[FDCAN%d] Attempting software reset...", fdcan_id);

        SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        uint32_t timeout = 100;
        while (!(READ_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT)) && timeout-- > 0) {
        }

        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
        timeout = 100;
        while (READ_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT) && timeout-- > 0) {
        }

        state->software_reset_done = 1;
    }

    return 0;
}

/**
 * @brief       V2.0: 检查并处理FDCAN Bus-Off状态（非阻塞）
 * @param       hfdcan  : FDCAN句柄指针
 * @param       fdcan_id: FDCAN编号 (1或2)
 * @retval      0 正常, 1 处于Bus-Off状态
 */
static uint8_t fdcan_check_and_handle_busoff(FDCAN_HandleTypeDef *hfdcan, uint8_t fdcan_id)
{
    uint32_t psr = hfdcan->Instance->PSR;
    uint8_t bo = (psr >> 7) & 0x1;
    
    if (bo) {
        if (fdcan_id == 1 && !g_fdcan1_in_busoff) {
            g_fdcan1_in_busoff = 1;
            g_fdcan1_busoff_count++;
            LOG_ERROR("[FDCAN1] BUS-OFF detected! Count: %u", g_fdcan1_busoff_count);
        } else if (fdcan_id == 2 && !g_fdcan2_in_busoff) {
            g_fdcan2_in_busoff = 1;
            g_fdcan2_busoff_count++;
            LOG_ERROR("[FDCAN2] BUS-OFF detected! Count: %u", g_fdcan2_busoff_count);
        }
        
        fdcan_busoff_recovery_nonblocking(hfdcan, fdcan_id);
        return 1;
    }
    
    return 0;
}


/**
 * @brief       FDCAN初始化
 * @param       presc   : 分频值，取值范围1~512;
 * @param       tsjw    : 重新同步跳跃时间单元.范围:1~128;
 * @param       ntsg1   : 时间段1的时间单元.取值范围2~256;
 * @param       ntsg2   : 时间段2的时间单元.取值范围2~128;
 * @param       mode    : FDCAN_MODE_NORMAL，普通模式;
                          FDCAN_MODE_INTERNAL_LOOPBACK，内部回环模式;
                          FDCAN_MODE_EXTERNAL_LOOPBACK，外部回环模式;
                          FDCAN_MODE_RESTRICTED_OPERATION，限制操作模式
                          FDCAN_MODE_BUS_MONITORING，总线监控模式
 * @note        以上5个参数, 除了操作模式选择，其余的参数在函数内部会减1, 所以, 任何一个参数都不能等于0
 *              配置FDCAN的时钟源为PLL1Q, FDCAN输入时钟频率 fdcan_ker_ck = pll1_q_ck = 80Mhz
 *              波特率 = fdcan_ker_ck / ((ntseg1 + ntseg2 + 1) * presc);
 *              调用示例: fdcan1_init(4, 4, 15, 4, mode)
 *              仲裁段: 80M / ((15 + 4 + 1) * 4) = 1Mbps, 采样点80%, SJW=4Tq
 *              数据段: 80M / ((15 + 4 + 1) * 1) = 4Mbps, 采样点80%, SJW=4Tq
 * @retval      0, 初始化成功; 其他, 初始化失败;
 */

/* FDCAN1初始化函数 */
uint8_t fdcan1_init(uint16_t presc, uint8_t tsjw, uint16_t ntseg1, uint8_t ntseg2, uint32_t mode)
{
    FDCAN_FilterTypeDef fdcan_filterconfig;

    /* ✅ 安全的DeInit：不会影响FDCAN2（已修改MspDeInit） */
    HAL_FDCAN_DeInit(&hfdcan1);
    hfdcan1.Instance = FDCAN1; 
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = mode;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = ENABLE;
    

    hfdcan1.Init.NominalPrescaler = presc;
    hfdcan1.Init.NominalSyncJumpWidth = tsjw;
    hfdcan1.Init.NominalTimeSeg1 = ntseg1;
    hfdcan1.Init.NominalTimeSeg2 = ntseg2;


    hfdcan1.Init.DataPrescaler = 1;      /* ✅ DataPrescaler=1: 80MHz / ((15+4+1) * 1) = 4MHz */
    hfdcan1.Init.DataSyncJumpWidth = 4;
    hfdcan1.Init.DataTimeSeg1 = 15;      /* ✅ 数据段采样点 = (1+15)/20 = 80% */
    hfdcan1.Init.DataTimeSeg2 = 4;
    
    /* ✅ 修复: 使用头文件中定义的Message RAM配置 */
    hfdcan1.Init.MessageRAMOffset = 0;
    hfdcan1.Init.StdFiltersNbr = FDCAN1_STD_FILTERS_NBR;
    hfdcan1.Init.ExtFiltersNbr = FDCAN1_EXT_FILTERS_NBR;
    hfdcan1.Init.RxFifo0ElmtsNbr = FDCAN1_RX_FIFO0_ELMTS_NBR;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan1.Init.RxFifo1ElmtsNbr = 0;
    hfdcan1.Init.RxBuffersNbr = 0;
    hfdcan1.Init.TxEventsNbr = 0;
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = FDCAN1_TX_FIFO_ELMTS_NBR;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        printf("[ERROR] FDCAN1 HAL_FDCAN_Init failed!\r\n");
        return 1;
    }

    /* 配置标准ID过滤器接收所有标准帧 */
    fdcan_filterconfig.IdType = FDCAN_STANDARD_ID;
    fdcan_filterconfig.FilterIndex = 0;
    fdcan_filterconfig.FilterType = FDCAN_FILTER_RANGE;
    fdcan_filterconfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filterconfig.FilterID1 = 0x000;
    fdcan_filterconfig.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filterconfig) != HAL_OK)
    {
        printf("[ERROR] FDCAN1 ConfigFilter (Standard) failed!\n");
        return 2;
    }

    /* ✅ 配置扩展ID过滤器接收所有扩展帧（电机使用扩展ID）*/
    fdcan_filterconfig.IdType = FDCAN_EXTENDED_ID;
    fdcan_filterconfig.FilterIndex = 0;
    fdcan_filterconfig.FilterType = FDCAN_FILTER_RANGE;
    fdcan_filterconfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filterconfig.FilterID1 = 0x00000000;
    fdcan_filterconfig.FilterID2 = 0x1FFFFFFF;  /* 接收所有29位扩展ID */

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filterconfig) != HAL_OK)
    {
        printf("[ERROR] FDCAN1 ConfigFilter (Extended) failed!\n");
        return 2;
    }

    /* ✅ 全局过滤器：接受标准帧和扩展帧 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  /* 标准帧接受 */
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  /* ✅ 扩展帧也接受 */
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    /* 开启FDCAN1 */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        printf("[ERROR] FDCAN1 HAL_FDCAN_Start failed!\r\n");
        return 3;
    }

    /* 使能接收FIFO 0新消息中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* 验证FDCAN1状态 */
    uint32_t cccr_val = hfdcan1.Instance->CCCR;
    if (cccr_val & FDCAN_CCCR_INIT) {
        printf("[ERROR] FDCAN1 stuck in INIT mode!\n");
        return 4;
    }

    printf("[FDCAN1] Init OK - 80MHz, 1Mbps/4Mbps, PB8/PB9\n");
    printf("[FDCAN1] Message RAM: %d WORDS used (offset=0)\n", FDCAN1_RAM_TOTAL);

    return 0;
}

/* FDCAN2初始化函数 */
uint8_t fdcan2_init(uint16_t presc, uint8_t tsjw, uint16_t ntseg1, uint8_t ntseg2, uint32_t mode)
{
    FDCAN_FilterTypeDef fdcan_filterconfig;

    /* ✅ 安全的DeInit：不会影响FDCAN1（已修改MspDeInit） */
    HAL_FDCAN_DeInit(&hfdcan2);
    hfdcan2.Instance = FDCAN2; 
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan2.Init.Mode = mode;
    hfdcan2.Init.AutoRetransmission = ENABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = ENABLE;
    
    /* 改进配置：增加错误处理能力 */
    hfdcan2.Init.NominalPrescaler = presc;
    hfdcan2.Init.NominalSyncJumpWidth = tsjw;
    hfdcan2.Init.NominalTimeSeg1 = ntseg1;
    hfdcan2.Init.NominalTimeSeg2 = ntseg2;

    hfdcan2.Init.DataPrescaler = 1;      /* ✅ DataPrescaler=1: 80MHz / ((15+4+1) * 1) = 4MHz */
    hfdcan2.Init.DataSyncJumpWidth = 4;
    hfdcan2.Init.DataTimeSeg1 = 15;      /* ✅ 数据段采样点 = (1+15)/20 = 80% */
    hfdcan2.Init.DataTimeSeg2 = 4;
    
    /* ✅ 修复: 使用头文件中定义的Message RAM配置，确保无冲突 */
    hfdcan2.Init.MessageRAMOffset = FDCAN2_MESSAGE_RAM_OFFSET;
    hfdcan2.Init.StdFiltersNbr = FDCAN2_STD_FILTERS_NBR;
    hfdcan2.Init.ExtFiltersNbr = FDCAN2_EXT_FILTERS_NBR;
    hfdcan2.Init.RxFifo0ElmtsNbr = FDCAN2_RX_FIFO0_ELMTS_NBR;
    hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    hfdcan2.Init.RxFifo1ElmtsNbr = 0;
    hfdcan2.Init.RxBuffersNbr = 0;
    hfdcan2.Init.TxEventsNbr = 0;
    hfdcan2.Init.TxBuffersNbr = 0;
    hfdcan2.Init.TxFifoQueueElmtsNbr = FDCAN2_TX_FIFO_ELMTS_NBR;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
    {
        printf("[ERROR] FDCAN2 HAL_FDCAN_Init failed!\n");
        return 1;   
    }

    /* 配置标准ID过滤器接收所有标准帧 */
    fdcan_filterconfig.IdType = FDCAN_STANDARD_ID;
    fdcan_filterconfig.FilterIndex = 0;
    fdcan_filterconfig.FilterType = FDCAN_FILTER_RANGE;
    fdcan_filterconfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filterconfig.FilterID1 = 0x000;
    fdcan_filterconfig.FilterID2 = 0x7FF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filterconfig) != HAL_OK)
    {
        printf("[ERROR] FDCAN2 ConfigFilter (Standard) failed!\n");
        return 2;
    }

    /* ✅ 配置扩展ID过滤器接收所有扩展帧（电机使用扩展ID）*/
    fdcan_filterconfig.IdType = FDCAN_EXTENDED_ID;
    fdcan_filterconfig.FilterIndex = 0;
    fdcan_filterconfig.FilterType = FDCAN_FILTER_RANGE;
    fdcan_filterconfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    fdcan_filterconfig.FilterID1 = 0x00000000;
    fdcan_filterconfig.FilterID2 = 0x1FFFFFFF;  /* 接收所有29位扩展ID */

    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filterconfig) != HAL_OK)
    {
        printf("[ERROR] FDCAN2 ConfigFilter (Extended) failed!\n");
        return 2;
    }

    /* ✅ 全局过滤器：接受标准帧和扩展帧 */
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  /* 标准帧接受 */
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  /* ✅ 扩展帧也接受 */
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);

    /* 开启FDCAN2 */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        printf("[ERROR] FDCAN2 HAL_FDCAN_Start failed!\n");
        return 3;
    }

    /* 使能接收FIFO 0新消息中断 */
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    /* 验证FDCAN2状态 */
    uint32_t cccr_val = hfdcan2.Instance->CCCR;
    if (cccr_val & FDCAN_CCCR_INIT) {
        printf("[ERROR] FDCAN2 stuck in INIT mode!\n");
        return 4;
    }

    printf("[FDCAN2] Init OK - 80MHz, 1Mbps/4Mbps, PB5/PB6\n");
    printf("[FDCAN2] Message RAM: %d WORDS used (offset=%d)\n", FDCAN2_RAM_TOTAL, FDCAN2_MESSAGE_RAM_OFFSET);
    printf("[FDCAN] Total RAM: %d WORDS / %d available\n", 
           FDCAN1_RAM_TOTAL + FDCAN2_RAM_TOTAL, FDCAN_MESSAGE_RAM_SIZE_WORDS);

    return 0;
}

/* 兼容性函数 (默认使用FDCAN1) */
uint8_t fdcan_init(uint16_t presc, uint8_t tsjw, uint16_t ntseg1, uint8_t ntseg2, uint32_t mode)
{
    return fdcan1_init(presc, tsjw, ntseg1, ntseg2, mode);
}

/**
 * @brief       FDCAN底层驱动，引脚配置，时钟配置，中断配置
 * @note        此函数会被HAL_FDCAN_Init()调用
 * @param       hcan:FDCAN句柄
 * @retval      无;
 */
void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hcan)
{
    GPIO_InitTypeDef gpio_init_struct;
    RCC_PeriphCLKInitTypeDef fdcan_periphclk;
  
    if (FDCAN1 == hcan->Instance)
    {
        /* FDCAN1时钟源配置为PLL1Q */
        fdcan_periphclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        fdcan_periphclk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        HAL_RCCEx_PeriphCLKConfig(&fdcan_periphclk);

        __HAL_RCC_FDCAN_CLK_ENABLE();                         /* 使能FDCAN时钟 */
        FDCAN1_RX_GPIO_CLK_ENABLE();                          /* FDCAN1_RX引脚时钟使能 */
        FDCAN1_TX_GPIO_CLK_ENABLE();                          /* FDCAN1_TX引脚时钟使能 */
        
        gpio_init_struct.Pin = FDCAN1_RX_GPIO_PIN;            /* FDCAN1_RX引脚 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;              /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                  /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;   /* ✅ V2.1: 极高速(4Mbps数据段需要) */
        gpio_init_struct.Alternate = FDCAN1_RX_GPIO_AF;       /* 复用为FDCAN1 */
        HAL_GPIO_Init(FDCAN1_RX_GPIO_PORT, &gpio_init_struct); /* 初始化FDCAN1_RX引脚 */

        gpio_init_struct.Pin = FDCAN1_TX_GPIO_PIN;            /* FDCAN1_TX引脚 */
        gpio_init_struct.Pull = GPIO_NOPULL;                  /* ✅ V2.1: TX不需要上拉，避免信号畸变 */
        gpio_init_struct.Alternate = FDCAN1_TX_GPIO_AF;       /* 复用为FDCAN1 */
        HAL_GPIO_Init(FDCAN1_TX_GPIO_PORT, &gpio_init_struct); /* 初始化FDCAN1_TX引脚 */

#if FDCAN1_RX0_INT_ENABLE                                     /* 如果使能了FDCAN1接收RX0中断 */
        /* 注意: 回调中使用了osSemaphoreRelease()，必须设置优先级 >= 6 */
        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 6, 0);          /* 抢占优先级6 (FreeRTOS安全) */
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);                  /* 使能FDCAN1中断线0中断 */
#endif
    }
    else if (FDCAN2 == hcan->Instance)
    {
        /* FDCAN2时钟源配置为PLL1Q */
        fdcan_periphclk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
        fdcan_periphclk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
        HAL_RCCEx_PeriphCLKConfig(&fdcan_periphclk);

        __HAL_RCC_FDCAN_CLK_ENABLE();                         /* 使能FDCAN时钟 */
        FDCAN2_RX_GPIO_CLK_ENABLE();                          /* FDCAN2_RX引脚时钟使能 */
        FDCAN2_TX_GPIO_CLK_ENABLE();                          /* FDCAN2_TX引脚时钟使能 */
        
        gpio_init_struct.Pin = FDCAN2_RX_GPIO_PIN;            /* FDCAN2_RX引脚 (PB5) */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;              /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                  /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;   /* ✅ V2.1: 极高速(4Mbps数据段需要) */
        gpio_init_struct.Alternate = FDCAN2_RX_GPIO_AF;       /* 复用为FDCAN2 */
        HAL_GPIO_Init(FDCAN2_RX_GPIO_PORT, &gpio_init_struct); /* 初始化FDCAN2_RX引脚 */

        gpio_init_struct.Pin = FDCAN2_TX_GPIO_PIN;            /* FDCAN2_TX引脚 (PB6) */
        gpio_init_struct.Pull = GPIO_NOPULL;                  /* ✅ V2.1: TX不需要上拉，避免信号畸变 */
        gpio_init_struct.Alternate = FDCAN2_TX_GPIO_AF;       /* 复用为FDCAN2 */
        HAL_GPIO_Init(FDCAN2_TX_GPIO_PORT, &gpio_init_struct); /* 初始化FDCAN2_TX引脚 */

#if FDCAN2_RX0_INT_ENABLE                                     /* 如果使能了FDCAN2接收RX0中断 */
        /* 注意: 回调中使用了osSemaphoreRelease()，必须设置优先级 >= 6 */
        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 6, 0);          /* 抢占优先级6 (FreeRTOS安全) */
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);                  /* 使能FDCAN2中断线0中断 */
#endif
    }
}

/**
 * @brief       此函数会被HAL_FDCAN_DeInit调用
 * @param       hfdcan:fdcan句柄
 * @retval      无
 * @note        ⚠️ 注意：FDCAN1和FDCAN2共享同一个外设时钟和复位控制
 *              为避免一个FDCAN的DeInit影响另一个，这里不执行全局复位
 */
void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *hfdcan)
{
    /* ⚠️ 禁用全局复位，因为FDCAN1和FDCAN2共享外设 */
    /* __HAL_RCC_FDCAN_FORCE_RESET(); */   /* 这会复位FDCAN1和FDCAN2！ */
    /* __HAL_RCC_FDCAN_RELEASE_RESET(); */ /* 停止复位 */

    /* 只禁用对应FDCAN的中断 */
#if FDCAN1_RX0_INT_ENABLE
    if (hfdcan->Instance == FDCAN1) {
        HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
    }
#endif

#if FDCAN2_RX0_INT_ENABLE
    if (hfdcan->Instance == FDCAN2) {
        HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
    }
#endif
}

/**
 * @brief       FDCAN 发送一组数据
 * @note        发送格式固定为: 标准ID, 数据帧
 * @param       len     :数据长度，取值范围：FDCAN_DLC_BYTES_0 ~ FDCAN_DLC_BYTES_64
 * @param       msg     :数据指针，最大为64个字节
 * @retval      发送状态 0, 成功; 1, 失败;
 */
/* FDCAN1发送函数（优化版） */
uint8_t fdcan1_send_msg(uint8_t *msg, uint32_t len)
{
    /* 快速状态检查 */
    uint32_t psr = hfdcan1.Instance->PSR;
    uint8_t bo = (psr >> 7) & 0x1;

    if (bo) {
        g_fdcan1_error_count++;
        return 1;
    }

    /* 检查发送FIFO是否有空间 */
    uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    if (free_level == 0) {
        return 1;
    }

    /* 配置发送消息头 */
    g_fdcan1_txheade.Identifier = 0x200;
    g_fdcan1_txheade.IdType = FDCAN_STANDARD_ID;
    g_fdcan1_txheade.TxFrameType = FDCAN_DATA_FRAME;
    g_fdcan1_txheade.DataLength = len;
    g_fdcan1_txheade.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    g_fdcan1_txheade.BitRateSwitch = FDCAN_BRS_ON;
    g_fdcan1_txheade.FDFormat = FDCAN_FD_CAN;
    g_fdcan1_txheade.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_fdcan1_txheade.MessageMarker = 0;

    /* 发送消息 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &g_fdcan1_txheade, msg) != HAL_OK)
    {
        g_fdcan1_error_count++;
        return 1;
    }

    g_fdcan1_send_count++;
    g_last_successful_send = HAL_GetTick();
    return 0;
}

/**
 * @brief       FDCAN1批量发送多个消息
 * @param       msgs: 消息数组
 * @param       lens: 消息长度数组
 * @param       count: 消息数量
 * @retval      发送状态 0, 全部成功; 1, 部分或全部失败;
 */
uint8_t fdcan1_send_batch(uint8_t *msgs[], uint32_t lens[], uint8_t count)
{
    if (count == 0) return 0;

    /* 检查总线状态 */
    uint32_t psr = hfdcan1.Instance->PSR;
    uint8_t bo = (psr >> 7) & 0x1;

    if (bo) {
        g_fdcan1_error_count += count;
        return 1;
    }

    /* 检查FIFO空间 */
    uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
    if (free_level < count) {
        return 1;
    }

    uint8_t success_count = 0;

    /* 批量发送所有消息 */
    for (uint8_t i = 0; i < count; i++) {
        g_fdcan1_txheade.Identifier = 0x200 + i;
        g_fdcan1_txheade.IdType = FDCAN_STANDARD_ID;
        g_fdcan1_txheade.TxFrameType = FDCAN_DATA_FRAME;
        g_fdcan1_txheade.DataLength = lens[i];
        g_fdcan1_txheade.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        g_fdcan1_txheade.BitRateSwitch = FDCAN_BRS_ON;
        g_fdcan1_txheade.FDFormat = FDCAN_FD_CAN;
        g_fdcan1_txheade.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        g_fdcan1_txheade.MessageMarker = i;

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &g_fdcan1_txheade, msgs[i]) == HAL_OK) {
            success_count++;
        } else {
            g_fdcan1_error_count++;
        }
    }

    if (success_count > 0) {
        g_fdcan1_send_count += success_count;
        g_last_successful_send = HAL_GetTick();
    }

    return (success_count == count) ? 0 : 1;
}

/**
 * @brief       FDCAN1发送一组数据（灵活配置版本）
 * @note        支持自定义ID、帧格式、数据类型等参数
 *              V2.0: 添加发送超时保护
 * @param       msg     : 数据指针，最大为64个字节
 * @param       config  : 发送配置参数结构体指针
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t fdcan1_send_msg_ex(uint8_t *msg, FDCAN_TxConfigTypeDef *config)
{
    if (config == NULL || msg == NULL) {
        LOG_ERROR("[FDCAN1] Invalid parameters");
        return 1;
    }

    /* 检查并处理Bus-Off状态 */
    if (fdcan_check_and_handle_busoff(&hfdcan1, 1)) {
        g_fdcan1_error_count++;
        return 1;
    }

    /* V2.0: 带超时的FIFO空间等待 */
    uint32_t start_tick = HAL_GetTick();
    uint32_t free_level;
    uint8_t retries = 0;
    
    while (retries < FDCAN_FIFO_WAIT_MAX_RETRIES) {
        free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        if (free_level > 0) {
            break;
        }
        
        /* 检查超时 */
        if ((HAL_GetTick() - start_tick) >= FDCAN_TX_TIMEOUT_MS) {
            g_fdcan1_error_count++;
            LOG_WARN("[FDCAN1] TX FIFO timeout");
            return 1;
        }
        
        retries++;
        /* 短暂延时后重试 */
        for (volatile uint32_t i = 0; i < (SystemCoreClock / 1000000 * FDCAN_FIFO_WAIT_DELAY_US); i++) {
            __NOP();
        }
    }
    
    if (free_level == 0) {
        g_fdcan1_error_count++;
        LOG_WARN("[FDCAN1] TX FIFO full after retries");
        return 1;
    }

    /* 获取错误状态用于ESI设置 */
    uint32_t psr = hfdcan1.Instance->PSR;
    uint8_t ep = (psr >> 5) & 0x1;
    
    g_fdcan1_txheade.Identifier = config->identifier;
    g_fdcan1_txheade.IdType = config->id_type;
    g_fdcan1_txheade.TxFrameType = config->frame_type;
    g_fdcan1_txheade.DataLength = config->data_length;
    g_fdcan1_txheade.ErrorStateIndicator = ep ? FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
    g_fdcan1_txheade.BitRateSwitch = config->bit_rate_switch;
    g_fdcan1_txheade.FDFormat = config->fd_format;
    g_fdcan1_txheade.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_fdcan1_txheade.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &g_fdcan1_txheade, msg) != HAL_OK)
    {
        g_fdcan1_error_count++;
        LOG_ERROR("[FDCAN1] HAL_FDCAN_AddMessageToTxFifoQ failed");
        return 1;
    }

    g_fdcan1_send_count++;
    g_last_successful_send = HAL_GetTick();
    return 0;
}

/* FDCAN2发送函数（优化版） */
uint8_t fdcan2_send_msg(uint8_t *msg, uint32_t len)
{
    /* ✅ V2.1: 移除额外限制，与CAN1保持一致 */

    /* 快速状态检查 */
    uint32_t psr = hfdcan2.Instance->PSR;
    uint8_t bo = (psr >> 7) & 0x1;

    if (bo) {
        g_fdcan2_error_count++;
        return 1;
    }

    /* 检查发送FIFO是否有空间 */
    uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    if (free_level == 0) {
        return 1;
    }

    /* 配置发送消息头 */
    g_fdcan2_txheade.Identifier = 0x124;
    g_fdcan2_txheade.IdType = FDCAN_STANDARD_ID;
    g_fdcan2_txheade.TxFrameType = FDCAN_DATA_FRAME;
    g_fdcan2_txheade.DataLength = len;
    g_fdcan2_txheade.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    g_fdcan2_txheade.BitRateSwitch = FDCAN_BRS_ON;
    g_fdcan2_txheade.FDFormat = FDCAN_FD_CAN;
    g_fdcan2_txheade.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_fdcan2_txheade.MessageMarker = 0;

    /* 发送消息 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &g_fdcan2_txheade, msg) != HAL_OK)
    {
        g_fdcan2_error_count++;
        return 1;
    }

    g_fdcan2_send_count++;
    g_last_successful_send = HAL_GetTick();
    return 0;
}

/**
 * @brief       FDCAN2批量发送多个消息
 * @param       msgs: 消息数组
 * @param       lens: 消息长度数组
 * @param       count: 消息数量
 * @retval      发送状态 0, 全部成功; 1, 部分或全部失败;
 */
uint8_t fdcan2_send_batch(uint8_t *msgs[], uint32_t lens[], uint8_t count)
{
    if (count == 0) return 0;

    /* 检查总线状态 */
    uint32_t psr = hfdcan2.Instance->PSR;
    uint8_t bo = (psr >> 7) & 0x1;

    if (bo) {
        g_fdcan2_error_count += count;
        return 1;
    }

    /* 检查INIT模式 */
    uint32_t cccr = hfdcan2.Instance->CCCR;
    if (cccr & FDCAN_CCCR_INIT) {
        g_fdcan2_error_count += count;
        return 1;
    }

    /* 检查FIFO空间 */
    uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
    if (free_level < count) {
        return 1;
    }

    uint8_t success_count = 0;

    /* 批量发送所有消息 */
    for (uint8_t i = 0; i < count; i++) {
        g_fdcan2_txheade.Identifier = 0x124 + i;
        g_fdcan2_txheade.IdType = FDCAN_STANDARD_ID;
        g_fdcan2_txheade.TxFrameType = FDCAN_DATA_FRAME;
        g_fdcan2_txheade.DataLength = lens[i];
        g_fdcan2_txheade.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        g_fdcan2_txheade.BitRateSwitch = FDCAN_BRS_ON;
        g_fdcan2_txheade.FDFormat = FDCAN_FD_CAN;
        g_fdcan2_txheade.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        g_fdcan2_txheade.MessageMarker = i;

        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &g_fdcan2_txheade, msgs[i]) == HAL_OK) {
            success_count++;
        } else {
            g_fdcan2_error_count++;
        }
    }

    if (success_count > 0) {
        g_fdcan2_send_count += success_count;
        g_last_successful_send = HAL_GetTick();
    }

    return (success_count == count) ? 0 : 1;
}

/**
 * @brief       FDCAN2发送一组数据（灵活配置版本）
 * @note        支持自定义ID、帧格式、数据类型等参数
 * @param       msg     : 数据指针，最大为64个字节
 * @param       config  : 发送配置参数结构体指针
 * @retval      发送状态 0, 成功; 1, 失败;
 */
uint8_t fdcan2_send_msg_ex(uint8_t *msg, FDCAN_TxConfigTypeDef *config)
{
    /* 参数检查 */
    if (config == NULL || msg == NULL) {
        printf("[ERROR] FDCAN2 Invalid parameters!\r\n");
        return 1;
    }

    /* ✅ V2.1: 移除额外的禁用限制，与CAN1保持一致
     * 这些限制会导致CAN2发送频率低于CAN1
     * 保留Bus-Off和错误计数器用于诊断，但不作为发送阻断条件
     */

    /* 检查并处理Bus-Off状态 */
    if (fdcan_check_and_handle_busoff(&hfdcan2, 2)) {
        g_fdcan2_error_count++;
        return 1;
    }

    uint32_t cccr = hfdcan2.Instance->CCCR;
    if (cccr & FDCAN_CCCR_INIT) {
        printf("[ERROR] FDCAN2 in INIT mode!\r\n");
        return 1;
    }

    /* 获取错误状态用于ESI设置 */
    uint32_t psr = hfdcan2.Instance->PSR;
    uint8_t ep = (psr >> 5) & 0x1;

    /* ✅ V2.1: 与CAN1一致的重试机制，移除阻塞的HAL_Delay */
    uint32_t start_tick = HAL_GetTick();
    uint32_t free_level;
    uint8_t retries = 0;

    while (retries < FDCAN_FIFO_WAIT_MAX_RETRIES) {
        free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2);
        if (free_level > 0) {
            break;
        }

        /* 检查超时 */
        if ((HAL_GetTick() - start_tick) >= FDCAN_TX_TIMEOUT_MS) {
            g_fdcan2_error_count++;
            LOG_WARN("[FDCAN2] TX FIFO timeout");
            return 1;
        }

        retries++;
        /* 短暂延时后重试（忙等待，不阻塞） */
        for (volatile uint32_t i = 0; i < (SystemCoreClock / 1000000 * FDCAN_FIFO_WAIT_DELAY_US); i++) {
            __NOP();
        }
    }

    if (free_level == 0) {
        g_fdcan2_error_count++;
        LOG_WARN("[FDCAN2] TX FIFO full after retries");
        return 1;
    }

    /* 配置发送消息头（使用用户提供的参数） */
    g_fdcan2_txheade.Identifier = config->identifier;
    g_fdcan2_txheade.IdType = config->id_type;
    g_fdcan2_txheade.TxFrameType = config->frame_type;
    g_fdcan2_txheade.DataLength = config->data_length;
    g_fdcan2_txheade.ErrorStateIndicator = ep ? FDCAN_ESI_PASSIVE : FDCAN_ESI_ACTIVE;
    g_fdcan2_txheade.BitRateSwitch = config->bit_rate_switch;
    g_fdcan2_txheade.FDFormat = config->fd_format;
    g_fdcan2_txheade.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    g_fdcan2_txheade.MessageMarker = 0;

    /* 发送消息 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &g_fdcan2_txheade, msg) != HAL_OK)
    {
        g_fdcan2_error_count++;
        printf("[ERROR] FDCAN2 send failed!\r\n");
        return 1;
    }

    g_fdcan2_send_count++;
    g_last_successful_send = HAL_GetTick();
    return 0;
}

/* 兼容性函数 (默认使用FDCAN1) */
uint8_t fdcan_send_msg(uint8_t *msg, uint32_t len)
{
    return fdcan1_send_msg(msg, len);
}

/**
 * @brief       批量发送多个消息（兼容性函数，默认使用FDCAN1）
 */
uint8_t fdcan_send_batch(uint8_t *msgs[], uint32_t lens[], uint8_t count)
{
    return fdcan1_send_batch(msgs, lens, count);
}

/**
 * @brief       FDCAN接收数据查询
 * @param       buf:数据缓存区
 * @retval      0, 没有接收到数据;
 *              其他, 接收到的数据长度;
 */
/* FDCAN1接收函数 */
uint8_t fdcan1_receive_msg(uint8_t *buf)
{
    /* 从RX FIFO0检索接收的消息 */
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &g_fdcan1_rxheade, buf) != HAL_OK)  /* 接收数据 */
    {
        return 0;
    }

    return g_fdcan1_rxheade.DataLength >> 16;
}

/* FDCAN2接收函数 */
uint8_t fdcan2_receive_msg(uint8_t *buf)
{
    /* 从RX FIFO0检索接收的消息 */
    if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &g_fdcan2_rxheade, buf) != HAL_OK)  /* 接收数据 */
    {
        return 0;
    }

    return g_fdcan2_rxheade.DataLength >> 16;
}

/* 兼容性函数 (默认使用FDCAN1) */
uint8_t fdcan_receive_msg(uint8_t *buf)
{
    return fdcan1_receive_msg(buf);
}

/**
 * @brief       诊断FDCAN全局状态
 * @param       无
 * @retval      无
 */
void fdcan_diagnose_status(void)
{
    printf("[FINAL STATUS]\n");
    
    /* FDCAN1状态诊断 */
    uint32_t cccr1 = hfdcan1.Instance->CCCR;
    uint32_t psr1 = hfdcan1.Instance->PSR;
    uint32_t ecr1 = hfdcan1.Instance->ECR;
    uint32_t txfqs1 = hfdcan1.Instance->TXFQS;
    
    uint8_t tec1 = (ecr1 >> 0) & 0xFF;
    uint8_t rec1 = (ecr1 >> 8) & 0xFF;
    uint8_t lec1 = (psr1 >> 0) & 0x7;
    uint8_t act1 = (psr1 >> 3) & 0x3;
    uint8_t bo1 = (psr1 >> 7) & 0x1;
    uint32_t free_level1 = (txfqs1 >> 0) & 0x3F;
    
    printf("FDCAN1 CCCR: 0x%08X, PSR: 0x%08X\n", cccr1, psr1);
    
    /* FDCAN2状态诊断 */
    uint32_t cccr2 = hfdcan2.Instance->CCCR;
    uint32_t psr2 = hfdcan2.Instance->PSR;
    uint32_t ecr2 = hfdcan2.Instance->ECR;
    uint32_t txfqs2 = hfdcan2.Instance->TXFQS;
    
    uint8_t tec2 = (ecr2 >> 0) & 0xFF;
    uint8_t rec2 = (ecr2 >> 8) & 0xFF;
    uint8_t lec2 = (psr2 >> 0) & 0x7;
    uint8_t act2 = (psr2 >> 3) & 0x3;
    uint8_t bo2 = (psr2 >> 7) & 0x1;
    uint32_t free_level2 = (txfqs2 >> 0) & 0x3F;
    
    printf("FDCAN2 CCCR: 0x%08X, PSR: 0x%08X\n", cccr2, psr2);
    
    /* 详细诊断信息 */
    printf("[DIAG] FDCAN1 Status: CCCR=0x%08X, PSR=0x%08X, ECR=0x%08X\n", cccr1, psr1, ecr1);
    printf("[DIAG] TEC=%u, REC=%u, LEC=%u, ACT=%u, BO=%u\n", tec1, rec1, lec1, act1, bo1);
    printf("[DIAG] TXFQS=0x%08X (Free=%u, Get=%u, Put=%u)\n", 
           txfqs1, free_level1, (txfqs1 >> 8) & 0x1F, (txfqs1 >> 16) & 0x1F);
    
    printf("[DIAG] FDCAN2 Status: CCCR=0x%08X, PSR=0x%08X, ECR=0x%08X\n", cccr2, psr2, ecr2);
    printf("[DIAG] TEC=%u, REC=%u, LEC=%u, ACT=%u, BO=%u\n", tec2, rec2, lec2, act2, bo2);
    printf("[DIAG] TXFQS=0x%08X (Free=%u, Get=%u, Put=%u)\n", 
           txfqs2, free_level2, (txfqs2 >> 8) & 0x1F, (txfqs2 >> 16) & 0x1F);
    
    /* 状态建议 */
    if (cccr1 & FDCAN_CCCR_INIT) {
        printf("[WARNING] FDCAN1 is in INIT mode - may need reset\n");
    }
    if (cccr2 & FDCAN_CCCR_INIT) {
        printf("[WARNING] FDCAN2 is in INIT mode - may need reset\n");
    }
    if (act1 == 1) {
        printf("[WARNING] FDCAN1 is in ERROR PASSIVE state\n");
    }
    if (act2 == 1) {
        printf("[WARNING] FDCAN2 is in ERROR PASSIVE state\n");
    }
    if (bo1) {
        printf("[ERROR] FDCAN1 is in BUS-OFF state!\n");
    }
    if (bo2) {
        printf("[ERROR] FDCAN2 is in BUS-OFF state!\n");
    }
    if (free_level1 == 0) {
        printf("[WARNING] FDCAN1 TX FIFO is full\n");
    }
    if (free_level2 == 0) {
        printf("[WARNING] FDCAN2 TX FIFO is full\n");
    }
}

/**
 * @brief       检查FDCAN硬件连接状态
 * @param       can_num: FDCAN编号（1或2）
 * @retval      0: 连接正常, 1: 连接异常
 */
uint8_t fdcan_check_hardware(uint8_t can_num)
{
    FDCAN_HandleTypeDef *handle;
    
    if (can_num == 1) {
        handle = &hfdcan1;
        printf("[HARDWARE] Checking FDCAN1 hardware connection...\n");
    } else if (can_num == 2) {
        handle = &hfdcan2;
        printf("[HARDWARE] Checking FDCAN2 hardware connection...\n");
    } else {
        printf("[ERROR] Invalid FDCAN number: %u\n", can_num);
        return 1;
    }
    
    /* 读取FDCAN状态寄存器 */
    uint32_t cccr = handle->Instance->CCCR;
    uint32_t psr = handle->Instance->PSR;
    uint32_t ecr = handle->Instance->ECR;
    
    uint8_t tec = (ecr >> 0) & 0xFF;
    uint8_t rec = (ecr >> 8) & 0xFF;
    uint8_t lec = (psr >> 0) & 0x7;
    uint8_t act = (psr >> 3) & 0x3;
    uint8_t bo = (psr >> 7) & 0x1;
    
    printf("[HARDWARE] FDCAN%u Status: TEC=%u, REC=%u, LEC=%u, ACT=%u, BO=%u\n", 
           can_num, tec, rec, lec, act, bo);
    
    /* 硬件连接诊断 */
    if (tec == 0 && rec == 0 && lec == 0) {
        printf("[HARDWARE] ⚠️ FDCAN%u可能的问题：\n", can_num);
        printf("  1. CAN收发器未正确连接或未使能（STB引脚）\n");
        printf("  2. CAN总线缺少终端电阻（60Ω）\n");
        printf("  3. CAN_H和CAN_L短路或开路\n");
        printf("  4. 收发器电源问题\n");
        return 1;
    }
    
    if (bo) {
        printf("[HARDWARE] ❌ FDCAN%u处于BUS-OFF状态！硬件连接严重异常\n", can_num);
        return 1;
    }
    
    if (tec > 127) {
        printf("[HARDWARE] ⚠️ FDCAN%u TEC过高（%u），可能存在硬件问题\n", can_num, tec);
        return 1;
    }
    
    if (lec != 0 && lec != 7) {
        printf("[HARDWARE] ⚠️ FDCAN%u最近错误码：%u\n", can_num, lec);
    }
    
    printf("[HARDWARE] ✅ FDCAN%u硬件连接基本正常\n", can_num);
    return 0;
}

/**
 * @brief       尝试恢复BUS-OFF的总线
 * @param       can_num: FDCAN编号（1或2）
 * @retval      0: 恢复成功, 1: 恢复失败
 */
uint8_t fdcan_recover_busoff(uint8_t can_num)
{
    FDCAN_HandleTypeDef *hfdcan;
    volatile uint8_t *busoff_flag;
    volatile uint32_t *recover_count;
    
    if (can_num == 1) {
        hfdcan = &hfdcan1;
        busoff_flag = &g_fdcan1_in_busoff;
        recover_count = &g_fdcan1_recover_count;
    } else if (can_num == 2) {
        hfdcan = &hfdcan2;
        busoff_flag = &g_fdcan2_in_busoff;
        recover_count = &g_fdcan2_recover_count;
    } else {
        return 1;
    }
    
    if (*busoff_flag == 0) {
        return 0;  /* 不在BUS-OFF状态 */
    }
    
    printf("[RECOVERY] FDCAN%u 尝试从BUS-OFF恢复...\n", can_num);
    
    /* 停止FDCAN */
    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) {
        printf("[RECOVERY] FDCAN%u Stop failed!\n", can_num);
        return 1;
    }
    
    /* 等待总线恢复 (128个11位隐性位时间，约1ms @ 1Mbps) */
    HAL_Delay(2);
    
    /* 清除错误状态 */
    hfdcan->Instance->ECR = 0xFFFFFFFF;
    hfdcan->Instance->PSR = 0xFFFFFFFF;
    
    /* 重新启动FDCAN */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        printf("[RECOVERY] FDCAN%u Start failed!\n", can_num);
        return 1;
    }
    
    /* 重新使能接收 */
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        printf("[RECOVERY] FDCAN%u Notification failed!\n", can_num);
        return 1;
    }
    
    /* 清除BUS-OFF标志 */
    *busoff_flag = 0;
    (*recover_count)++;
    
    printf("[RECOVERY] ✅ FDCAN%u 恢复成功! (第%u次恢复)\n", can_num, *recover_count);
    
    return 0;
}

/**
 * @brief       检查并尝试恢复BUS-OFF状态 (供系统定时调用)
 * @retval      无
 */
void fdcan_check_and_recover(void)
{
    /* 检查FDCAN1 */
    uint32_t psr1 = hfdcan1.Instance->PSR;
    uint8_t bo1 = (psr1 >> 7) & 0x1;
    if (bo1 && g_fdcan1_in_busoff == 0) {
        g_fdcan1_in_busoff = 1;
        g_fdcan1_busoff_count++;
        printf("[WARNING] FDCAN1 进入BUS-OFF状态! (第%u次)\n", g_fdcan1_busoff_count);
    }
    
    /* 检查FDCAN2 */
    uint32_t psr2 = hfdcan2.Instance->PSR;
    uint8_t bo2 = (psr2 >> 7) & 0x1;
    if (bo2 && g_fdcan2_in_busoff == 0) {
        g_fdcan2_in_busoff = 1;
        g_fdcan2_busoff_count++;
        printf("[WARNING] FDCAN2 进入BUS-OFF状态! (第%u次)\n", g_fdcan2_busoff_count);
    }
    
    /* 尝试自动恢复 (如果处于BUS-OFF状态且没有发送在进行) */
    uint32_t txfqs1 = hfdcan1.Instance->TXFQS;
    uint8_t tfqf1 = (txfqs1 >> 5) & 0x1;  /* TXFIFO Q Full */
    
    uint32_t txfqs2 = hfdcan2.Instance->TXFQS;
    uint8_t tfqf2 = (txfqs2 >> 5) & 0x1;
    
    if (g_fdcan1_in_busoff && !tfqf1) {
        fdcan_recover_busoff(1);
    }
    
    if (g_fdcan2_in_busoff && !tfqf2) {
        fdcan_recover_busoff(2);
    }
}

/**
 * @brief       显示FDCAN通信统计信息
 * @param       无
 * @retval      无
 */
void fdcan_show_statistics(void)
{
    printf("[STATISTICS] FDCAN通信统计信息:\n");
    printf("  FDCAN1: 成功发送=%u, 错误=%u, 成功率=%.1f%%\n", 
           g_fdcan1_send_count, g_fdcan1_error_count,
           g_fdcan1_send_count + g_fdcan1_error_count > 0 ? 
           (float)g_fdcan1_send_count / (g_fdcan1_send_count + g_fdcan1_error_count) * 100 : 0);
    
    printf("  FDCAN2: 成功发送=%u, 错误=%u, 成功率=%.1f%%\n", 
           g_fdcan2_send_count, g_fdcan2_error_count,
           g_fdcan2_send_count + g_fdcan2_error_count > 0 ? 
           (float)g_fdcan2_send_count / (g_fdcan2_send_count + g_fdcan2_error_count) * 100 : 0);
    
    uint32_t current_time = HAL_GetTick();
    if (g_last_successful_send > 0) {
        printf("  最后成功发送: %ums前\n", current_time - g_last_successful_send);
    } else {
        printf("  最后成功发送: 从未成功\n");
    }
}

/**
 * @brief       检查系统资源状态
 * @param       无
 * @retval      0: 资源正常, 1: 资源紧张
 */
uint8_t fdcan_check_system_resources(void)
{
    uint32_t current_time = HAL_GetTick();
    
    /* 检查运行时间 */
    if (g_system_start_time == 0) {
        g_system_start_time = current_time;
    }
    
    g_total_operation_time = current_time - g_system_start_time;
    
    /* 检查长时间运行稳定性 */
    if (g_total_operation_time > 3600000) { /* 1小时 */
        printf("[RESOURCE] 系统已运行 %u 分钟，建议重启\n", g_total_operation_time / 60000);
    }
    
    /* 检查通信间隔 */
    if (g_last_successful_send > 0 && (current_time - g_last_successful_send) > 10000) {
        printf("[RESOURCE] 警告：10秒内无成功通信，系统可能已停止\n");
        return 1;
    }
    
    /* 检查错误率 */
    uint32_t total_attempts = g_fdcan1_send_count + g_fdcan1_error_count + g_fdcan2_send_count + g_fdcan2_error_count;
    uint32_t total_errors = g_fdcan1_error_count + g_fdcan2_error_count;
    
    if (total_attempts > 1000 && total_errors > total_attempts * 0.3) {
        printf("[RESOURCE] 错误率过高(%.1f%%)，系统稳定性受影响\n", 
               (float)total_errors / total_attempts * 100);
        return 1;
    }
    
    /* 内存使用检查（简单堆检查） */
    void *test_alloc = malloc(100);
    if (test_alloc == NULL) {
        printf("[RESOURCE] 警告：内存分配失败，系统内存可能不足\n");
        return 1;
    }
    free(test_alloc);
    
    printf("[RESOURCE] 系统资源正常，已运行 %u 秒\n", g_total_operation_time / 1000);
    return 0;
}

/**
 * @brief       系统维护和清理
 * @param       无
 * @retval      无
 */
void fdcan_system_maintenance(void)
{
    printf("[MAINTENANCE] 执行系统维护...\n");
    
    /* 检查并尝试恢复BUS-OFF */
    fdcan_check_and_recover();
    
    /* 定期清理统计（防止溢出） */
    if (g_fdcan1_send_count > 10000 || g_fdcan2_send_count > 10000) {
        printf("[MAINTENANCE] 通信计数过高，执行统计重置\n");
        fdcan_reset_statistics();
    }
    
    /* 检查并重置FDCAN状态 */
    if (g_fdcan1_error_count > 500) {
        printf("[MAINTENANCE] FDCAN1错误过多，执行强制重置\n");
        fdcan_force_reset(1);
        g_fdcan1_error_count = 0;  /* 重置错误计数 */
    }
    
    if (g_fdcan2_error_count > 500) {
        printf("[MAINTENANCE] FDCAN2错误过多，执行强制重置\n");
        fdcan_force_reset(2);
        g_fdcan2_error_count = 0;  /* 重置错误计数 */
    }
    
    printf("[MAINTENANCE] 系统维护完成\n");
}

/**
 * @brief       监控FDCAN错误状态
 * @note        V6.5: 移除自动禁用逻辑，允许双CAN同时工作
 *              CAN1控制电机1-6，CAN2控制电机7-12
 * @param       无
 * @retval      0: 正常, 其他: 警告标志
 */
uint8_t fdcan2_monitor_and_protect(void)
{
    uint8_t warning_flags = 0;

    /* 读取FDCAN1状态寄存器 */
    uint32_t psr1 = hfdcan1.Instance->PSR;
    uint32_t ecr1 = hfdcan1.Instance->ECR;
    uint8_t tec1 = (ecr1 >> 0) & 0xFF;
    uint8_t rec1 = (ecr1 >> 8) & 0xFF;
    uint8_t ep1 = (psr1 >> 5) & 0x1;
    uint8_t bo1 = (psr1 >> 7) & 0x1;

    /* 读取FDCAN2状态寄存器 */
    uint32_t psr2 = hfdcan2.Instance->PSR;
    uint32_t ecr2 = hfdcan2.Instance->ECR;
    uint8_t tec2 = (ecr2 >> 0) & 0xFF;
    uint8_t rec2 = (ecr2 >> 8) & 0xFF;
    uint8_t ep2 = (psr2 >> 5) & 0x1;
    uint8_t bo2 = (psr2 >> 7) & 0x1;

    /* 静态计数器，用于定期报告 */
    static uint32_t monitor_count = 0;
    monitor_count++;

    /* 每100次监控打印一次状态（减少日志输出） */
    if (monitor_count % 100 == 0) {
        printf("[CAN Monitor] CAN1: TEC=%u REC=%u EP=%u BO=%u | CAN2: TEC=%u REC=%u EP=%u BO=%u\n",
               tec1, rec1, ep1, bo1, tec2, rec2, ep2, bo2);
    }

    /* 记录警告标志（不自动禁用） */
    if (bo1) {
        g_fdcan1_busoff_count++;
        warning_flags |= 0x01;
    }
    if (bo2) {
        g_fdcan2_busoff_count++;
        warning_flags |= 0x02;
    }
    if (ep1) {
        warning_flags |= 0x04;
    }
    if (ep2) {
        g_fdcan2_error_passive_count++;
        warning_flags |= 0x08;
    }

    return warning_flags;
}


/**
 * @brief       重置通信统计计数
 * @param       无
 * @retval      无
 */
void fdcan_reset_statistics(void)
{
    g_fdcan1_send_count = 0;
    g_fdcan2_send_count = 0;
    g_fdcan1_error_count = 0;
    g_fdcan2_error_count = 0;
    g_last_successful_send = 0;
    printf("[STATISTICS] 通信统计已重置\n");
}

/**
 * @brief       重置FDCAN控制器（强制恢复）
 * @param       can_num: FDCAN编号（1或2）
 * @retval      0: 成功, 1: 失败
 */
uint8_t fdcan_force_reset(uint8_t can_num)
{
    FDCAN_HandleTypeDef *handle;
    
    if (can_num == 1) {
        handle = &hfdcan1;
        printf("[RESET] Forcing FDCAN1 reset...\n");
    } else if (can_num == 2) {
        handle = &hfdcan2;
        printf("[RESET] Forcing FDCAN2 reset...\n");
    } else {
        printf("[ERROR] Invalid FDCAN number: %u\n", can_num);
        return 1;
    }
    
    /* 停止FDCAN */
    HAL_FDCAN_Stop(handle);
    HAL_Delay(100);
    
    /* 清除所有错误状态 */
    handle->Instance->ECR = 0;
    
    /* 重新初始化 */
    if (HAL_FDCAN_Init(handle) != HAL_OK) {
        printf("[ERROR] Failed to reinitialize FDCAN%u!\n", can_num);
        return 1;
    }
    
    /* 重新启动 */
    if (HAL_FDCAN_Start(handle) != HAL_OK) {
        printf("[ERROR] Failed to restart FDCAN%u!\n", can_num);
        return 1;
    }
    
    HAL_Delay(50);
    printf("[RESET] FDCAN%u reset completed successfully\n", can_num);
    return 0;
}

/* FDCAN1和FDCAN2的中断服务函数在stm32h7xx_it.c中定义 */

/**
 * @brief       接收FIFO0回调函数 - 统一处理FDCAN1和FDCAN2
 * @param       hfdcan:FDCAN句柄
 * @param       RxFifo0ITs:接收FIFO 0中断状态
 * @retval      无
 * @note        ✅ 简化版本：只在中断中释放信号量，数据读取由FDCANRxTask任务完成
 *              这样可以避免在中断中处理复杂逻辑，符合FreeRTOS最佳实践
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        /* ✅ 释放信号量，通知FDCANRxTask任务有新数据 */
        extern osSemaphoreId_t fdcan_rx_semaphore;
        if (fdcan_rx_semaphore != NULL) {
            osSemaphoreRelease(fdcan_rx_semaphore);
        }

        /* 重新激活接收中断（保持中断使能） */
        HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    }
}


