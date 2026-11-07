/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : ethernetif.c
  * Description        : This file provides code for the configuration
  *                      of the ethernetif.c MiddleWare.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include "lan8742.h"
#include <string.h>
#include "cmsis_os.h"
#include "lwip/tcpip.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */
#include <stdio.h>  /* For printf debugging */
#include "lan8720_driver.h"  /* LAN8720 driver for external PHY modules */
/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT ( portMAX_DELAY )
/* Time to block waiting for transmissions to finish */
#define ETHIF_TX_TIMEOUT (2000U)
/* USER CODE BEGIN OS_THREAD_STACK_SIZE_WITH_RTOS */
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE ( 350 )
/* USER CODE END OS_THREAD_STACK_SIZE_WITH_RTOS */
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

/* ETH Setting  */
#define ETH_DMA_TRANSMIT_TIMEOUT               ( 20U )
#define ETH_TX_BUFFER_MAX             ((ETH_TX_DESC_CNT) * 2U)
/* ETH_RX_BUFFER_SIZE parameter is defined in lwipopts.h */

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/
/*
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx Buffers will be allocated from LwIP stack Rx memory pool,
          then passed to ETH HAL driver.
        - Tx Buffers will be allocated from LwIP stack memory heap,
          then passed to ETH HAL driver.

@Notes:
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_RX_DESC_CNT in ETH GUI (Rx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_TX_DESC_CNT in ETH GUI (Tx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
  2.b. Rx Buffers must have the same size: ETH_RX_BUFFER_SIZE, this value must
       passed to ETH DMA in the init field (heth.Init.RxBuffLen)
  2.c  The RX Ruffers addresses and sizes must be properly defined to be aligned
       to L1-CACHE line size (32 bytes).
*/

/* Data Type Definitions */
typedef enum
{
  RX_ALLOC_OK       = 0x00,
  RX_ALLOC_ERROR    = 0x01
} RxAllocStatusTypeDef;

typedef struct
{
  struct pbuf_custom pbuf_custom;
  uint8_t buff[(ETH_RX_BUFFER_SIZE + 31) & ~31] __ALIGNED(32);
} RxBuff_t;

/* Memory Pool Declaration */
#define ETH_RX_BUFFER_CNT             16U  /* FIXED: Increased from 12 to 16 for high-speed transfer */
LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");

/* Variable Definitions */
static uint8_t RxAllocStatus;

/* DEBUG: Global TX packet counter (volatile for debugger access) */
volatile uint32_t g_eth_tx_count = 0;
volatile uint32_t g_eth_tx_arp_count = 0;
volatile uint32_t g_eth_tx_ip_count = 0;

/* DEBUG: Global RX packet counter */
volatile uint32_t g_eth_rx_count = 0;
volatile uint32_t g_eth_rx_arp_count = 0;
volatile uint32_t g_eth_rx_ip_count = 0;

#if defined ( __ICCARM__ ) /*!< IAR Compiler */

#pragma location=0x30040000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30040060
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM ) /* MDK ARM Compiler */
__attribute__((at(0x30040000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30040060))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

osSemaphoreId RxPktSemaphore = NULL;   /* Semaphore to signal incoming packets */
osSemaphoreId TxPktSemaphore = NULL;   /* Semaphore to signal transmit packet complete */

/* Global Ethernet handle */
ETH_HandleTypeDef heth;

ETH_TxPacketConfig TxConfig;

/* Private function prototypes -----------------------------------------------*/
int32_t ETH_PHY_IO_Init(void);
int32_t ETH_PHY_IO_DeInit (void);
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal);
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal);
int32_t ETH_PHY_IO_GetTick(void);

lan8742_Object_t LAN8742;
lan8742_IOCtx_t  LAN8742_IOCtx = {ETH_PHY_IO_Init,
                                  ETH_PHY_IO_DeInit,
                                  ETH_PHY_IO_WriteReg,
                                  ETH_PHY_IO_ReadReg,
                                  ETH_PHY_IO_GetTick};

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/* Private functions ---------------------------------------------------------*/
void pbuf_free_custom(struct pbuf *p);

/**
  * @brief  Ethernet Rx Transfer completed callback
  * @param  handlerEth: ETH handler
  * @retval None
  */
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *handlerEth)
{
  /* V12.1: 移除printf避免中断阻塞 - 使用计数器替代调试输出 */
  /* 如需调试，可通过g_eth_irq_count变量查看中断计数 */
  osSemaphoreRelease(RxPktSemaphore);
}
/**
  * @brief  Ethernet Tx Transfer completed callback
  * @param  handlerEth: ETH handler
  * @retval None
  */
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *handlerEth)
{
  osSemaphoreRelease(TxPktSemaphore);
}
/**
  * @brief  Ethernet DMA transfer error callback
  * @param  handlerEth: ETH handler
  * @retval None
  */
void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *handlerEth)
{
  if((HAL_ETH_GetDMAError(handlerEth) & ETH_DMACSR_RBU) == ETH_DMACSR_RBU)
  {
     osSemaphoreRelease(RxPktSemaphore);
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
 * @brief In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{
  HAL_StatusTypeDef hal_eth_init_status = HAL_OK;
/* USER CODE BEGIN OS_THREAD_ATTR_CMSIS_RTOS_V2 */
  osThreadAttr_t attributes;
/* USER CODE END OS_THREAD_ATTR_CMSIS_RTOS_V2 */
  uint32_t duplex, speed = 0;
  int32_t PHYLinkState = 0;
  ETH_MACConfigTypeDef MACConf = {0};
  /* Start ETH HAL Init */

  uint8_t MACAddr[6];
  heth.Instance = ETH;

  /* V3.3: 使用STM32唯一ID生成唯一MAC地址 */
  /* STM32H7有96位UID, 位于0x1FF1E800 (参考RM0433) */
  uint32_t uid[3];
  uid[0] = *(uint32_t*)(0x1FF1E800);  /* UID[31:0] */
  uid[1] = *(uint32_t*)(0x1FF1E804);  /* UID[63:32] */
  uid[2] = *(uint32_t*)(0x1FF1E808);  /* UID[95:64] */

  /* 生成MAC地址 - 使用本地管理地址 + UID哈希 */
  MACAddr[0] = 0x02;  /* 本地管理地址 (Locally Administered Address) */
  MACAddr[1] = 0x80;  /* 组织唯一标识符低8位 */
  MACAddr[2] = 0xE1;  /* 保持原有厂商标识 */
  MACAddr[3] = (uid[0] >> 0) & 0xFF;   /* UID低字节 */
  MACAddr[4] = (uid[0] >> 8) & 0xFF;   /* UID次低字节 */
  MACAddr[5] = (uid[0] >> 16) & 0xFF;  /* UID第三字节 */

  printf("[ETH] Generated unique MAC: %02X:%02X:%02X:%02X:%02X:%02X (UID=0x%08X)\r\n",
         MACAddr[0], MACAddr[1], MACAddr[2],
         MACAddr[3], MACAddr[4], MACAddr[5], uid[0]);

  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1536;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  /* CRITICAL FIX: Initialize RX memory pool BEFORE HAL_ETH_Init */
  /* HAL_ETH_Init calls HAL_ETH_RxAllocateCallback which needs the pool */
  printf("[ETH] Initializing RX memory pool before HAL_ETH_Init...\r\n");
  printf("[ETH] RX_POOL size: %u buffers x %u bytes = %u total\r\n",
         ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), ETH_RX_BUFFER_CNT * sizeof(RxBuff_t));
  LWIP_MEMPOOL_INIT(RX_POOL);
  printf("[ETH] LWIP_MEMPOOL_INIT(RX_POOL) completed\r\n");

  hal_eth_init_status = HAL_ETH_Init(&heth);

  /* Check HAL_ETH_Init result */
  if (hal_eth_init_status != HAL_OK) {
    printf("[ETH] ERROR: HAL_ETH_Init() failed with status %d\r\n", hal_eth_init_status);
  } else {
    printf("[ETH] HAL_ETH_Init() succeeded\r\n");
  }

  /* Check if RX buffers were allocated during init */
  printf("[ETH] Checking RX buffer allocation after HAL_ETH_Init...\r\n");
  if (RxAllocStatus == RX_ALLOC_ERROR) {
    printf("[ETH] WARNING: RX buffer allocation had errors during init!\r\n");
  } else {
    printf("[ETH] RX buffer allocation status: OK\r\n");
  }

  /* CRITICAL FIX: Configure MAC packet filter to accept broadcast frames */
  /* This is required for ARP and ICMP to work properly */
  printf("[ETH] Configuring MAC packet filter...\r\n");

  ETH_MACFilterConfigTypeDef FilterConfig = {0};
  HAL_ETH_GetMACFilterConfig(&heth, &FilterConfig);

  /* Configure filter settings for normal operation */
  FilterConfig.PromiscuousMode = DISABLE;
  FilterConfig.ReceiveAllMode = DISABLE;
  FilterConfig.BroadcastFilter = DISABLE;  /* DISABLE means ACCEPT broadcast (inverted logic!) */
  FilterConfig.PassAllMulticast = ENABLE;  /* Pass all multicast for now */
  FilterConfig.DestAddrInverseFiltering = DISABLE;
  FilterConfig.HashUnicast = DISABLE;
  FilterConfig.HashMulticast = DISABLE;
  FilterConfig.HachOrPerfectFilter = DISABLE;
  FilterConfig.SrcAddrFiltering = DISABLE;
  FilterConfig.SrcAddrInverseFiltering = DISABLE;
  FilterConfig.ControlPacketsFilter = 0x00;  /* Block all control frames */

  HAL_ETH_SetMACFilterConfig(&heth, &FilterConfig);
  printf("[ETH] MAC filter configured - Broadcast: ACCEPTED\r\n");

  /* Configure MAC settings */
  ETH_MACConfigTypeDef MACConf_init = {0};
  HAL_ETH_GetMACConfig(&heth, &MACConf_init);

  MACConf_init.DropTCPIPChecksumErrorPacket = DISABLE;
  MACConf_init.ForwardRxErrorPacket = DISABLE;
  MACConf_init.ForwardRxUndersizedGoodPacket = DISABLE;

  HAL_ETH_SetMACConfig(&heth, &MACConf_init);

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;

  /* End ETH HAL Init */

  /* RX POOL already initialized before HAL_ETH_Init() */

#if LWIP_ARP || LWIP_ETHERNET
  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] =  heth.Init.MACAddr[0];
  netif->hwaddr[1] =  heth.Init.MACAddr[1];
  netif->hwaddr[2] =  heth.Init.MACAddr[2];
  netif->hwaddr[3] =  heth.Init.MACAddr[3];
  netif->hwaddr[4] =  heth.Init.MACAddr[4];
  netif->hwaddr[5] =  heth.Init.MACAddr[5];

  /* maximum transfer unit */
  netif->mtu = ETH_MAX_PAYLOAD;

  /* Accept broadcast address and ARP traffic */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  #if LWIP_ARP
    netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP;
  #else
    netif->flags |= NETIF_FLAG_BROADCAST;
  #endif /* LWIP_ARP */

  /* create a binary semaphore used for informing ethernetif of frame reception */
  printf("[ETH] Creating RxPktSemaphore...\r\n");
  RxPktSemaphore = osSemaphoreNew(1, 0, NULL);
  if (RxPktSemaphore == NULL) {
    printf("[ETH] ERROR: Failed to create RxPktSemaphore!\r\n");
  } else {
    printf("[ETH] RxPktSemaphore created successfully\r\n");
  }

  /* create a binary semaphore used for informing ethernetif of frame transmission */
  printf("[ETH] Creating TxPktSemaphore...\r\n");
  TxPktSemaphore = osSemaphoreNew(1, 0, NULL);
  if (TxPktSemaphore == NULL) {
    printf("[ETH] ERROR: Failed to create TxPktSemaphore!\r\n");
  } else {
    printf("[ETH] TxPktSemaphore created successfully\r\n");
  }

  /* create the task that handles the ETH_MAC */
/* USER CODE BEGIN OS_THREAD_NEW_CMSIS_RTOS_V2 */
  printf("[ETH] Creating EthIf task...\r\n");
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "EthIf";
  attributes.stack_size = INTERFACE_THREAD_STACK_SIZE;
  attributes.priority = osPriorityAboveNormal;  /* AboveNormal priority for network processing */
  osThreadId_t ethif_handle = osThreadNew(ethernetif_input, netif, &attributes);
  if (ethif_handle == NULL) {
    printf("[ETH] ERROR: Failed to create EthIf task!\r\n");
  } else {
    printf("[ETH] EthIf task created successfully\r\n");
  }
/* USER CODE END OS_THREAD_NEW_CMSIS_RTOS_V2 */

/* USER CODE BEGIN PHY_PRE_CONFIG */
  printf("[ETH] Initializing LAN8720 PHY Module...\r\n");
  printf("[ETH] This is an EXTERNAL LAN8720 module (not onboard LAN8742)\r\n");

  /* LAN8720 PHY Address: 0xFF means auto-detection */
  /* PHY address based on schematic: RXER/PHYAD0 pulled up via 10R resistor */
  /* This means PHYAD0=1, so PHY address = 0x01 */
  static uint32_t lan8720_phy_addr = 0x01;  /* Fixed address 0x01 (per schematic) */

  int32_t phy_init_status = LAN8720_Init(&lan8720_phy_addr);

  if (phy_init_status < 0) {
    printf("[ETH] ================================================================\r\n");
    printf("[ETH] ERROR: LAN8720 initialization FAILED! (Status: %d)\r\n", phy_init_status);
    printf("[ETH] ================================================================\r\n");
    printf("[ETH] Troubleshooting checklist:\r\n");
    printf("[ETH] \r\n");
    printf("[ETH] 1. HARDWARE CONNECTIONS:\r\n");
    printf("[ETH]    - MDIO (PA2) connected to LAN8720 MDIO pin?\r\n");
    printf("[ETH]    - MDC (PC1) connected to LAN8720 MDC pin?\r\n");
    printf("[ETH]    - REF_CLK (PA1) connected to LAN8720 nINT/REFCLKO pin?\r\n");
    printf("[ETH]    - All RMII data lines connected (CRS_DV, RXD0/1, TX_EN, TXD0/1)?\r\n");
    printf("[ETH] \r\n");
    printf("[ETH] 2. POWER SUPPLY:\r\n");
    printf("[ETH]    - LAN8720 VDD = 3.3V (check with multimeter)\r\n");
    printf("[ETH]    - All ground pins connected?\r\n");
    printf("[ETH] \r\n");
    printf("[ETH] 3. REF_CLK CONFIGURATION:\r\n");
    printf("[ETH]    - LAN8720 provides 50MHz clock on nINT/REFCLKO pin\r\n");
    printf("[ETH]    - Check with oscilloscope: 50MHz square wave on PA1?\r\n");
    printf("[ETH]    - LAN8720 pin 2 (nINT/REFCLKO) connected to STM32 PA1?\r\n");
    printf("[ETH] \r\n");
    printf("[ETH] 4. LED INDICATORS:\r\n");
    printf("[ETH]    - GREEN LED should be ON when powered (no need for cable)\r\n");
    printf("[ETH]    - YELLOW LED constant = initialization problem!\r\n");
    printf("[ETH]    - If GREEN off + YELLOW on = PHY not configured correctly\r\n");
    printf("[ETH] \r\n");
    printf("[ETH] 5. PHY ADDRESS CONFIGURATION:\r\n");
    printf("[ETH]    - LAN8720 address set by hardware pins (usually 0x00 or 0x01)\r\n");
    printf("[ETH]    - Try changing 'lan8720_phy_addr' to 0x00 or 0x01 if auto-detect fails\r\n");
    printf("[ETH] \r\n");
    printf("[ETH] 6. nRST PIN (OPTIONAL):\r\n");
    printf("[ETH]    - If nRST is connected to GPIO, define in lan8720_driver.h:\r\n");
    printf("[ETH]      #define LAN8720_RST_GPIO_Port GPIOx\r\n");
    printf("[ETH]      #define LAN8720_RST_Pin GPIO_PIN_x\r\n");
    printf("[ETH] ================================================================\r\n");

    netif_set_link_down(netif);
    netif_set_down(netif);
    return;
  }

  if (phy_init_status == 1) {
    printf("[ETH] WARNING: LAN8720 initialized but link is DOWN\r\n");
    printf("[ETH] - This is normal if Ethernet cable is not connected\r\n");
    printf("[ETH] - Link status will be checked periodically\r\n");
  }

  printf("[ETH] LAN8720 initialized successfully!\r\n");
  printf("[ETH] - PHY Address: 0x%02X\r\n", (unsigned int)lan8720_phy_addr);
/* USER CODE END PHY_PRE_CONFIG */

  /* REMOVED: Manual RX buffer allocation before HAL_ETH_Start_IT
   * This was being overwritten by HAL_ETH_Start_IT's internal initialization.
   * The proper way is to let HAL_ETH_Start_IT allocate buffers via callback.
   */
  printf("[ETH] RX buffers will be allocated by HAL_ETH_Start_IT via callback\r\n");

  /* Check link state using LAN8720 driver */
  int32_t link_state = LAN8720_GetLinkState(lan8720_phy_addr);

  if (link_state <= 0) {
    printf("[ETH] Link is DOWN\r\n");
    printf("[ETH] - Check Ethernet cable connection\r\n");
    printf("[ETH] - Check if cable is plugged into router/switch\r\n");
    printf("[ETH] - Link LED should be solid (not blinking)\r\n");

    /* CRITICAL: Even if link is down, we must start ETH to allow reception when link comes up */
    printf("[ETH] Starting ETH interface even without link (required for DMA setup)...\r\n");
    HAL_StatusTypeDef eth_start_status = HAL_ETH_Start_IT(&heth);
    if (eth_start_status != HAL_OK) {
      printf("[ETH] ERROR: HAL_ETH_Start_IT failed with status %d\r\n", eth_start_status);
    } else {
      printf("[ETH] HAL_ETH_Start_IT succeeded\r\n");

      /* CRITICAL DEBUG: Check RX descriptors immediately after HAL_ETH_Start_IT */
      ETH_DMADescTypeDef *rxDesc = (ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[0];
      printf("[ETH] *** IMMEDIATE CHECK after HAL_ETH_Start_IT ***\r\n");
      printf("[ETH] RxDesc[0].DESC2 (Buffer): 0x%08X\r\n", rxDesc->DESC2);
      printf("[ETH] RxDesc[1].DESC2 (Buffer): 0x%08X\r\n", ((ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[1])->DESC2);
      printf("[ETH] RxDesc[2].DESC2 (Buffer): 0x%08X\r\n", ((ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[2])->DESC2);
      printf("[ETH] RxDesc[3].DESC2 (Buffer): 0x%08X\r\n", ((ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[3])->DESC2);
    }

    /* Set interface up but link down - will be updated by link thread */
    netif_set_up(netif);
    netif_set_link_down(netif);
  } else {
    printf("[ETH] Link is UP!\r\n");

    /* For LAN8720, speed/duplex is typically auto-negotiated */
    /* We'll use default 100M Full-Duplex for now */
    duplex = ETH_FULLDUPLEX_MODE;
    speed = ETH_SPEED_100M;

    printf("[ETH] - Speed: 100 Mbps (auto-negotiated)\r\n");
    printf("[ETH] - Duplex: Full-Duplex (auto-negotiated)\r\n");

    /* Get MAC Config MAC */
    HAL_ETH_GetMACConfig(&heth, &MACConf);
    MACConf.DuplexMode = duplex;
    MACConf.Speed = speed;
    HAL_ETH_SetMACConfig(&heth, &MACConf);

    /* CRITICAL: Start ETH with interrupts */
    HAL_StatusTypeDef eth_start_status = HAL_ETH_Start_IT(&heth);
    if (eth_start_status != HAL_OK) {
      printf("[ETH] ERROR: HAL_ETH_Start_IT failed with status %d\r\n", eth_start_status);
    } else {
      printf("[ETH] HAL_ETH_Start_IT succeeded\r\n");

      /* CRITICAL DEBUG: Check RX descriptors immediately after HAL_ETH_Start_IT */
      ETH_DMADescTypeDef *rxDesc = (ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[0];
      printf("[ETH] *** IMMEDIATE CHECK after HAL_ETH_Start_IT ***\r\n");
      printf("[ETH] RxDesc[0].DESC2 (Buffer): 0x%08X\r\n", rxDesc->DESC2);
      printf("[ETH] RxDesc[1].DESC2 (Buffer): 0x%08X\r\n", ((ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[1])->DESC2);
      printf("[ETH] RxDesc[2].DESC2 (Buffer): 0x%08X\r\n", ((ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[2])->DESC2);
      printf("[ETH] RxDesc[3].DESC2 (Buffer): 0x%08X\r\n", ((ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[3])->DESC2);
    }

    netif_set_up(netif);
    netif_set_link_up(netif);
/* USER CODE BEGIN PHY_POST_CONFIG */
    printf("[ETH] Interface UP, IP configured\r\n");
/* USER CODE END PHY_POST_CONFIG */
  }

  if (hal_eth_init_status != HAL_OK)
  {
    Error_Handler();
  }
#endif /* LWIP_ARP || LWIP_ETHERNET */

/* USER CODE BEGIN LOW_LEVEL_INIT */

/* USER CODE END LOW_LEVEL_INIT */
}

/**
 * @brief This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  uint32_t i = 0U;
  struct pbuf *q = NULL;
  err_t errval = ERR_OK;
  ETH_BufferTypeDef Txbuffer[ETH_TX_DESC_CNT] = {0};

  /* DEBUG: Log packet transmission attempt */
  static uint32_t tx_count = 0;
  tx_count++;

  /* Update global counters */
  g_eth_tx_count++;

  /* Identify packet type for debugging */
  if (p->len >= 14) {
    uint8_t *eth_hdr = (uint8_t *)p->payload;
    uint16_t eth_type = (eth_hdr[12] << 8) | eth_hdr[13];
    if (eth_type == 0x0806) g_eth_tx_arp_count++;  /* ARP */
    else if (eth_type == 0x0800) g_eth_tx_ip_count++;  /* IPv4 */
  }

  /* Simple counter - no printf to avoid deadlock */
  static volatile uint32_t total_tx_count = 0;
  total_tx_count++;

  /* DISABLE printf in TX path - causes system hang
  if(tx_count <= 5) {
    printf("[ETH TX] Packet #%u: len=%u bytes\n", (unsigned int)tx_count, (unsigned int)p->tot_len);
  }
  */

  memset(Txbuffer, 0 , ETH_TX_DESC_CNT*sizeof(ETH_BufferTypeDef));

  for(q = p; q != NULL; q = q->next)
  {
    if(i >= ETH_TX_DESC_CNT)
      return ERR_IF;

    Txbuffer[i].buffer = q->payload;
    Txbuffer[i].len = q->len;

    if(i>0)
    {
      Txbuffer[i-1].next = &Txbuffer[i];
    }

    if(q->next == NULL)
    {
      Txbuffer[i].next = NULL;
    }

    i++;
  }

  TxConfig.Length = p->tot_len;
  TxConfig.TxBuffer = Txbuffer;
  TxConfig.pData = p;

  pbuf_ref(p);

  /* CRITICAL FIX: Clean D-Cache for DMA transmission on STM32H7 */
  /* Ensure data is written to RAM before DMA reads it */
  /* Note: Cache line size on Cortex-M7 is 32 bytes, must align addresses and sizes */
  for(q = p; q != NULL; q = q->next)
  {
    if(q->len > 0 && q->payload != NULL)
    {
      /* Align start address down to cache line boundary (32 bytes) */
      uint32_t addr = (uint32_t)q->payload;
      uint32_t aligned_addr = addr & ~0x1FU;  /* Clear lower 5 bits */

      /* Calculate aligned size (round up to cache line boundary) */
      uint32_t size = q->len + (addr - aligned_addr);
      uint32_t aligned_size = (size + 31U) & ~0x1FU;  /* Round up to 32-byte boundary */

      /* Clean the D-Cache for this buffer */
      SCB_CleanDCache_by_Addr((uint32_t*)aligned_addr, (int32_t)aligned_size);
    }
  }

  /* DISABLE: May cause deadlock in interrupt context
  if(tx_count <= 10) {
    printf("[ETH TX] D-Cache cleaned, calling HAL_ETH_Transmit_IT...\n");
  }
  */

  do
  {
    if(HAL_ETH_Transmit_IT(&heth, &TxConfig) == HAL_OK)
    {
      errval = ERR_OK;
    }
    else
    {
      uint32_t eth_error = HAL_ETH_GetError(&heth);

      if(eth_error & HAL_ETH_ERROR_BUSY)
      {
        /* Wait for descriptors to become available */
        osSemaphoreAcquire(TxPktSemaphore, ETHIF_TX_TIMEOUT);
        HAL_ETH_ReleaseTxPacket(&heth);
        errval = ERR_BUF;
      }
      else
      {
        /* Other error - log it */
        /* DISABLE: May cause deadlock
        if(tx_count <= 10) {
          printf("[ETH TX] ERROR: HAL_ETH_Transmit_IT failed, error=0x%08X\n", (unsigned int)eth_error);
        }
        */
        pbuf_free(p);
        errval =  ERR_IF;
      }
    }
  }while(errval == ERR_BUF);

  return errval;
}

/**
 * @brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
   */
static struct pbuf * low_level_input(struct netif *netif)
{
  struct pbuf *p = NULL;

  if(RxAllocStatus == RX_ALLOC_OK)
  {
    HAL_ETH_ReadData(&heth, (void **)&p);
  }

  return p;
}

/**
 * @brief This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input(void* argument)
{
  struct pbuf *p = NULL;
  struct netif *netif = (struct netif *) argument;
  static uint32_t rx_count = 0;  /* 接收计数器 */

  for( ;; )
  {
    if (osSemaphoreAcquire(RxPktSemaphore, TIME_WAITING_FOR_INPUT) == osOK)
    {
      do
      {
        p = low_level_input( netif );
        if (p != NULL)
        {
          rx_count++;
          /* 打印每一个接收到的包用于调试 */
          //printf("[ETH] RX packet #%lu, len=%u bytes\r\n", (unsigned long)rx_count, p->tot_len);

          if (netif->input( p, netif) != ERR_OK )
          {
            printf("[ETH] ERROR: netif->input failed!\r\n");
            pbuf_free(p);
          }
        }
      } while(p!=NULL);
    }
  }
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{
  err_t errval;
  errval = ERR_OK;

/* USER CODE BEGIN 5 */

/* USER CODE END 5 */

  return errval;

}
#endif /* LWIP_ARP */

/**
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  // MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
  netif->output = etharp_output;
#else
  /* The user should write its own code in low_level_output_arp_off function */
  netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
  * @brief  Custom Rx pbuf free callback
  * @param  pbuf: pbuf to be freed
  * @retval None
  */
void pbuf_free_custom(struct pbuf *p)
{
  struct pbuf_custom* custom_pbuf = (struct pbuf_custom*)p;
  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);

  /* If the Rx Buffer Pool was exhausted, signal the ethernetif_input task to
   * call HAL_ETH_GetRxDataBuffer to rebuild the Rx descriptors. */

  if (RxAllocStatus == RX_ALLOC_ERROR)
  {
    RxAllocStatus = RX_ALLOC_OK;
    osSemaphoreRelease(RxPktSemaphore);
  }
}

/* USER CODE BEGIN 6 */

/**
* @brief  Returns the current time in milliseconds
*         when LWIP_TIMERS == 1 and NO_SYS == 1
* @param  None
* @retval Current Time value
*/
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/* USER CODE END 6 */

/**
  * @brief  Initializes the ETH MSP.
  * @param  ethHandle: ETH handle
  * @retval None
  */

void HAL_ETH_MspInit(ETH_HandleTypeDef* ethHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspInit 0 */

  /* USER CODE END ETH_MspInit 0 */
    /* Enable Peripheral clock */
    __HAL_RCC_ETH1MAC_CLK_ENABLE();
    __HAL_RCC_ETH1TX_CLK_ENABLE();
    __HAL_RCC_ETH1RX_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init */
    /* 注意: ETH回调中使用了osSemaphoreRelease()，必须设置优先级 >= 6 */
    HAL_NVIC_SetPriority(ETH_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(ETH_IRQn);
  /* USER CODE BEGIN ETH_MspInit 1 */

  /* USER CODE END ETH_MspInit 1 */
  }
}

void HAL_ETH_MspDeInit(ETH_HandleTypeDef* ethHandle)
{
  if(ethHandle->Instance==ETH)
  {
  /* USER CODE BEGIN ETH_MspDeInit 0 */

  /* USER CODE END ETH_MspDeInit 0 */
    /* Disable Peripheral clock */
    __HAL_RCC_ETH1MAC_CLK_DISABLE();
    __HAL_RCC_ETH1TX_CLK_DISABLE();
    __HAL_RCC_ETH1RX_CLK_DISABLE();

    /**ETH GPIO Configuration
    PC1     ------> ETH_MDC
    PA1     ------> ETH_REF_CLK
    PA2     ------> ETH_MDIO
    PA7     ------> ETH_CRS_DV
    PC4     ------> ETH_RXD0
    PC5     ------> ETH_RXD1
    PB11     ------> ETH_TX_EN
    PB12     ------> ETH_TXD0
    PB13     ------> ETH_TXD1
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(ETH_IRQn);

  /* USER CODE BEGIN ETH_MspDeInit 1 */

  /* USER CODE END ETH_MspDeInit 1 */
  }
}

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
  * @brief  Initializes the MDIO interface GPIO and clocks.
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_Init(void)
{
  /* We assume that MDIO GPIO configuration is already done
     in the ETH_MspInit() else it should be done here
  */

  /* Configure the MDIO Clock */
  HAL_ETH_SetMDIOClockRange(&heth);

  return 0;
}

/**
  * @brief  De-Initializes the MDIO interface .
  * @param  None
  * @retval 0 if OK, -1 if ERROR
  */
int32_t ETH_PHY_IO_DeInit (void)
{
  return 0;
}

/**
  * @brief  Read a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  pRegVal: pointer to hold the register value
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_ReadReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t *pRegVal)
{
  if(HAL_ETH_ReadPHYRegister(&heth, DevAddr, RegAddr, pRegVal) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  Write a value to a PHY register through the MDIO interface.
  * @param  DevAddr: PHY port address
  * @param  RegAddr: PHY register address
  * @param  RegVal: Value to be written
  * @retval 0 if OK -1 if Error
  */
int32_t ETH_PHY_IO_WriteReg(uint32_t DevAddr, uint32_t RegAddr, uint32_t RegVal)
{
  if(HAL_ETH_WritePHYRegister(&heth, DevAddr, RegAddr, RegVal) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

/**
  * @brief  Get the time in millisecons used for internal PHY driver process.
  * @retval Time value
  */
int32_t ETH_PHY_IO_GetTick(void)
{
  return HAL_GetTick();
}

/**
  * @brief  Check the ETH link state then update ETH driver and netif link accordingly.
  * @retval None
  */
void ethernet_link_thread(void* argument)
{
  ETH_MACConfigTypeDef MACConf = {0};
  int32_t PHYLinkState = 0;
  uint32_t linkchanged = 0U, speed = 0U, duplex = 0U;

  struct netif *netif = (struct netif *) argument;
/* USER CODE BEGIN ETH link init */
  static uint32_t lan8720_phy_addr = 0x01;  /* Use the same address as in low_level_init */
  static uint32_t link_check_count = 0;
/* USER CODE END ETH link init */

  for(;;)
  {
  /* Use LAN8720 driver instead of LAN8742 */
  PHYLinkState = LAN8720_GetLinkState(lan8720_phy_addr);

  link_check_count++;
  /* Print link status every 50 checks (5 seconds) for debugging */
  if (link_check_count % 50 == 0) {
    printf("[ETH Link] Check #%lu: Link %s\r\n",
           (unsigned long)link_check_count,
           (PHYLinkState > 0) ? "UP" : "DOWN");
  }

  /* LAN8720 driver returns: 1=link up, 0=link down, negative=error */
  if(netif_is_link_up(netif) && (PHYLinkState <= 0))
  {
    /* Link went down */
    printf("[ETH Link] Link DOWN detected, stopping interface\r\n");
    HAL_ETH_Stop_IT(&heth);
    netif_set_down(netif);
    netif_set_link_down(netif);
  }
  else if(!netif_is_link_up(netif) && (PHYLinkState > 0))
  {
    /* Link came up - configure speed/duplex */
    printf("[ETH Link] Link UP detected, configuring interface\r\n");

    /* For LAN8720, we'll use fixed 100M Full-Duplex for now */
    /* You can read LAN8720 register 0x1F to get actual negotiated speed/duplex */
    duplex = ETH_FULLDUPLEX_MODE;
    speed = ETH_SPEED_100M;
    linkchanged = 1;

    if(linkchanged)
    {
      /* Get MAC Config MAC */
      HAL_ETH_GetMACConfig(&heth, &MACConf);
      MACConf.DuplexMode = duplex;
      MACConf.Speed = speed;
      HAL_ETH_SetMACConfig(&heth, &MACConf);
      HAL_ETH_Start_IT(&heth);
      netif_set_up(netif);
      netif_set_link_up(netif);
      printf("[ETH Link] Interface UP - 100M Full-Duplex\r\n");
    }
  }

/* USER CODE BEGIN ETH link Thread core code for User BSP */

/* USER CODE END ETH link Thread core code for User BSP */

    osDelay(100);
  }
}

void HAL_ETH_RxAllocateCallback(uint8_t **buff)
{
/* USER CODE BEGIN HAL ETH RxAllocateCallback */
  /* CRITICAL: NO printf() calls here! This is called during HAL_ETH_Init()
   * in a critical section that may cause deadlock with UART mutex */
  static uint32_t alloc_count = 0;
  static uint32_t alloc_fail_count = 0;

  struct pbuf_custom *p = LWIP_MEMPOOL_ALLOC(RX_POOL);

  alloc_count++;

  if (p)
  {
    /* Get the buff from the struct pbuf address. */
    *buff = (uint8_t *)p + offsetof(RxBuff_t, buff);
    p->custom_free_function = pbuf_free_custom;
    /* Initialize the struct pbuf.
    * This must be performed whenever a buffer's allocated because it may be
    * changed by lwIP or the app, e.g., pbuf_free decrements ref. */
    pbuf_alloced_custom(PBUF_RAW, 0, PBUF_REF, p, *buff, ETH_RX_BUFFER_SIZE);
  }
  else
  {
    RxAllocStatus = RX_ALLOC_ERROR;
    *buff = NULL;
    alloc_fail_count++;
  }
/* USER CODE END HAL ETH RxAllocateCallback */
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length)
{
/* USER CODE BEGIN HAL ETH RxLinkCallback */

  struct pbuf **ppStart = (struct pbuf **)pStart;
  struct pbuf **ppEnd = (struct pbuf **)pEnd;
  struct pbuf *p = NULL;

  /* Get the struct pbuf from the buff address. */
  p = (struct pbuf *)(buff - offsetof(RxBuff_t, buff));
  p->next = NULL;
  p->tot_len = 0;
  p->len = Length;

  /* Chain the buffer. */
  if (!*ppStart)
  {
    /* The first buffer of the packet. */
    *ppStart = p;
  }
  else
  {
    /* Chain the buffer to the end of the packet. */
    (*ppEnd)->next = p;
  }
  *ppEnd  = p;

  /* Update the total length of all the buffers of the chain. Each pbuf in the chain should have its tot_len
   * set to its own length, plus the length of all the following pbufs in the chain. */
  for (p = *ppStart; p != NULL; p = p->next)
  {
    p->tot_len += Length;
  }

  /* Invalidate data cache because Rx DMA's writing to physical memory makes it stale. */
  SCB_InvalidateDCache_by_Addr((uint32_t *)buff, Length);

/* USER CODE END HAL ETH RxLinkCallback */
}

void HAL_ETH_TxFreeCallback(uint32_t * buff)
{
/* USER CODE BEGIN HAL ETH TxFreeCallback */

  pbuf_free((struct pbuf *)buff);

/* USER CODE END HAL ETH TxFreeCallback */
}

/* USER CODE BEGIN 8 */

/* USER CODE END 8 */
