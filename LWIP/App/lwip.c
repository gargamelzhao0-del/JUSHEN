/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
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
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */
#include "ethernetif.h"
#include <string.h>

/* USER CODE BEGIN 0 */
#include <stdio.h>  /* For printf debugging */
#include "param_config.h"  /* For network parameters from param system */
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
static void ethernet_link_status_updated(struct netif *netif);
/* ETH Variables initialization ----------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];
/* USER CODE BEGIN OS_THREAD_ATTR_CMSIS_RTOS_V2 */
#define INTERFACE_THREAD_STACK_SIZE ( 1024 )
osThreadAttr_t attributes;
/* USER CODE END OS_THREAD_ATTR_CMSIS_RTOS_V2 */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  NetworkParam_t net_param;
  bool use_param_system = (param_get_network(&net_param) == 0);

  if (use_param_system) {
    IP_ADDRESS[0] = net_param.ip_addr[0];
    IP_ADDRESS[1] = net_param.ip_addr[1];
    IP_ADDRESS[2] = net_param.ip_addr[2];
    IP_ADDRESS[3] = net_param.ip_addr[3];
    NETMASK_ADDRESS[0] = net_param.netmask[0];
    NETMASK_ADDRESS[1] = net_param.netmask[1];
    NETMASK_ADDRESS[2] = net_param.netmask[2];
    NETMASK_ADDRESS[3] = net_param.netmask[3];
    GATEWAY_ADDRESS[0] = net_param.gateway[0];
    GATEWAY_ADDRESS[1] = net_param.gateway[1];
    GATEWAY_ADDRESS[2] = net_param.gateway[2];
    GATEWAY_ADDRESS[3] = net_param.gateway[3];
  } else {
    IP_ADDRESS[0] = 192;
    IP_ADDRESS[1] = 168;
    IP_ADDRESS[2] = 1;
    IP_ADDRESS[3] = 100;
    NETMASK_ADDRESS[0] = 255;
    NETMASK_ADDRESS[1] = 255;
    NETMASK_ADDRESS[2] = 255;
    NETMASK_ADDRESS[3] = 0;
    GATEWAY_ADDRESS[0] = 0;
    GATEWAY_ADDRESS[1] = 0;
    GATEWAY_ADDRESS[2] = 0;
    GATEWAY_ADDRESS[3] = 0;
  }

/* USER CODE BEGIN IP_ADDRESSES */
  printf("\r\n");
  printf("=============================================\r\n");
  printf("        LwIP Stack Initialization\r\n");
  printf("=============================================\r\n");
  printf("IP Address:  %d.%d.%d.%d\r\n", IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  printf("Netmask:     %d.%d.%d.%d\r\n", NETMASK_ADDRESS[0], NETMASK_ADDRESS[1], NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  printf("Gateway:     %d.%d.%d.%d\r\n", GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
  printf("=============================================\r\n");
/* USER CODE END IP_ADDRESSES */

  /* Initialize the LwIP stack with RTOS */
  printf("[LwIP] Initializing TCP/IP stack...\r\n");
  tcpip_init( NULL, NULL );

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) with RTOS */
  printf("[LwIP] Adding network interface...\r\n");
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  /* We must always bring the network interface up connection or not... */
  netif_set_up(&gnetif);

  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);

  printf("[LwIP] Network interface configured\r\n");

  /* CRITICAL FIX: Verify ETH RX is enabled */
  extern ETH_HandleTypeDef heth;
  printf("\r\n");
  printf("==============================================\r\n");
  printf("[ETH] *** RX INTERRUPT DIAGNOSTIC v2.0 ***\r\n");
  printf("==============================================\r\n");
  printf("[ETH] Checking RX DMA status...\r\n");
  printf("[ETH] DMA gState: %u\r\n", heth.gState);
  printf("[ETH] DMA RxDescList: %p\r\n", (void*)heth.RxDescList.RxDesc);

  /* Check if RX interrupts are already enabled */
  uint32_t dma_ier = READ_REG(heth.Instance->DMACIER);
  printf("[ETH] Current DMACIER: 0x%08X\r\n", dma_ier);
  printf("[ETH]   - RIE (RX interrupt): %s\r\n", (dma_ier & ETH_DMACIER_RIE) ? "ENABLED" : "DISABLED");
  printf("[ETH]   - NIE (Normal interrupt): %s\r\n", (dma_ier & ETH_DMACIER_NIE) ? "ENABLED" : "DISABLED");

  /* Force enable RX interrupts if not already */
  __HAL_ETH_DMA_ENABLE_IT(&heth, (ETH_DMACIER_NIE | ETH_DMACIER_RIE));
  printf("[ETH] RX interrupts force-enabled\r\n");

  /* Verify interrupts were enabled */
  dma_ier = READ_REG(heth.Instance->DMACIER);
  printf("[ETH] Updated DMACIER: 0x%08X\r\n", dma_ier);

  /* CRITICAL: Check RX DMA descriptor chain */
  printf("\r\n[ETH] === RX DMA Descriptor Diagnostics ===\r\n");
  ETH_DMADescTypeDef *rxDesc = (ETH_DMADescTypeDef *)heth.RxDescList.RxDesc[0];
  printf("[ETH] RxDesc[0] address: %p\r\n", (void*)rxDesc);
  printf("[ETH] RxDesc[0].DESC0 (RX Buffer Addr): 0x%08X\r\n", rxDesc->DESC0);
  printf("[ETH] RxDesc[0].BackupAddr0: 0x%08X\r\n", rxDesc->BackupAddr0);
  printf("[ETH] RxDesc[0].DESC1: 0x%08X\r\n", rxDesc->DESC1);
  printf("[ETH] RxDesc[0].DESC2: 0x%08X\r\n", rxDesc->DESC2);
  printf("[ETH] RxDesc[0].DESC3: 0x%08X\r\n", rxDesc->DESC3);
  printf("[ETH] RxDesc OWN bit: %s\r\n", (rxDesc->DESC3 & ETH_DMARXNDESCRF_OWN) ? "DMA" : "CPU");

  /* CRITICAL FIX: Check if RX buffer is NULL (DESC0 holds buffer address) */
  if (rxDesc->DESC0 == 0) {
    printf("[ETH] !!! CRITICAL ERROR: RX Buffer (DESC0) is NULL! !!!\r\n");
    printf("[ETH] This causes Fatal Bus Error when DMA tries to write!\r\n");
    printf("[ETH] RX buffer allocation failed during init.\r\n");
  } else {
    printf("[ETH] RX Buffer allocation OK: 0x%08X\r\n", rxDesc->DESC0);
  }

  /* Check NVIC interrupt enable status */
  printf("\r\n[ETH] === NVIC Interrupt Diagnostics ===\r\n");
  printf("[ETH] ETH_IRQn = %d\r\n", ETH_IRQn);
  uint32_t nvic_iser = NVIC->ISER[ETH_IRQn >> 5];
  uint32_t nvic_bit = 1 << (ETH_IRQn & 0x1F);
  printf("[ETH] NVIC ISER register: 0x%08X\r\n", nvic_iser);
  printf("[ETH] ETH interrupt in NVIC: %s\r\n", (nvic_iser & nvic_bit) ? "ENABLED" : "DISABLED");
  printf("[ETH] ETH interrupt priority: %u\r\n", NVIC_GetPriority(ETH_IRQn));

  /* Check MAC and DMA status registers */
  printf("\r\n[ETH] === MAC/DMA Status ===\r\n");
  printf("[ETH] MACCR: 0x%08X\r\n", READ_REG(heth.Instance->MACCR));
  printf("[ETH] MACPFR (Packet Filter): 0x%08X\r\n", READ_REG(heth.Instance->MACPFR));
  printf("[ETH] DMACSR: 0x%08X\r\n", READ_REG(heth.Instance->DMACSR));
  printf("[ETH] DMADSR: 0x%08X\r\n", READ_REG(heth.Instance->DMADSR));

  /* Check MAC hardware statistics counters (MMC registers) */
  printf("\r\n[ETH] === MAC Hardware RX Statistics ===\r\n");
  printf("[ETH] Rx Unicast Good (MMCRUPGR): %u\r\n", READ_REG(heth.Instance->MMCRUPGR));
  printf("[ETH] Rx CRC Error (MMCRCRCEPR): %u\r\n", READ_REG(heth.Instance->MMCRCRCEPR));
  printf("[ETH] Rx Alignment Error (MMCRAEPR): %u\r\n", READ_REG(heth.Instance->MMCRAEPR));

  /* Check RX queue and FIFO status */
  printf("[ETH] MTLRQDR (RX Queue Debug): 0x%08X\r\n", READ_REG(heth.Instance->MTLRQDR));
  printf("[ETH] MACRFCR (RX Flow Control): 0x%08X\r\n", READ_REG(heth.Instance->MACRFCR));

  /* Check DMA RX descriptor tail pointer */
  printf("[ETH] DMACRDTPR (RX Desc Tail): 0x%08X\r\n", READ_REG(heth.Instance->DMACRDTPR));
  printf("[ETH] DMACRDRLR (RX Desc Length): 0x%08X\r\n", READ_REG(heth.Instance->DMACRDRLR));

  /* Decode MACPFR (critical for RX filtering) */
  uint32_t macpfr = READ_REG(heth.Instance->MACPFR);
  printf("\r\n[ETH] === MAC Packet Filter Analysis ===\r\n");
  printf("[ETH] PR (Promiscuous): %s\r\n", (macpfr & 0x1) ? "ENABLED" : "DISABLED");
  printf("[ETH] PM (Pass All Multicast): %s\r\n", (macpfr & 0x10) ? "ENABLED" : "DISABLED");
  printf("[ETH] RA (Receive All): %s\r\n", (macpfr & 0x80000000) ? "ENABLED" : "DISABLED");

  /* CRITICAL FIX: Enable promiscuous mode to receive all packets */
  printf("\r\n[ETH] *** Enabling Promiscuous Mode ***\r\n");
  SET_BIT(heth.Instance->MACPFR, ETH_MACPFR_PR);
  printf("[ETH] MACPFR after fix: 0x%08X\r\n", READ_REG(heth.Instance->MACPFR));

  /* Check if RX DMA is actually started */
  uint32_t dmacrcr = READ_REG(heth.Instance->DMACRCR);
  printf("\r\n[ETH] === DMA RX Control Register ===\r\n");
  printf("[ETH] DMACRCR: 0x%08X\r\n", dmacrcr);
  printf("[ETH]   - SR (Start/Stop Receive): %s\r\n", (dmacrcr & ETH_DMACRCR_SR) ? "STARTED" : "STOPPED");
  printf("[ETH]   - RBSZ (RX Buffer Size): %u bytes\r\n", (dmacrcr >> 1) & 0x3FFF);

  /* If RX is stopped, force start it */
  if (!(dmacrcr & ETH_DMACRCR_SR)) {
    printf("[ETH] !!! WARNING: RX DMA is STOPPED! Force starting...\r\n");
    SET_BIT(heth.Instance->DMACRCR, ETH_DMACRCR_SR);
    printf("[ETH] DMACRCR after fix: 0x%08X\r\n", READ_REG(heth.Instance->DMACRCR));
  }

  printf("\r\n[LwIP] Initialization complete!\r\n");
  printf("=============================================\r\n\r\n");

  /* Create the Ethernet link handler thread */
/* USER CODE BEGIN H7_OS_THREAD_NEW_CMSIS_RTOS_V2 */
  memset(&attributes, 0x0, sizeof(osThreadAttr_t));
  attributes.name = "EthLink";
  attributes.stack_size = INTERFACE_THREAD_STACK_SIZE;
  attributes.priority = osPriorityBelowNormal;
  osThreadNew(ethernet_link_thread, &gnetif, &attributes);
/* USER CODE END H7_OS_THREAD_NEW_CMSIS_RTOS_V2 */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
static void ethernet_link_status_updated(struct netif *netif)
{
  if (netif_is_up(netif))
  {
/* USER CODE BEGIN 5 */
/* USER CODE END 5 */
  }
  else /* netif is down */
  {
/* USER CODE BEGIN 6 */
/* USER CODE END 6 */
  }
}

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
  sd = 0; // dummy code
/* USER CODE END 7 */

  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
  recved_bytes = 0; // dummy code
/* USER CODE END 9 */
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
  recved_bytes = 0; // dummy code
/* USER CODE END 10 */
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

