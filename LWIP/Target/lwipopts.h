/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : Target/lwipopts.h
  * Description        : This file overrides LwIP stack default configuration
  *                      done in opt.h file.
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

/* Define to prevent recursive inclusion --------------------------------------*/
#ifndef __LWIPOPTS__H__
#define __LWIPOPTS__H__

#include "main.h"

/*-----------------------------------------------------------------------------*/
/* Current version of LwIP supported by CubeMx: 2.1.2 -*/
/*-----------------------------------------------------------------------------*/

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */
/* Disable SLIP interface - not used in this project */
#define LWIP_SLIP 0

/* USER CODE END 0 */

#ifdef __cplusplus
 extern "C" {
#endif

/* STM32CubeMX Specific Parameters (not defined in opt.h) ---------------------*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- WITH_RTOS enabled (Since FREERTOS is set) -----*/
#define WITH_RTOS 1
/*----- CHECKSUM_BY_HARDWARE enabled -----*/
#define CHECKSUM_BY_HARDWARE 1
/*-----------------------------------------------------------------------------*/

/* LwIP Stack Parameters (modified compared to initialization value in opt.h) -*/
/* Parameters set in STM32CubeMX LwIP Configuration GUI -*/
/*----- Default value in ETH configuration GUI in CubeMx: 1524 -----*/
#define ETH_RX_BUFFER_SIZE 1536
/*----- Value in opt.h for MEM_ALIGNMENT: 1 -----*/
#define MEM_ALIGNMENT 4
/*----- Default Value for MEM_SIZE: 1600 ---*/
#define MEM_SIZE 24576  /* FIXED: Increased from 14336 to 24576 (24KB) for high-speed data transfer */
/*----- Default Value for H7 devices: 0x30004000 -----*/
#define LWIP_RAM_HEAP_POINTER 0x30004000
/*----- Value supported for H7 devices: 1 -----*/
#define LWIP_SUPPORT_CUSTOM_PBUF 1
/*----- NEW: Enable TX single pbuf optimization for zero-copy -----*/
#define LWIP_NETIF_TX_SINGLE_PBUF 1
/*----- Value in opt.h for LWIP_ETHERNET: LWIP_ARP || PPPOE_SUPPORT -*/
#define LWIP_ETHERNET 1
/*----- Value in opt.h for LWIP_DNS_SECURE: (LWIP_DNS_SECURE_RAND_XID | LWIP_DNS_SECURE_NO_MULTIPLE_OUTSTANDING | LWIP_DNS_SECURE_RAND_SRC_PORT) -*/
#define LWIP_DNS_SECURE 7
/*----- Value in opt.h for TCP_SND_QUEUELEN: (4*TCP_SND_BUF + (TCP_MSS - 1))/TCP_MSS -----*/
#define TCP_SND_QUEUELEN 20  /* FIXED: Increased from 9 to 20 for high throughput */

/*----- TCP Send Buffer - Increased for high throughput -----*/
#define TCP_SND_BUF 8192  /* 8KB send buffer - increased from default 2920 */
#define TCP_MSS 1460      /* Maximum Segment Size */

/*----- FIX: Increase TCP receive window for stability -----*/
#define TCP_WND 8192      /* 8KB receive window - increased from default 2920 */

/*----- Value in opt.h for TCP_SNDLOWAT: LWIP_MIN(LWIP_MAX(((TCP_SND_BUF)/2), (2 * TCP_MSS) + 1), (TCP_SND_BUF) - 1) -----*/
#define TCP_SNDLOWAT 1071
/*----- Value in opt.h for TCP_SNDQUEUELOWAT: LWIP_MAX(TCP_SND_QUEUELEN)/2, 5) -*/
#define TCP_SNDQUEUELOWAT 5
/*----- Value in opt.h for TCP_WND_UPDATE_THRESHOLD: LWIP_MIN(TCP_WND/4, TCP_MSS*4) -----*/
#define TCP_WND_UPDATE_THRESHOLD 1460  /* FIXED: Increased from 536 to 1460 (1 MSS) */
/*----- Default Value for LWIP_NETIF_STATUS_CALLBACK: 0 ---*/
#define LWIP_NETIF_STATUS_CALLBACK 1
/*----- Value in opt.h for LWIP_NETIF_LINK_CALLBACK: 0 -----*/
#define LWIP_NETIF_LINK_CALLBACK 1
/*----- Value in opt.h for TCPIP_THREAD_STACKSIZE: 0 -----*/
#define TCPIP_THREAD_STACKSIZE 2048  /* FIXED: Increased from 1024 to 2048 for stability */
/*----- Value in opt.h for TCPIP_THREAD_PRIO: 1 -----*/
#define TCPIP_THREAD_PRIO 24
/*----- Value in opt.h for TCPIP_MBOX_SIZE: 0 -----*/
#define TCPIP_MBOX_SIZE 32  /* FIXED: Increased from 6 to 32 for high-throughput TCP */
/*----- Value in opt.h for SLIPIF_THREAD_STACKSIZE: 0 -----*/
/* Disabled - SLIP interface not used */
/* #define SLIPIF_THREAD_STACKSIZE 1024 */
/*----- Value in opt.h for SLIPIF_THREAD_PRIO: 1 -----*/
/* Disabled - SLIP interface not used */
/* #define SLIPIF_THREAD_PRIO 3 */
/*----- Value in opt.h for DEFAULT_THREAD_STACKSIZE: 0 -----*/
#define DEFAULT_THREAD_STACKSIZE 1024
/*----- Value in opt.h for DEFAULT_THREAD_PRIO: 1 -----*/
#define DEFAULT_THREAD_PRIO 3
/*----- Value in opt.h for DEFAULT_UDP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_UDP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_TCP_RECVMBOX_SIZE: 0 -----*/
#define DEFAULT_TCP_RECVMBOX_SIZE 6
/*----- Value in opt.h for DEFAULT_ACCEPTMBOX_SIZE: 0 -----*/
#define DEFAULT_ACCEPTMBOX_SIZE 6
/*----- Value in opt.h for RECV_BUFSIZE_DEFAULT: INT_MAX -----*/
#define RECV_BUFSIZE_DEFAULT 2000000000
/*----- Value in opt.h for LWIP_STATS: 1 -----*/
#define LWIP_STATS 1  /* FIXED: Enabled for performance monitoring (was 0) */
/*----- NEW: Additional buffer pool settings for high performance -----*/
/* V3.3: 增加缓冲区大小以支持高频率数据传输 */
#define MEMP_NUM_PBUF 48       /* 增加PBUF数量 */
#define MEMP_NUM_TCP_PCB 10
#define MEMP_NUM_TCP_PCB_LISTEN 5
#define MEMP_NUM_TCP_SEG 96    /* 增加TCP段数量 */
#define PBUF_POOL_SIZE 64      /* 增加PBUF池大小 */

/*----- FIX: Disable TCP out-of-order queue to prevent assertion errors -----*/
#define TCP_QUEUE_OOSEQ 0

/*----- V3.3: 启用TCP保活机制 -----*/
#define LWIP_TCP_KEEPALIVE  1
#define TCP_KEEPIDLE_DEFAULT  15000   /* 15秒无数据后开始保活 */
#define TCP_KEEPINTVL_DEFAULT 1000    /* 保活间隔1秒 */
#define TCP_KEEPCNT_DEFAULT   3       /* 3次保活失败后断开 */

/*----- V3.3: 增加TCP发送缓冲区 -----*/
#undef TCP_SND_BUF
#define TCP_SND_BUF 16384   /* 16KB send buffer */

#undef TCP_WND
#define TCP_WND 16384       /* 16KB receive window */

#undef TCP_SND_QUEUELEN
#define TCP_SND_QUEUELEN 32 /* 增加发送队列长度 */
/*----- Value in opt.h for CHECKSUM_GEN_IP: 1 -----*/
#define CHECKSUM_GEN_IP 0  /* Hardware checksum */
/*----- Value in opt.h for CHECKSUM_GEN_UDP: 1 -----*/
#define CHECKSUM_GEN_UDP 0  /* Hardware checksum */
/*----- Value in opt.h for CHECKSUM_GEN_TCP: 1 -----*/
#define CHECKSUM_GEN_TCP 0  /* Hardware checksum */
/*----- Value in opt.h for CHECKSUM_GEN_ICMP6: 1 -----*/
#define CHECKSUM_GEN_ICMP6 0  /* Hardware checksum */
/*----- Value in opt.h for CHECKSUM_CHECK_IP: 1 -----*/
#define CHECKSUM_CHECK_IP 0  /* Hardware checksum */
/*----- Value in opt.h for CHECKSUM_CHECK_UDP: 1 -----*/
#define CHECKSUM_CHECK_UDP 0  /* Hardware checksum */
/*----- Value in opt.h for CHECKSUM_CHECK_TCP: 1 -----*/
#define CHECKSUM_CHECK_TCP 0  /* Hardware checksum */
/*----- Value in opt.h for CHECKSUM_CHECK_ICMP6: 1 -----*/
#define CHECKSUM_CHECK_ICMP6 0
/*-----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/* Disable SLIP interface - not used in this project */
#define LWIP_SLIP 0

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /*__LWIPOPTS__H__ */
