/**
  ******************************************************************************
  * @file    lan8720_driver.h
  * @author  Modified for LAN8720 Module Support
  * @brief   LAN8720 PHY driver header
  ******************************************************************************
  */

#ifndef __LAN8720_DRIVER_H
#define __LAN8720_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/* ==================== Configuration Section ==================== */

/* Uncomment and define if nRST pin is connected to a GPIO */
/* Example: PB0 is connected to LAN8720 nRST */
// #define LAN8720_RST_GPIO_Port   GPIOB
// #define LAN8720_RST_Pin         GPIO_PIN_0

/* ==================== Function Prototypes ==================== */

/**
  * @brief  Perform hardware reset of LAN8720 PHY
  * @param  None
  * @retval None
  */
void LAN8720_HardwareReset(void);

/**
  * @brief  Perform software reset of LAN8720 PHY
  * @param  phy_addr: PHY address (0x00 or 0x01)
  * @retval 0 if success, -1 if failed
  */
int32_t LAN8720_SoftwareReset(uint32_t phy_addr);

/**
  * @brief  Read LAN8720 PHY ID
  * @param  phy_addr: PHY address
  * @param  phy_id: Pointer to store PHY ID
  * @retval 0 if success, -1 if failed
  */
int32_t LAN8720_ReadPHY_ID(uint32_t phy_addr, uint32_t *phy_id);

/**
  * @brief  Auto-detect LAN8720 PHY address
  * @param  detected_addr: Pointer to store detected address
  * @retval 0 if success, -1 if not found
  */
int32_t LAN8720_DetectAddress(uint32_t *detected_addr);

/**
  * @brief  Initialize LAN8720 PHY
  * @param  phy_addr: Pointer to PHY address (set to 0xFF for auto-detection)
  * @retval 0 if success, negative if failed, 1 if success but no link
  */
int32_t LAN8720_Init(uint32_t *phy_addr);

/**
  * @brief  Get LAN8720 link status
  * @param  phy_addr: PHY address
  * @retval 1 if link up, 0 if link down, negative if error
  */
int32_t LAN8720_GetLinkState(uint32_t phy_addr);

#ifdef __cplusplus
}
#endif

#endif /* __LAN8720_DRIVER_H */
