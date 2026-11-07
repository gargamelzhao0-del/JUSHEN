/**
  ******************************************************************************
  * @file    lan8720_driver.c
  * @author  Modified for LAN8720 Module Support
  * @brief   LAN8720 PHY driver with fixed address and hardware reset support
  ******************************************************************************
  * This driver is specifically designed for LAN8720 external modules that:
  * - Use fixed PHY address (typically 0x00 or 0x01)
  * - May require hardware reset via GPIO
  * - Have different register map than LAN8742
  ******************************************************************************
  */

#include "lan8720_driver.h"
#include "stm32h7xx_hal.h"
#include "cmsis_os.h"  /* For osDelay() */
#include <stdio.h>
#include <stdbool.h>

/* LAN8720 PHY Address - Common addresses are 0x00 or 0x01 */
#define LAN8720_PHY_ADDRESS_0   0x00
#define LAN8720_PHY_ADDRESS_1   0x01

/* LAN8720 Register Definitions */
#define LAN8720_BCR             0x00  /* Basic Control Register */
#define LAN8720_BSR             0x01  /* Basic Status Register  */
#define LAN8720_PHYI1R          0x02  /* PHY Identifier 1       */
#define LAN8720_PHYI2R          0x03  /* PHY Identifier 2       */
#define LAN8720_SR              0x1F  /* Special Register       */

/* LAN8720 BCR Bit Definitions */
#define LAN8720_BCR_SOFT_RESET  0x8000  /* Soft Reset */
#define LAN8720_BCR_LOOPBACK    0x4000  /* Loopback */
#define LAN8720_BCR_SPEED_SEL   0x2000  /* Speed Select (1=100M)  */
#define LAN8720_BCR_AUTONEG     0x1000  /* Auto-Negotiation Enable */
#define LAN8720_BCR_POWER_DOWN  0x0800  /* Power Down */
#define LAN8720_BCR_ISOLATE     0x0400  /* Isolate */
#define LAN8720_BCR_RESTART_AN  0x0200  /* Restart Auto-Negotiation */
#define LAN8720_BCR_DUPLEX_MODE 0x0100  /* Duplex Mode (1=Full) */

/* LAN8720 BSR Bit Definitions */
#define LAN8720_BSR_LINK_STATUS 0x0004  /* Link Status */
#define LAN8720_BSR_AUTONEG_ABLE 0x0008  /* Auto-Negotiation Ability */
#define LAN8720_BSR_AUTONEG_COMP 0x0020  /* Auto-Negotiation Complete */

/* LAN8720 SR Bit Definitions (Address 0x1F) */
#define LAN8720_SR_SPEED_MASK   0x001C  /* Speed indication bits */
#define LAN8720_SR_SPEED_10HALF 0x0004  /* 10BASE-T Half-Duplex */
#define LAN8720_SR_SPEED_10FULL 0x0014  /* 10BASE-T Full-Duplex */
#define LAN8720_SR_SPEED_100HALF 0x0008  /* 100BASE-TX Half-Duplex */
#define LAN8720_SR_SPEED_100FULL 0x0018  /* 100BASE-TX Full-Duplex */

/* External ETH handle (defined in ethernetif.c) */
extern ETH_HandleTypeDef heth;

/**
  * @brief  Perform hardware reset of LAN8720 PHY (if nRST pin is connected)
  * @param  None
  * @retval None
  * @note   If nRST pin is not connected to GPIO, this function does nothing
  */
void LAN8720_HardwareReset(void)
{
#ifdef LAN8720_RST_GPIO_Port
    printf("[LAN8720] Performing hardware reset...\r\n");

    /* Pull nRST low */
    HAL_GPIO_WritePin(LAN8720_RST_GPIO_Port, LAN8720_RST_Pin, GPIO_PIN_RESET);
    osDelay(pdMS_TO_TICKS(50));  /* Hold reset for 50ms (PHY datasheet requires min 100us) */

    /* Release nRST */
    HAL_GPIO_WritePin(LAN8720_RST_GPIO_Port, LAN8720_RST_Pin, GPIO_PIN_SET);
    osDelay(pdMS_TO_TICKS(100));  /* Wait for PHY to complete reset (typical 50-100ms) */

    printf("[LAN8720] Hardware reset complete\r\n");
#else
    printf("[LAN8720] No hardware reset pin configured\r\n");
#endif
}

/**
  * @brief  Perform software reset of LAN8720 PHY
  * @param  phy_addr: PHY address (0x00 or 0x01)
  * @retval 0 if success, -1 if failed
  */
int32_t LAN8720_SoftwareReset(uint32_t phy_addr)
{
    uint32_t reg_value = 0;
    uint32_t timeout = 0;

    printf("[LAN8720] Performing software reset (PHY=0x%02X)...\r\n", (unsigned int)phy_addr);

    /* Set soft reset bit */
    HAL_StatusTypeDef write_status = HAL_ETH_WritePHYRegister(&heth, phy_addr, LAN8720_BCR, LAN8720_BCR_SOFT_RESET);
    printf("[LAN8720] WritePHYRegister status: %d\r\n", write_status);

    if (write_status != HAL_OK) {
        printf("[LAN8720] ERROR: Failed to write reset bit!\r\n");
        return -1;
    }

    /* Wait for reset complete (reset bit self-clears) */
    timeout = 0;
    do {
        HAL_StatusTypeDef read_status = HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_BCR, &reg_value);

        if (read_status != HAL_OK) {
            printf("[LAN8720] ERROR: Failed to read BCR during reset (timeout=%u)!\r\n", timeout);
            return -1;
        }

        /* Print progress every 100ms */
        if (timeout % 100 == 0) {
            printf("[LAN8720] Waiting for reset... timeout=%u, BCR=0x%04X\r\n", timeout, (unsigned int)reg_value);
        }

        osDelay(pdMS_TO_TICKS(1));  /* Use osDelay for FreeRTOS */
        timeout++;
    } while ((reg_value & LAN8720_BCR_SOFT_RESET) && (timeout < 500));

    if (timeout >= 500) {
        printf("[LAN8720] ERROR: Software reset timeout! Final BCR=0x%04X\r\n", (unsigned int)reg_value);
        return -1;
    }

    printf("[LAN8720] Software reset complete (%u ms)\r\n", (unsigned int)timeout);
    return 0;
}

/**
  * @brief  Read LAN8720 PHY ID
  * @param  phy_addr: PHY address
  * @param  phy_id: Pointer to store PHY ID
  * @retval 0 if success, -1 if failed
  */
int32_t LAN8720_ReadPHY_ID(uint32_t phy_addr, uint32_t *phy_id)
{
    uint32_t id1 = 0, id2 = 0;

    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_PHYI1R, &id1) != HAL_OK) {
        return -1;
    }

    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_PHYI2R, &id2) != HAL_OK) {
        return -1;
    }

    *phy_id = (id1 << 16) | id2;
    return 0;
}

/**
  * @brief  Auto-detect LAN8720 PHY address
  * @param  detected_addr: Pointer to store detected address
  * @retval 0 if success, -1 if not found
  */
int32_t LAN8720_DetectAddress(uint32_t *detected_addr)
{
    uint32_t phy_id = 0;
    uint32_t test_addresses[] = {LAN8720_PHY_ADDRESS_0, LAN8720_PHY_ADDRESS_1};

    printf("[LAN8720] Auto-detecting PHY address...\r\n");

    for (int i = 0; i < 2; i++) {
        printf("[LAN8720] Trying address 0x%02X...\r\n", (unsigned int)test_addresses[i]);

        if (LAN8720_ReadPHY_ID(test_addresses[i], &phy_id) == 0) {
            /* LAN8720 PHY ID is typically 0x0007C0Fx */
            if ((phy_id & 0xFFFFFFF0) == 0x0007C0F0) {
                *detected_addr = test_addresses[i];
                printf("[LAN8720] Found PHY at address 0x%02X (ID=0x%08X)\r\n",
                       (unsigned int)test_addresses[i], (unsigned int)phy_id);
                return 0;
            }
        }
    }

    printf("[LAN8720] ERROR: PHY not found at any address!\r\n");
    return -1;
}

/**
  * @brief  Initialize LAN8720 PHY
  * @param  phy_addr: Pointer to PHY address (will be auto-detected if *phy_addr = 0xFF)
  * @retval 0 if success, negative if failed
  */
int32_t LAN8720_Init(uint32_t *phy_addr)
{
    uint32_t reg_value = 0;
    uint32_t phy_id = 0;

    printf("[LAN8720] Starting PHY initialization...\r\n");

    /* Step 1: Hardware reset (if available) */
    LAN8720_HardwareReset();

    /* Step 2: Auto-detect PHY address if not specified */
    if (*phy_addr == 0xFF) {
        if (LAN8720_DetectAddress(phy_addr) != 0) {
            printf("[LAN8720] ERROR: Failed to detect PHY address!\r\n");
            printf("[LAN8720] - Check MDIO/MDC connections\r\n");
            printf("[LAN8720] - Check PHY power supply (3.3V)\r\n");
            printf("[LAN8720] - Check REF_CLK signal (50MHz)\r\n");
            return -1;
        }
    } else {
        printf("[LAN8720] Using fixed PHY address: 0x%02X\r\n", (unsigned int)*phy_addr);
    }

    /* Step 3: Read and verify PHY ID */
    if (LAN8720_ReadPHY_ID(*phy_addr, &phy_id) != 0) {
        printf("[LAN8720] ERROR: Failed to read PHY ID!\r\n");
        return -2;
    }

    printf("[LAN8720] PHY ID: 0x%08X\r\n", (unsigned int)phy_id);

    /* Verify it's a SMSC LAN87xx family chip */
    if ((phy_id & 0xFFFFFFF0) != 0x0007C0F0) {
        printf("[LAN8720] WARNING: Unexpected PHY ID! Expected 0x0007C0Fx\r\n");
    }

    /* Step 4: Software reset with retry mechanism */
    bool reset_success = false;
    for (int retry = 0; retry < 3 && !reset_success; retry++) {
        if (retry > 0) {
            printf("[LAN8720] Software reset retry %d...\r\n", retry);
            osDelay(pdMS_TO_TICKS(50));
        }
        
        uint32_t reg_value = LAN8720_BCR_SOFT_RESET;
        if (HAL_ETH_WritePHYRegister(&heth, *phy_addr, LAN8720_BCR, reg_value) == HAL_OK) {
            osDelay(pdMS_TO_TICKS(10));
            
            uint32_t timeout = 0;
            do {
                HAL_ETH_ReadPHYRegister(&heth, *phy_addr, LAN8720_BCR, &reg_value);
                osDelay(pdMS_TO_TICKS(1));
                timeout++;
            } while ((reg_value & LAN8720_BCR_SOFT_RESET) && (timeout < 100));
            
            if (timeout < 100) {
                reset_success = true;
                printf("[LAN8720] Software reset complete (%u ms)\r\n", timeout);
            }
        }
    }
    
    if (!reset_success) {
        printf("[LAN8720] Software reset failed, continuing without reset\r\n");
        osDelay(pdMS_TO_TICKS(100));
    }

    /* Step 5: Enable auto-negotiation */
    if (HAL_ETH_ReadPHYRegister(&heth, *phy_addr, LAN8720_BCR, &reg_value) != HAL_OK) {
        printf("[LAN8720] ERROR: Failed to read BCR!\r\n");
        return -4;
    }

    reg_value |= LAN8720_BCR_AUTONEG | LAN8720_BCR_RESTART_AN;
    if (HAL_ETH_WritePHYRegister(&heth, *phy_addr, LAN8720_BCR, reg_value) != HAL_OK) {
        printf("[LAN8720] ERROR: Failed to enable auto-negotiation!\r\n");
        return -5;
    }

    printf("[LAN8720] Auto-negotiation enabled\r\n");

    /* Step 6: Wait for link up (with timeout) - SKIP WAIT, let system continue */
    printf("[LAN8720] Checking link status (non-blocking)...\r\n");
    if (HAL_ETH_ReadPHYRegister(&heth, *phy_addr, LAN8720_BSR, &reg_value) == HAL_OK) {
        if (reg_value & LAN8720_BSR_LINK_STATUS) {
            printf("[LAN8720] Link is UP!\r\n");

            /* Read negotiated speed/duplex */
            if (HAL_ETH_ReadPHYRegister(&heth, *phy_addr, LAN8720_SR, &reg_value) == HAL_OK) {
                uint32_t speed_bits = reg_value & LAN8720_SR_SPEED_MASK;
                const char *speed_str = "Unknown";

                switch (speed_bits) {
                    case LAN8720_SR_SPEED_10HALF:
                        speed_str = "10BASE-T Half-Duplex";
                        break;
                    case LAN8720_SR_SPEED_10FULL:
                        speed_str = "10BASE-T Full-Duplex";
                        break;
                    case LAN8720_SR_SPEED_100HALF:
                        speed_str = "100BASE-TX Half-Duplex";
                        break;
                    case LAN8720_SR_SPEED_100FULL:
                        speed_str = "100BASE-TX Full-Duplex";
                        break;
                }
                printf("[LAN8720] Speed/Duplex: %s\r\n", speed_str);
            }
        } else {
            printf("[LAN8720] Link is DOWN (auto-negotiation in progress)\r\n");
            printf("[LAN8720] System will continue, link will establish shortly\r\n");
        }
    }

    printf("[LAN8720] Initialization complete\r\n");

    return 0;  /* Always return success, let network stack handle link status */
}

/**
  * @brief  Get LAN8720 link status
  * @param  phy_addr: PHY address
  * @retval 1 if link up, 0 if link down, negative if error
  */
int32_t LAN8720_GetLinkState(uint32_t phy_addr)
{
    uint32_t reg_value = 0;

    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, LAN8720_BSR, &reg_value) != HAL_OK) {
        return -1;
    }

    return (reg_value & LAN8720_BSR_LINK_STATUS) ? 1 : 0;
}
