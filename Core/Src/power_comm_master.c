#include "power_comm_master.h"
#include <string.h>

static void CS_Select(PowerCommMaster_t *comm)
{
    if (comm->cs_port != NULL) {
        HAL_GPIO_WritePin(comm->cs_port, comm->cs_pin, GPIO_PIN_RESET);
    }
}

static void CS_Deselect(PowerCommMaster_t *comm)
{
    if (comm->cs_port != NULL) {
        HAL_GPIO_WritePin(comm->cs_port, comm->cs_pin, GPIO_PIN_SET);
    }
}

static bool I2C_ReadRegister(PowerCommMaster_t *comm, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (comm->hi2c == NULL) {
        return false;
    }
    
    HAL_StatusTypeDef status;
    
    for (uint8_t retry = 0; retry < POWER_COMM_RETRY_COUNT; retry++) {
        status = HAL_I2C_Mem_Read(comm->hi2c, POWER_SLAVE_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, POWER_COMM_TIMEOUT_MS);
        
        if (status == HAL_OK) {
            comm->rx_count++;
            return true;
        }
    }
    
    comm->error_count++;
    return false;
}

static bool I2C_WriteRegister(PowerCommMaster_t *comm, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (comm->hi2c == NULL) {
        return false;
    }
    
    HAL_StatusTypeDef status;
    
    for (uint8_t retry = 0; retry < POWER_COMM_RETRY_COUNT; retry++) {
        status = HAL_I2C_Mem_Write(comm->hi2c, POWER_SLAVE_I2C_ADDR << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, POWER_COMM_TIMEOUT_MS);
        
        if (status == HAL_OK) {
            comm->tx_count++;
            return true;
        }
    }
    
    comm->error_count++;
    return false;
}

static bool SPI_Transfer(PowerCommMaster_t *comm, uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    if (comm->hspi == NULL) {
        return false;
    }
    
    HAL_StatusTypeDef status;
    
    CS_Select(comm);
    
    for (uint8_t retry = 0; retry < POWER_COMM_RETRY_COUNT; retry++) {
        status = HAL_SPI_TransmitReceive(comm->hspi, tx_data, rx_data, len, POWER_COMM_TIMEOUT_MS);
        
        if (status == HAL_OK) {
            CS_Deselect(comm);
            comm->rx_count++;
            comm->tx_count++;
            return true;
        }
    }
    
    CS_Deselect(comm);
    comm->error_count++;
    return false;
}

static bool SPI_ReadRegister(PowerCommMaster_t *comm, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t tx_buf[POWER_TOTAL_DATA_SIZE + 4];
    uint8_t rx_buf[POWER_TOTAL_DATA_SIZE + 4];
    
    memset(tx_buf, 0, sizeof(tx_buf));
    tx_buf[0] = reg | 0x80;
    tx_buf[1] = (len >> 8) & 0xFF;
    tx_buf[2] = len & 0xFF;
    
    if (!SPI_Transfer(comm, tx_buf, rx_buf, len + 4)) {
        return false;
    }
    
    memcpy(data, &rx_buf[4], len);
    return true;
}

static bool ReadRegister(PowerCommMaster_t *comm, uint8_t reg, uint8_t *data, uint16_t len)
{
    if (comm->comm_mode == POWER_COMM_I2C) {
        return I2C_ReadRegister(comm, reg, data, len);
    } else {
        return SPI_ReadRegister(comm, reg, data, len);
    }
}

static uint16_t CalculateChecksum(const uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint16_t)(~sum + 1);
}

void PowerCommMaster_Init(PowerCommMaster_t *comm, I2C_HandleTypeDef *hi2c, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    memset(comm, 0, sizeof(PowerCommMaster_t));
    
    comm->hi2c = hi2c;
    comm->hspi = hspi;
    comm->cs_port = cs_port;
    comm->cs_pin = cs_pin;
    
    comm->comm_mode = (hi2c != NULL) ? POWER_COMM_I2C : POWER_COMM_SPI;
    comm->slave_online = false;
    comm->offline_count = 0;
    
    if (cs_port != NULL) {
        CS_Deselect(comm);
    }
    
    comm->init_ok = true;
}

bool PowerCommMaster_ReadBattery(PowerCommMaster_t *comm, PowerBatteryStatus_t *battery)
{
    if (!comm->init_ok || battery == NULL) {
        return false;
    }
    
    return ReadRegister(comm, POWER_REG_BATTERY, (uint8_t*)battery, sizeof(PowerBatteryStatus_t));
}

bool PowerCommMaster_ReadMotor(PowerCommMaster_t *comm, PowerMotorStatus_t *motor)
{
    if (!comm->init_ok || motor == NULL) {
        return false;
    }
    
    return ReadRegister(comm, POWER_REG_MOTOR, (uint8_t*)motor, sizeof(PowerMotorStatus_t));
}

bool PowerCommMaster_ReadFault(PowerCommMaster_t *comm, PowerFaultStatus_t *fault)
{
    if (!comm->init_ok || fault == NULL) {
        return false;
    }
    
    return ReadRegister(comm, POWER_REG_FAULT, (uint8_t*)fault, sizeof(PowerFaultStatus_t));
}

bool PowerCommMaster_ReadSystem(PowerCommMaster_t *comm, PowerSystemStatus_t *system)
{
    if (!comm->init_ok || system == NULL) {
        return false;
    }
    
    return ReadRegister(comm, POWER_REG_SYSTEM, (uint8_t*)system, sizeof(PowerSystemStatus_t));
}

bool PowerCommMaster_ReadAll(PowerCommMaster_t *comm, PowerBatteryStatus_t *battery, PowerMotorStatus_t *motor, PowerFaultStatus_t *fault, PowerSystemStatus_t *system)
{
    if (!comm->init_ok) {
        return false;
    }
    
    uint8_t buffer[POWER_TOTAL_DATA_SIZE + 2];
    
    if (!ReadRegister(comm, POWER_REG_ALL, buffer, POWER_TOTAL_DATA_SIZE + 2)) {
        return false;
    }
    
    uint16_t received_checksum = ((uint16_t)buffer[POWER_TOTAL_DATA_SIZE] << 8) | buffer[POWER_TOTAL_DATA_SIZE + 1];
    uint16_t calculated_checksum = CalculateChecksum(buffer, POWER_TOTAL_DATA_SIZE);
    
    if (received_checksum != calculated_checksum) {
        comm->error_count++;
        return false;
    }
    
    uint16_t offset = 0;
    
    if (battery != NULL) {
        memcpy(battery, &buffer[offset], sizeof(PowerBatteryStatus_t));
    }
    offset += sizeof(PowerBatteryStatus_t);
    
    if (motor != NULL) {
        memcpy(motor, &buffer[offset], sizeof(PowerMotorStatus_t));
    }
    offset += sizeof(PowerMotorStatus_t);
    
    if (fault != NULL) {
        memcpy(fault, &buffer[offset], sizeof(PowerFaultStatus_t));
    }
    offset += sizeof(PowerFaultStatus_t);
    
    if (system != NULL) {
        memcpy(system, &buffer[offset], sizeof(PowerSystemStatus_t));
    }
    
    return true;
}

bool PowerCommMaster_ReadEvent(PowerCommMaster_t *comm, EventType_t *event_type, uint8_t *event_data)
{
    if (!comm->init_ok || event_type == NULL) {
        return false;
    }
    
    uint8_t buffer[8];
    
    if (!ReadRegister(comm, POWER_REG_EVENT, buffer, 8)) {
        return false;
    }
    
    *event_type = (EventType_t)buffer[0];
    if (event_data != NULL) {
        *event_data = buffer[1];
    }
    
    return true;
}

bool PowerCommMaster_ReadHeartbeat(PowerCommMaster_t *comm, uint8_t *bms_online, uint8_t *motor_online, uint8_t *system_state, uint8_t *soc)
{
    if (!comm->init_ok) {
        return false;
    }
    
    uint8_t buffer[4];
    
    if (!ReadRegister(comm, POWER_REG_HEARTBEAT, buffer, 4)) {
        return false;
    }
    
    if (bms_online != NULL) {
        *bms_online = buffer[0];
    }
    if (motor_online != NULL) {
        *motor_online = buffer[1];
    }
    if (system_state != NULL) {
        *system_state = buffer[2];
    }
    if (soc != NULL) {
        *soc = buffer[3];
    }
    
    return true;
}

bool PowerCommMaster_SwitchCommMode(PowerCommMaster_t *comm, PowerCommMode_t new_mode)
{
    if (!comm->init_ok) {
        return false;
    }
    
    if (new_mode == POWER_COMM_I2C && comm->hi2c == NULL) {
        return false;
    }
    if (new_mode == POWER_COMM_SPI && comm->hspi == NULL) {
        return false;
    }
    
    uint8_t tx_data[2] = { (uint8_t)new_mode, 0x00 };
    
    bool result;
    if (comm->comm_mode == POWER_COMM_I2C) {
        result = I2C_WriteRegister(comm, POWER_REG_COMM_MODE, tx_data, 2);
    } else {
        uint8_t rx_buf[4];
        uint8_t tx_buf[4] = { POWER_REG_COMM_MODE, 2, new_mode, 0 };
        result = SPI_Transfer(comm, tx_buf, rx_buf, 4);
    }
    
    if (result) {
        comm->comm_mode = new_mode;
    }
    
    return result;
}

bool PowerCommMaster_CheckSlaveOnline(PowerCommMaster_t *comm)
{
    uint16_t version = PowerCommMaster_GetVersion(comm);
    
    if (version != 0 && (version >> 8) == (POWER_DATA_VERSION >> 8)) {
        comm->slave_online = true;
        comm->offline_count = 0;
        return true;
    }
    
    comm->offline_count++;
    if (comm->offline_count >= 5) {
        comm->slave_online = false;
    }
    
    return false;
}

uint16_t PowerCommMaster_GetVersion(PowerCommMaster_t *comm)
{
    if (!comm->init_ok) {
        return 0;
    }
    
    uint8_t buffer[4];
    
    if (!ReadRegister(comm, POWER_REG_VERSION, buffer, 4)) {
        return 0;
    }
    
    return ((uint16_t)buffer[0] << 8) | buffer[1];
}
