#include "ina226.h"

float INA226_getBusV(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
return (INA226_getBusVReg(I2CHandler, DevAddress));
}//获取总线电压值，返回以伏特为单位的浮点数值

float INA226_getCurrent(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
return (INA226_getCurrentReg(I2CHandler, DevAddress)*INA226_CURRENTLSB_INV);
}//获取电流值，返回以安培为单位的浮点数值。

float INA226_getPower(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
return (INA226_getPowerReg(I2CHandler, DevAddress)*INA226_POWERLSB_INV);
}//获取功率值，返回以瓦特为单位的浮点数值。

uint8_t INA226_setConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
uint8_t SentTable[3];
SentTable[0] = INA226_CONFIG;
SentTable[1] = (ConfigWord & 0xFF00) >> 8;
SentTable[2] = (ConfigWord & 0x00FF);
return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}//配置 INA226 电流/电压监测芯片的寄存器。

uint16_t INA226_getConfig(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_CONFIG};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取 INA226 电流/电压监测芯片的配置字并返回其值。

uint16_t INA226_getShuntV(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_SHUNTV};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取INA226芯片的电流检测电阻电压值

uint16_t INA226_getBusVReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_BUSV};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取 INA226 电流/电压监测芯片的总线电压值寄存器并返回其值。

uint8_t INA226_setCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
uint8_t SentTable[3];
SentTable[0] = INA226_CALIB;
SentTable[1] = (ConfigWord & 0xFF00) >> 8;
SentTable[2] = (ConfigWord & 0x00FF);
return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}//设置INA226芯片的校准寄存器

uint16_t INA226_getCalibrationReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_CALIB};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取INA226芯片的校准寄存器值

uint16_t INA226_getPowerReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_POWER};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取功率寄存器的值

uint16_t INA226_getCurrentReg(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_CURRENT};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取 INA226 芯片的电流寄存器值

uint16_t INA226_getManufID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_MANUF_ID};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取INA226芯片的制造商ID寄存器

uint16_t INA226_getDieID(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_DIE_ID};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取INA226芯片的Die ID寄存器

uint8_t INA226_setMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
uint8_t SentTable[3];
SentTable[0] = INA226_MASK;
SentTable[1] = (ConfigWord & 0xFF00) >> 8;
SentTable[2] = (ConfigWord & 0x00FF);
return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}//设置INA226芯片的掩码寄存器(该寄存器的每个位对应于INA226设备的不同事件。如果某个位被设置为1，则该事件的中断将被禁用。如果该位被设置为0，则该事件的中断将被启用。因此，掩码寄存器允许用户选择要使能或禁用的特定中断事件。)

uint16_t INA226_getMaskEnable(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_MASK};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取掩码寄存器的值

uint8_t INA226_setAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress, uint16_t ConfigWord) {
uint8_t SentTable[3];
SentTable[0] = INA226_ALERTL;
SentTable[1] = (ConfigWord & 0xFF00) >> 8;
SentTable[2] = (ConfigWord & 0x00FF);
return HAL_I2C_Master_Transmit(I2CHandler, DevAddress, SentTable, 3, INA226_I2CTIMEOUT);
}//设置INA226芯片的警报限制寄存器的函数

uint16_t INA226_getAlertLimit(I2C_HandleTypeDef *I2CHandler, uint16_t DevAddress) {
uint8_t SentTable[1] = {INA226_ALERTL};
uint8_t ReceivedTable[2];
HAL_I2C_Master_Transmit(I2CHandler,DevAddress, SentTable, 1, INA226_I2CTIMEOUT);
if (HAL_I2C_Master_Receive(I2CHandler,DevAddress, ReceivedTable, 2, INA226_I2CTIMEOUT) != HAL_OK) return 0xFF;
else return ((uint16_t)ReceivedTable[0]<<8 | ReceivedTable[1]);
}//读取INA226的报警限制寄存器内容
