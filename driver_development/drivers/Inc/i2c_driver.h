#ifndef INC_I2C_DRIVER_H
#define INC_I2C_DRIVER_H

#include "STM32F303REx.h"

typedef struct // for nucleo f3 I2C interface was completely redesigned comparing to f4, f1 (need to write it differently)
{
    uint32_t I2C_SCLSpeed;
    uint8_t I2C_DeviceAddress;
} I2C_Config_t;

typedef struct
{
    I2C_RegDef_t *pI2Cx;
    I2C_Config_t I2C_Config;
} I2C_Handle_t;

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

void I2C_Init(I2C_Handle_t* pI2CHandle);

#endif