#include "i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ConfigureSlaveConnection(I2C_RegDef_t *pI2Cx);

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    switch(EnorDi){
        case ENABLE:{
            if (pI2Cx == I2C1){
                I2C1_PCLK_EN();
            } else if(pI2Cx == I2C2){
                I2C2_PCLK_EN();
            }
            break;
        }
        case DISABLE:{
            if (pI2Cx == I2C1){
                I2C1_PCLK_DI();
            } else if(pI2Cx == I2C2){
                I2C2_PCLK_DI();
            }
            break;
        }
    }
}

void I2C_Init(I2C_Handle_t* pI2CHandle)
{ // just configure SCL I2C 
    uint32_t timing_register = 0;

    timing_register = pI2CHandle->I2C_Config.I2C_SCLSpeed;
    
    pI2CHandle->pI2Cx->TIMINGR = timing_register;
}


void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t slaveAddr)
{
    pI2CHandle->pI2Cx->CR2 = 0; //reset 
    // generate start condition
    pI2CHandle->TxLen = Len;
    pI2CHandle->TxBuffer = pTxbuffer;
    I2C_ConfigureSlaveConnection(pI2CHandle);

    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    for(uint8_t i=0; i < pI2CHandle->TxLen; i++){
        while(!(pI2CHandle->pI2Cx->ISR & I2C_ISR_TXIS)){
        }
        pI2CHandle->pI2Cx->TXDR=data[i];
    }    
    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx){
    pI2Cx->CR2 |= (1 << I2C_CR2_START);
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx){
    pI2Cx->CR2 |= (1 << I2C_CR2_STOP);
}

static void I2C_ConfigureSlaveConnection(I2C_Handle_t *pI2CHandle){
    pI2CHandle->pI2Cx->CR2 |= (pI2CHandle->I2C_Config.I2C_SlaveDeviceAddress << I2C_CR2_ADD0);
    pI2CHandle->pI2Cx->CR2 |= (pI2CHandle->TxLen << I2C_CR2_NBYTES);
}
