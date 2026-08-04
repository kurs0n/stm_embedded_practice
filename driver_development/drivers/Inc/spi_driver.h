#ifndef INC_SPI_DRIVER_H 
#define INC_SPI_DRIVER_H

#include "STM32F303REx.h"

/*
* @SPI_DeviceMode
*/
#define SPI_DEVICE_MODE_MASTER 1
#define SPI_DEVICE_MODE_SLAVE 0

/*
* @SPI_BusConfig
*/
#define SPI_BUS_CONFIG_FD 1
#define SPI_BUS_CONFIG_HD 2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 4

/*
* @SPI_SclkSpeed
*/
#define SPI_SCLK_SPEED_DIV2 0
#define SPI_SCLK_SPEED_DIV4 1
#define SPI_SCLK_SPEED_DIV8 2
#define SPI_SCLK_SPEED_DIV16 3
#define SPI_SCLK_SPEED_DIV32 4
#define SPI_SCLK_SPEED_DIV64 5
#define SPI_SCLK_SPEED_DIV128 6
#define SPI_SCLK_SPEED_DIV256 7

/*
* @SPI_DFF
*/
#define SPI_DFF_8BITS  7
#define SPI_DFF_16BITS 15

/*
* @SPI_CPOL
*/
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
* @SPI_CPHA
*/
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
* @SPI_SSM
*/
#define SPI_SSM_EN 1
#define SPI_SSM_DI 0

/*
* @SPI_SSI
*/
#define SPI_SSI_EN 1  
#define SPI_SSI_DI 0

/*
* @SPI_SSOE
*/
#define SPI_SSOE_EN 1
#define SPI_SSOE_DI 0 

/*
* @SPI_FRXTH
*/
#define SPI_FRXTH_EN 1
#define SPI_FRXTH_DI 0

/*
* @SPI_BIDIOE
*/
#define SPI_BIDIOE_EN 1 
#define SPI_BIDIOE_DI 0

typedef struct 
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL; 
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
    uint8_t SPI_SSI;
    uint8_t SPI_SSOE;
    uint8_t SPI_FRXTH;
    uint8_t SPI_BIDIOE;
} SPI_Config_t;

typedef struct 
{
    SPI_RegDef_t *pSPIx; 
    SPI_Config_t SPIConfig; 
    uint8_t *pTxBuffer; 
    uint8_t *pRxBuffer;
    uint32_t TxLen;
    uint32_t RxLen;
    uint8_t TxState;
    uint8_t RxState;
} SPI_Handle_t;

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi); // peripherial clock setup

void SPI_Init(SPI_Handle_t *pSPIHandle); // init and deinitialization
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_Enable(SPI_Handle_t *pSPIHandle);
void SPI_Disable(SPI_Handle_t *pSPIHandle);
// read and write data
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

// IRQ ISR Handling
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

// APPLICATION CALLBACK
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv);

// SPI specific macros 
#define SPI_READY 0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

// SPI possible application events (for interrupts)
#define SPI_EVENT_TX_CMPLT 1
#define SPI_EVENT_RX_CMPLT 2
#define SPI_EVENT_OVR_CMPLT 3

// generic macros

#define ENABLE 1
#define DISABLE 0

#endif
