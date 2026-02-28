#include "spi_driver.h"

void inline __attribute__((always_inline)) delay(uint32_t delay)
{
   while(delay--) __asm("");
}

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
    switch(EnorDi){
        case ENABLE:{
            if(pSPIx == SPI1){
                SPI1_PCLK_EN();
            } else if(pSPIx == SPI2) {
                SPI2_PCLK_EN();
            } else if(pSPIx == SPI3) {
                SPI3_PCLK_EN();
            } else if(pSPIx == SPI4) {
                SPI4_PCLK_EN();
            }
            break;
        }
        case DISABLE:{
            if(pSPIx == SPI1){
                SPI1_PCLK_DI();
            } else if(pSPIx == SPI2) {
                SPI2_PCLK_DI();
            } else if(pSPIx == SPI3) {
                SPI3_PCLK_DI();
            } else if(pSPIx == SPI4) {
                SPI4_PCLK_DI();
            }
            break;
        }
    }
}
 
void SPI_Init(SPI_Handle_t *pSPIHandle){
    uint32_t control_register1 = 0;
    uint32_t control_register2 = 0;
    
    pSPIHandle->pSPIx->CR1 = 0;
    pSPIHandle->pSPIx->CR2 = 0;

    pSPIHandle->pSPIx->CR1 &= ~(ENABLE << SPI_CR1_SPE);

    control_register1 |= pSPIHandle->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;
    
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
        control_register1 &= ~(ENABLE<<SPI_CR1_BIDIMODE); 
    } else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
        control_register1 |= ENABLE<<SPI_CR1_BIDIMODE;
    } else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
        control_register1 &= ~(ENABLE<<SPI_CR1_BIDIMODE); 
        control_register1 |= ENABLE<<SPI_CR1_RXONLY;
    }
    
    control_register1 |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);
    control_register1 |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
    control_register1 |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
    control_register1 |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;
    control_register1 |= pSPIHandle->SPIConfig.SPI_BIDIOE << SPI_CR1_BIDIOE;
    control_register1 |= (pSPIHandle->SPIConfig.SPI_SSI << SPI_CR1_SSI);

    control_register2 |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR2_DS);
    control_register2 |= (pSPIHandle->SPIConfig.SPI_SSOE << SPI_CR2_SSOE);
    control_register2 |= (pSPIHandle->SPIConfig.SPI_FRXTH << SPI_CR2_FRXTH); 
    
    pSPIHandle->pSPIx->CR2 = control_register2;
    pSPIHandle->pSPIx->CR1 = control_register1;
}

void SPI_Enable(SPI_Handle_t *pSPIHandle, uint8_t enOrDis){
    if(enOrDis == ENABLE)
        pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_SPE);
    else
        pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

void SPI_DeInit(SPI_RegDef_t *pSPIx){
    if(pSPIx == SPI1){
        SPI1_REG_RESET();
    } else if(pSPIx == SPI2){
        SPI2_REG_RESET();
    } else if(pSPIx == SPI3){
        SPI3_REG_RESET();
    } else if(pSPIx == SPI4){
        SPI4_REG_RESET();
    }
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
    while( Len > 0) {
        while( ! (pSPIx->SR & (1 << SPI_SR_TXE)) );
        *(volatile uint8_t *)&pSPIx->DR = *pTxBuffer;   // this approach work for setting DS to just 8 bits
        Len--;
        pTxBuffer++;
    }
}
 