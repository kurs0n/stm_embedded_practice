#include "spi_driver.h"

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
