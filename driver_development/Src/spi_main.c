#include "gpio_driver.h"
#include "spi_driver.h"
#include <string.h>

void inline __attribute__((always_inline)) delay(uint32_t delay)
{
   while(delay--) __asm("");
}

int main(void){
    GPIO_PeriClockControl(GPIOA, ENABLE);
    GPIO_Handle_t gpioA;
    gpioA.pGPIOx = GPIOA;
    GPIO_PinConfig_t configGPIO;
    configGPIO.GPIO_PinNumber = GPIO_PIN_NO_5;
    configGPIO.GPIO_PinMode = GPIO_MODE_ALTERNATE;
    configGPIO.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    configGPIO.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    configGPIO.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    configGPIO.GPIO_PinAltFunMode = 5;
    gpioA.GPIO_PinConfig = configGPIO;
    GPIO_Init(&gpioA); 

    configGPIO.GPIO_PinNumber = GPIO_PIN_NO_7;
    gpioA.GPIO_PinConfig = configGPIO;
    GPIO_Init(&gpioA); 

    SPI_PeriClockControl(SPI1,ENABLE);
    SPI_Handle_t spi1;
    spi1.pSPIx = SPI1;
    SPI_Config_t configspi1;
    configspi1.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    configspi1.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    configspi1.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256;
    configspi1.SPI_DFF = SPI_DFF_8BITS;
    configspi1.SPI_CPOL = SPI_CPOL_HIGH;
    configspi1.SPI_CPHA = SPI_CPHA_HIGH;
    configspi1.SPI_SSM = SPI_SSM_EN; 
    configspi1.SPI_SSI = SPI_SSI_EN;

    spi1.SPIConfig = configspi1; 
    SPI_Init(&spi1);
    SPI_Enable(&spi1);

    char test[] = "hello world1";
    for(int i=0; i<100000; i++){
        SPI_SendData(spi1.pSPIx,(uint8_t *)test, strlen(test));
        delay(10000);
    }
}
