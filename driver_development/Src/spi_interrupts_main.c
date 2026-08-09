#include "gpio_driver.h"
#include "spi_driver.h"
#include "string.h"
#include <string.h>

volatile uint8_t slave_interruption = 0;

void config_gpio_interrupt_slave(GPIO_Handle_t *pGPIOHandle){
    pGPIOHandle->pGPIOx = GPIOC;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_3;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_RT; // gpio mode interrupt for rising trigger
    pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;

    GPIO_PeriClockControl(GPIOC,ENABLE);
    GPIO_Init(pGPIOHandle);
}

void config_gpio_into_spi(GPIO_Handle_t *gpioHandle, GPIO_PinConfig_t *configGPIO){
    gpioHandle->pGPIOx = GPIOA;
    gpioHandle->GPIO_PinConfig = *configGPIO;
    GPIO_Init(gpioHandle);
}

void config_spi(SPI_Handle_t *spiHandle){
    spiHandle->pSPIx = SPI1;
    SPI_Config_t config_spi1;
    config_spi1.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
    config_spi1.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    config_spi1.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV128;
    config_spi1.SPI_DFF = SPI_DFF_8BITS;
    config_spi1.SPI_CPOL = SPI_CPOL_LOW;
    config_spi1.SPI_CPHA = SPI_CPHA_HIGH;
    config_spi1.SPI_SSM = SPI_SSM_EN;
    config_spi1.SPI_SSI = SPI_SSI_EN;
    config_spi1.SPI_SSOE = SPI_SSOE_EN;
    config_spi1.SPI_FRXTH = SPI_FRXTH_EN;
    config_spi1.SPI_BIDIOE = SPI_BIDIOE_DI;
    spiHandle->SPIConfig = config_spi1;
    SPI_Init(spiHandle);
}

uint8_t dummyByte = 0xff;
SPI_Handle_t SPI1Handle;
GPIO_Handle_t gpioA;

int main(void){
    //config gpio interrupt
    char buffer[255] = "";
    GPIO_Handle_t gpio_interrupt_slave;
    memset(&gpio_interrupt_slave, 0, sizeof(gpio_interrupt_slave));     
    config_gpio_interrupt_slave(&gpio_interrupt_slave);
    GPIO_IRQConfig(IRQ_NO_EXTI3, 5, ENABLE);

    GPIO_PeriClockControl(GPIOA, ENABLE);

    // configure GPIO as a SPI
    GPIO_PinConfig_t configGPIO;
    configGPIO.GPIO_PinNumber = GPIO_PIN_NO_5;
    configGPIO.GPIO_PinMode = GPIO_MODE_ALTERNATE;
    configGPIO.GPIO_PinSpeed = GPIO_SPEED_HIGH;
    configGPIO.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    configGPIO.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    configGPIO.GPIO_PinAltFunMode = 5;
    config_gpio_into_spi(&gpioA, &configGPIO);
    
    configGPIO.GPIO_PinNumber = GPIO_PIN_NO_6;
    configGPIO.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioA.GPIO_PinConfig = configGPIO;
    config_gpio_into_spi(&gpioA, &configGPIO);

    configGPIO.GPIO_PinNumber = GPIO_PIN_NO_7;
    configGPIO.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    gpioA.GPIO_PinConfig = configGPIO;
    config_gpio_into_spi(&gpioA, &configGPIO);

    // RASPBERRY PI PICO NEEDS TO HAVE CSS pin configured. It can be done by regular GPIO pin.
    GPIO_Handle_t slave_CSS_pin; // PC5
    memset(&slave_CSS_pin, 0, sizeof(slave_CSS_pin));
    slave_CSS_pin.pGPIOx = GPIOC;
    slave_CSS_pin.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_5;
    slave_CSS_pin.GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_OUTPUT;
    slave_CSS_pin.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
    slave_CSS_pin.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
    slave_CSS_pin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    GPIO_Init(&slave_CSS_pin);
    GPIO_WriteToOutputPin(slave_CSS_pin.pGPIOx, GPIO_PIN_NO_5, 1);
    //configure spi interrupts
    
    SPI_PeriClockControl(SPI1, ENABLE);
    memset(&SPI1Handle, 0, sizeof(SPI1Handle));
    config_spi(&SPI1Handle);
    
    // nvic setup
    GPIO_IRQConfig(IRQ_NO_SPI1, 5, ENABLE);
    // end nvic setup

    SPI_Enable(&SPI1Handle);

    int i=0;

    while(1){
        while(slave_interruption){
            char byte = 0x00;    
            GPIO_WriteToOutputPin(slave_CSS_pin.pGPIOx, GPIO_PIN_NO_5, 0);
            while( SPI_SendDataIT(&SPI1Handle, &dummyByte, 1) == SPI_BUSY_IN_TX);
            while( SPI_ReceiveDataIT(&SPI1Handle, (uint8_t *)&byte, 1) == SPI_BUSY_IN_RX);
            GPIO_WriteToOutputPin(slave_CSS_pin.pGPIOx, GPIO_PIN_NO_5, 1);
            buffer[i] = byte;
            i++;
            slave_interruption = 0;
        }
    }
}


void EXTI3_IRQHandler(void){ 
    slave_interruption = 1;
    GPIO_IRQHandling(GPIO_PIN_NO_3);
}

void SPI1_IRQHandler(void){
    SPI_IRQHandling(&SPI1Handle);
}