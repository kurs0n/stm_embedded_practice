#include "gpio_driver.h"
#include "spi_driver.h"
#include "string.h"

volatile uint8_t slave_interruption = 0;

void config_gpio_interrupt_slave(GPIO_Handle_t *pGPIOHandle){
    pGPIOHandle->pGPIOx = GPIOC;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_NO_3;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinMode=GPIO_MODE_IT_RT;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_HIGH;
    pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
    GPIO_PeriClockControl(GPIOC,ENABLE);
    GPIO_Init(pGPIOHandle);
}

void config_spi_interrupt_mode(SPI_Handle_t *spiHandle){
    
}

SPI_Handle_t SPI1Handle;

int main(void){
    //config gpio interrupt
    GPIO_Handle_t gpio_interrupt_slave;
    memset(&gpio_interrupt_slave, 0, sizeof(gpio_interrupt_slave));     
    config_gpio_interrupt_slave(&gpio_interrupt_slave);
    GPIO_IRQConfig(IRQ_NO_EXTI3, 5, ENABLE);

    //configure spi interrupts
    memset(&SPI1Handle, 0, sizeof(SPI1Handle));


    while(1){
        while(slave_interruption){
            __asm("nop");
            slave_interruption = 0;
        }
    }
}


void EXTI3_IRQHandler(void){ 
    slave_interruption = 1;
    GPIO_IRQHandling(GPIO_PIN_NO_3);
}

void SPI1_IRQHandler(void){
    // handle
}