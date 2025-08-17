#include "gpio_driver.h"

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnableOrDisable)
{
    switch (EnableOrDisable)
    {
        case ENABLE:
        {
            if (pGPIOx == GPIOA)
            {
                GPIOA_PERI_CLOCK_ENABLE();
            }
            else if (
                pGPIOx == GPIOB)
            {
                GPIOB_PERI_CLOCK_ENABLE();
            }
            else if (pGPIOx == GPIOC)
            {
                GPIOC_PERI_CLOCK_ENABLE();
            }
            else if (pGPIOx == GPIOD)
            {
                GPIOD_PERI_CLOCK_ENABLE();
            }
            else if (
                pGPIOx == GPIOE)
            {
                GPIOE_PERI_CLOCK_ENABLE();
            }
            else if (
                pGPIOx == GPIOF)
            {
                GPIOF_PERI_CLOCK_ENABLE();
            }
            else if (
                pGPIOx == GPIOG)
            {
                GPIOG_PERI_CLOCK_ENABLE();
            }
            break;
        }
        case DISABLE:
        {
            if (pGPIOx == GPIOA)
            {
                GPIOA_PERI_CLOCK_DISABLE();
            }
            else if (
                pGPIOx == GPIOB)
            {
                GPIOB_PERI_CLOCK_DISABLE();
            }
            else if (pGPIOx == GPIOC)
            {
                GPIOC_PERI_CLOCK_DISABLE();
            }
            else if (pGPIOx == GPIOD)
            {
                GPIOD_PERI_CLOCK_DISABLE();
            }
            else if (
                pGPIOx == GPIOE)
            {
                GPIOE_PERI_CLOCK_DISABLE();
            }
            else if (
                pGPIOx == GPIOF)
            {
                GPIOF_PERI_CLOCK_DISABLE();
            }
            else if (
                pGPIOx == GPIOG)
            {
                GPIOG_PERI_CLOCK_DISABLE();
            }
            break;
        }
    }
}

// void GPIO_Init(GPIO_Handle_t *pGPIOHandle); // init and deinitialization
// void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// //read and write data

// uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
// uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
// void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value); // 0 or 1 value
// void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
// void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// //IRQ ISR Handling

// void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
// void GPIO_IRQHandling(uint8_t PinNumber);