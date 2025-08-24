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
        break;
    }
    }
}

// usually use OR instead of = in embedded programming due to sometimes we dont want to touch registers which have some values reserved
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
    uint32_t temp = 0;

    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
    {
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // correct offset calculated
        pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));              // clearing
        pGPIOHandle->pGPIOx->MODER |= temp;                                                                    // OR to not touch other places in register setting stuff
    }
    else
    {
        switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
        {
        case GPIO_MODE_IT_FT:
        {
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            break;
        }

        case GPIO_MODE_IT_RT:
        {
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            break;
        }

        case GPIO_MODE_IT_RFT:
        {
            EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
            break;
        }
        }

        // Syscfg configuration to route the GPIO port to the EXTI line
        SYSCFG_PCLK_EN();

        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4; 
        SYSCFG->EXTICR[temp1] |= (GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx) << (temp2 * 4));

        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    }

    // configure speed for GPIO pin
    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
    pGPIOHandle->pGPIOx->OSPEEDR |= temp;

    // configure pull up pull down settings

    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing
    pGPIOHandle->pGPIOx->PUPDR |= temp;

    // configure output type

    temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
    pGPIOHandle->pGPIOx->OTYPER |= temp;

    // configure alternate function stuff.
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTERNATE)
    {
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8)
        {
            temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
            pGPIOHandle->pGPIOx->AFR[0] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits
            pGPIOHandle->pGPIOx->AFR[0] |= temp;
        }
        else
        {
            temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8)));
            pGPIOHandle->pGPIOx->AFR[1] &= ~(0xF << (4 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // clearing bits
            pGPIOHandle->pGPIOx->AFR[1] |= temp;
        }
    }
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
    if (pGPIOx == GPIOA)
    {
        GPIOA_REG_RESET();
    }
    else if (
        pGPIOx == GPIOB)
    {
        GPIOB_REG_RESET();
    }
    else if (pGPIOx == GPIOC)
    {
        GPIOC_REG_RESET();
    }
    else if (pGPIOx == GPIOD)
    {
        GPIOD_REG_RESET();
    }
    else if (
        pGPIOx == GPIOE)
    {
        GPIOE_REG_RESET();
    }
    else if (
        pGPIOx == GPIOF)
    {
        GPIOF_REG_RESET();
    }
}

// //read and write data

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    uint8_t val = 0;
    val = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return val;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
    uint16_t val = 0;
    val = (uint16_t)pGPIOx->IDR;
    return val;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
    if (Value == GPIO_PIN_SET)
    {
        pGPIOx->ODR |= (Value << PinNumber);
    }
    else
    {
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
    pGPIOx->ODR &= ~(0xFFFF);
    pGPIOx->ODR |= (Value & 0xFFFF);
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

// //IRQ ISR Handling

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprxSection = IRQNumber % 4;
    uint8_t shift_amount =  (8 * iprxSection) + (8 - NO_PRIORITY_BITS_IMPLEMENTED);

    *((volatile uint32_t *)(NVIC_PR_BASE_ADDR + (iprx * 4))) |= (IRQPriority << shift_amount);
}

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis){
    if (EnOrDis == ENABLE){
        if (IRQNumber <= 32 ){  
            *NVIC_ISER0 |= (1 << IRQNumber);
        } else if(IRQNumber >= 32 && IRQNumber <= 64) {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32 ));
        } else if (IRQNumber > 64 ){
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    } else {
          if (IRQNumber <= 32 ){  
            *NVIC_ICER0 |= (1 << IRQNumber);
        } else if(IRQNumber >= 32 && IRQNumber <= 64) {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32 ));
        } else if (IRQNumber > 64 ){
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
        }
    } 
    // GPIO_IRQPriorityConfig(IRQNumber,IRQPriority);
}

void GPIO_IRQHandling(uint8_t PinNumber){
    if (EXTI->PR&(1 <<PinNumber)){
        EXTI->PR |= (1<< PinNumber);
    }
}