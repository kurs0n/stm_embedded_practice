#ifndef STM32F303REx_H
#define STM32F303REx_HL

#include <stdint.h>

#define _vo volatile

#define FLASH_BASEADDR 0x08000000U
#define SRAM_BASEADDR 0x20000000UL
#define ROM_BASEADDR 0x1FFFD800UL


/* Peripherial buses adresses */

#define PERIPH_BASE 0x40000000UL
#define APB1PERIPH_BASE PERIPH_BASE
#define APB2PERIPH_BASE 0x40010000UL
#define AHB1PERIPH_BASE 0x40020000UL
#define AHB2PERIPH_BASE 0x48000000UL
#define AHB3PERIPH_BASE 0x50000000UL

#define SRAM SRAM_BASEADDR 

/* Peripherial buses addresses end */

/* Base addresses of peripherials which are hanging on AHB3 BUS */

#define ADC1_ADC2_BASEADDR AHB3PERIPH_BASE
#define ADC3_ADC4_BASEADDR (AHB3PERIPH_BASE + 0x0400UL)

/* Base addresses of peripherials which are hanging on AHB3 BUS end */

/* Base addresses of peripherials which are hanging on AHB2 BUS */

#define GPIOA_BASEADDR (AHB2PERIPH_BASE + 0x0000UL)
#define GPIOB_BASEADDR (AHB2PERIPH_BASE + 0x0400UL)
#define GPIOC_BASEADDR (AHB2PERIPH_BASE + 0x0800UL)
#define GPIOD_BASEADDR (AHB2PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASEADDR (AHB2PERIPH_BASE + 0x1000UL)
#define GPIOF_BASEADDR (AHB2PERIPH_BASE + 0x1400UL)

/* Base addresses of peripherials which are hanging on AHB2 BUS end*/


/* Base addresses of peripherials which are hanging on AHB1 BUS */

#define DMA1_BASEADDR AHB1PERIPH_BASE
#define DMA2_BASEADDR (AHB1PERIPH_BASE + 0x0400UL)
#define RCC_BASEADDR (AHB1PERIPH_BASE + 0x1000UL)
#define FLASHIF_BASEADDR (AHB1PERIPH_BASE + 0x2000UL)
#define CRC_BASEADDR (AHB1PERIPH_BASE + 0x3000UL)
#define TSC_BASEADDR (AHB1PERIPH_BASE + 0x4000UL)

/* Base addresses of peripherials which are hanging on AHB1 BUS end*/


/* Base addresses of peripherials which are hanging on APB1 BUS */

#define TIM2_BASEADDR APB1PERIPH_BASE
#define TIM3_BASEADDR (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASEADDR (APB1PERIPH_BASE + 0x0800UL)
#define TIM6_BASEADDR (APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASEADDR (APB1PERIPH_BASE + 0x1400UL)
#define RTC_BASEADDR (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASEADDR (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASEADDR (APB1PERIPH_BASE + 0x3000UL)
#define I2S2EXT_BASEADDR (APB1PERIPH_BASE + 0x3400UL)
#define SPI2_I2S2_BASEADDR (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_I2S3_BASEADDR (APB1PERIPH_BASE + 0x3C00UL)
#define I2S3EXT_BASEADDR (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASEADDR (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASEADDR (APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASEADDR (APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASEADDR (APB1PERIPH_BASE + 0x5000UL)
#define I2C1_BASEADDR (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASEADDR (APB1PERIPH_BASE + 0x5800UL)
#define USBDEVICEFS_BASEADDR (APB1PERIPH_BASE + 0x5C00UL)
#define USBSRAM15BYTES_BASEADDR (APB1PERIPH_BASE + 0x6000UL)
#define BXCAN_BASEADDR (APB1PERIPH_BASE + 0x6400UL)
#define PWR_BASEADDR (APB1PERIPH_BASE + 0x7000UL)
#define DAC1_BASEADDR (APB1PERIPH_BASE + 0x7400UL)

/* Base addresses of peripherials which are hanging on APB1 BUS end */


/* Base addresses of peripherials which are hanging on APB2 BUS */

#define SYSCFG_COMP_OPAMP_BASEADDR APB2PERIPH_BASE
#define EXTI_BASEADDR (APB2PERIPH_BASE + 0x0400UL)
#define TIM1_BASEADDR (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASEADDR (APB2PERIPH_BASE + 0x3000UL)
#define TIM8_BASEADDR (APB2PERIPH_BASE + 0x3400UL)
#define USART1_BASEADDR (APB2PERIPH_BASE + 0x3800UL)
#define TIM15_BASEADDR (APB2PERIPH_BASE + 0x4000UL)
#define TIM16_BASEADDR (APB2PERIPH_BASE + 0x4400UL)
#define TIM17_BASEADDR (APB2PERIPH_BASE + 0x4800UL)

/* Base addresses of peripherials which are hanging on APB2 BUS end */



// Peripheral register definition structures

typedef struct
{
    _vo uint32_t MODER;      // GPIO port mode register
    _vo uint32_t OTYPER;
    _vo uint32_t OSPEEDR;   // GPIO port output speed register
    _vo uint32_t PUPDR;     // GPIO port pull-up/pull-down register
    _vo uint32_t IDR;       // GPIO port input data register
    _vo uint32_t ODR;       // GPIO port output data register
    _vo uint32_t BSRR;      // GPIO port bit set/reset register
    _vo uint32_t LCKR;      // GPIO port configuration lock register
    _vo uint32_t AFR[2];    // GPIO alternate function registers (AFR[0] and AFR[1])
    _vo uint32_t BRR;    // GPIO port bit reset register
} GPIO_RegDef_t;

typedef struct{
    _vo uint32_t CR; // RCC clock control register
    _vo uint32_t CFGR; // RCC clock configuration register
    _vo uint32_t CIR; // RCC clock interrupt register
    _vo uint32_t APB2RSTR;  // RCC APB2 peripheral reset register
    _vo uint32_t APB1RSTR;  // RCC APB1 peripheral reset register
    _vo uint32_t AHBENR;  // RCC AHB peripheral clock enable register
    _vo uint32_t APB2ENR;  // RCC APB2 peripheral clock enable register
    _vo uint32_t APB1ENR;  // RCC APB1 peripheral clock enable register
    _vo uint32_t BDCR;  // RCC Backup domain control register
    _vo uint32_t CSR;  // RCC clock control and status register
    _vo uint32_t AHBRSTR;  // RCC AHB peripheral reset register
    _vo uint32_t CFGR2;  // RCC clock configuration register 2
    _vo uint32_t CFGR3;  // RCC clock configuration register 3
} RCC_RegDef_t;

/* Peripherial Definitions */

#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t *)GPIOF_BASEADDR)

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

/* Reset GPIOx peripherials */

#define GPIOA_REG_RESET() do{GPIOA->BSRR |= (0xFFFF << 16); GPIOA->BSRR &= ~(0xFFFF << 16);}while(0)
#define GPIOB_REG_RESET() do{GPIOB->BSRR |= (0xFFFF << 16); GPIOB->BSRR &= ~(0xFFFF << 16);}while(0)
#define GPIOC_REG_RESET() do{GPIOC->BSRR |= (0xFFFF << 16); GPIOC->BSRR &= ~(0xFFFF << 16);}while(0)
#define GPIOD_REG_RESET() do{GPIOD->BSRR |= (0xFFFF << 16); GPIOD->BSRR &= ~(0xFFFF << 16);}while(0)
#define GPIOE_REG_RESET() do{GPIOE->BSRR |= (0xFFFF << 16); GPIOE->BSRR &= ~(0xFFFF << 16);}while(0)
#define GPIOF_REG_RESET() do{GPIOF->BSRR |= (0xFFFF << 16); GPIOF->BSRR &= ~(0xFFFF << 16);}while(0)

/* Reset GPIOx peripherials end */

/* Clock Enable Macros for GPIOx peripherials */

#define GPIOA_PERI_CLOCK_ENABLE()   ( RCC->AHBENR |= (1<<17))
#define GPIOB_PERI_CLOCK_ENABLE()   ( RCC->AHBENR |= (1<<18))
#define GPIOC_PERI_CLOCK_ENABLE()   ( RCC->AHBENR |= (1<<19))
#define GPIOD_PERI_CLOCK_ENABLE()   ( RCC->AHBENR |= (1<<20))
#define GPIOE_PERI_CLOCK_ENABLE()   ( RCC->AHBENR |= (1<<21))
#define GPIOF_PERI_CLOCK_ENABLE()   ( RCC->AHBENR |= (1<<22))

/* Clock Enable Macros for GPIOx peripherials end */

/* Clock Enable Macros for I2Cx peripherials */

#define I2C1_PCLK_EN()  (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()  (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()  (RCC->APB1ENR |= (1<<30))

/* Clock Enable Macros for I2Cx peripherials end */

/* Clock Enable Macros for SPIx peripherials */

#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1<<15))

/* Clock Enable Macros for SPIx peripherials end */


/* Clock Enable Macros for USARTx peripherials */

#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<14))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1<<18))
#define UART4_PCLK_EN() (RCC->APB1ENR |= (1<<19))
#define UART5_PCLK_EN() (RCC->APB1ENR |= (1<<20))

/* Clock Enable Macros for USARTx peripherials end*/


/* Clock Enable Macros for SYSCFG peripherial */

#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= 1)

/* Clock Enable Macros for SYSCFG peripherial end*/

/* Clock Disable Macros for GPIOx peripherials */

#define GPIOA_PERI_CLOCK_DISABLE()   ( RCC->AHBENR &= ~(1<<17))
#define GPIOB_PERI_CLOCK_DISABLE()   ( RCC->AHBENR &= ~(1<<18))
#define GPIOC_PERI_CLOCK_DISABLE()   ( RCC->AHBENR &= ~(1<<19))
#define GPIOD_PERI_CLOCK_DISABLE()   ( RCC->AHBENR &= ~(1<<20))
#define GPIOE_PERI_CLOCK_DISABLE()   ( RCC->AHBENR &= ~(1<<21))
#define GPIOF_PERI_CLOCK_DISABLE()   ( RCC->AHBENR &= ~(1<<22))

/* Clock Disable Macros for GPIOx peripherials end */

/* Clock Disable Macros for I2Cx peripherials */

#define I2C1_PCLK_DI()  (RCC->APB1ENR &= ~(1<<21))
#define I2C2_PCLK_DI()  (RCC->APB1ENR &= ~(1<<22))
#define I2C3_PCLK_DI()  (RCC->APB1ENR &= ~(1<<30))

/* Clock Disable Macros for I2Cx peripherials end */

/* Clock Disable Macros for SPIx peripherials */

#define SPI1_PCLK_DI() (RCC->APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= ~(1<<15))

/* Clock Disable Macros for SPIx peripherials end */


/* Clock Disable Macros for USARTx peripherials */

#define USART1_PCLK_DI() (RCC->APB2ENR &= ~(1<<14))
#define USART2_PCLK_DI() (RCC->APB1ENR &= ~(1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= ~(1<<18))
#define UART4_PCLK_DI()  (RCC->APB1ENR &= ~(1<<19))
#define UART5_PCLK_DI()  (RCC->APB1ENR &= ~(1<<20))

/* Clock Disable Macros for USARTx peripherials end*/


/* Clock Disable Macros for SYSCFG peripherial */

#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= ~(1))

/* Clock Disable Macros for SYSCFG peripherial end*/


#endif
