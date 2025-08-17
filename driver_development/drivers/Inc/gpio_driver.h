#ifndef INC_GPIO_DRIVER_H
#define INC_GPIO_DRIVER_H

#include "STM32F303REx.h"

typedef struct
{
    uint8_t GPIO_PinNumber;
    uint8_t GPIO_PinMode; // possible values from here: @GPIO_PIN_MODES
    uint8_t GPIO_PinSpeed;
    uint8_t GPIO_PinPuPdControl;
    uint8_t GPIO_PinOPType;
    uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;

typedef struct
{
    GPIO_RegDef_t *pGPIOx;
    GPIO_PinConfig_t GPIO_PinConfig;
} GPIO_Handle_t;

/*
@GPIO_PIN_MODES
GPIO PIN possible modes
*/

#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTERNATE 2
#define GPIO_MODE_ANALOG 3

/* GPIO PIN possible output types */
#define GPIO_OP_TYPE_PP 0 // push-pull
#define GPIO_OP_TYPE_OD 1 // open-drain

/* GPIO PIN possible output speed */

#define GPIO_SPEED_LOW 0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_HIGH 3

/* GPIO PIN pull-up pull-down configuration macros */

#define GPIO_NO_PUPD 0
#define GPIO_PU 1
#define GPIO_PD 2

/* GPIO PIN numbers */
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15

void GPIO_PeriClockControl(GPIO_RegDef_t *config, uint8_t EnableOrDisable); // Peripherial clock setup

void GPIO_Init(GPIO_Handle_t *pGPIOHandle); // init and deinitialization
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// read and write data

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value); // 0 or 1 value
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ ISR Handling

void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis);
void GPIO_IRQHandling(uint8_t PinNumber);

// generic macros

#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

#endif