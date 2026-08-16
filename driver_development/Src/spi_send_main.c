#include "gpio_driver.h"
#include "spi_driver.h"
#include <string.h>

int delay(void) {
  for (volatile uint32_t i = 0; i < 500000; i++)
    ;
}

void config_gpio_external_button(GPIO_Handle_t *pGPIOHandle) {
  pGPIOHandle->pGPIOx = GPIOB;
  pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
  pGPIOHandle->GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
  pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
  pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
  GPIO_PeriClockControl(GPIOB, ENABLE);
  GPIO_Init(pGPIOHandle);
}

int main(void) {
  GPIO_Handle_t gpioButton;
  config_gpio_external_button(&gpioButton);

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

  configGPIO.GPIO_PinNumber = GPIO_PIN_NO_6;
  configGPIO.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  gpioA.GPIO_PinConfig = configGPIO;
  GPIO_Init(&gpioA);

  configGPIO.GPIO_PinNumber = GPIO_PIN_NO_7;
  configGPIO.GPIO_PinPuPdControl = GPIO_NO_PUPD;
  gpioA.GPIO_PinConfig = configGPIO;
  GPIO_Init(&gpioA);

  configGPIO.GPIO_PinNumber = GPIO_PIN_NO_4;
  configGPIO.GPIO_PinMode = GPIO_MODE_ALTERNATE;
  configGPIO.GPIO_PinAltFunMode = 5;
  configGPIO.GPIO_PinPuPdControl = GPIO_PU;
  gpioA.GPIO_PinConfig = configGPIO;
  GPIO_Init(&gpioA);

  SPI_PeriClockControl(SPI1, ENABLE);
  SPI_Handle_t spi1;
  spi1.pSPIx = SPI1;
  SPI_Config_t configspi1;
  configspi1.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
  configspi1.SPI_BusConfig = SPI_BUS_CONFIG_FD;
  configspi1.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV128;
  configspi1.SPI_DFF = SPI_DFF_8BITS;
  configspi1.SPI_CPOL = SPI_CPOL_LOW;
  configspi1.SPI_CPHA = SPI_CPHA_HIGH;
  configspi1.SPI_SSM = SPI_SSM_DI;
  configspi1.SPI_SSI = SPI_SSI_DI;
  configspi1.SPI_SSOE = SPI_SSOE_EN;
  configspi1.SPI_FRXTH = SPI_FRXTH_EN;
  configspi1.SPI_BIDIOE = SPI_BIDIOE_DI;

  spi1.SPIConfig = configspi1;
  SPI_Init(&spi1);

  char test[] = "hello world1";
  const uint8_t howManyBytesToSend = strlen(test);

  while (1) {
    if (GPIO_ReadFromInputPin(gpioButton.pGPIOx,
                              gpioButton.GPIO_PinConfig.GPIO_PinNumber)) {
      SPI_Enable(&spi1);
      SPI_SendData(spi1.pSPIx, &howManyBytesToSend, 1);
      while (spi1.pSPIx->SR & (1 << SPI_SR_BSY)) {
      }
      SPI_SendData(spi1.pSPIx, (uint8_t *)test, howManyBytesToSend);
      while (spi1.pSPIx->SR & (1 << SPI_SR_BSY)) {
      }
      SPI_Disable(&spi1);
      delay();
    }
  }
}
