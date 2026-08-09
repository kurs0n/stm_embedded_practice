#include "spi_driver.h" 
void inline __attribute__((always_inline)) delay(uint32_t delay) {
  while (delay--)
    __asm("");
}

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle);

static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle);

static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle);

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
  switch (EnorDi) {
  case ENABLE: {
    if (pSPIx == SPI1) {
      SPI1_PCLK_EN();
    } else if (pSPIx == SPI2) {
      SPI2_PCLK_EN();
    } else if (pSPIx == SPI3) {
      SPI3_PCLK_EN();
    } else if (pSPIx == SPI4) {
      SPI4_PCLK_EN();
    }
    break;
  }
  case DISABLE: {
    if (pSPIx == SPI1) {
      SPI1_PCLK_DI();
    } else if (pSPIx == SPI2) {
      SPI2_PCLK_DI();
    } else if (pSPIx == SPI3) {
      SPI3_PCLK_DI();
    } else if (pSPIx == SPI4) {
      SPI4_PCLK_DI();
    }
    break;
  }
  }
}

void SPI_Init(SPI_Handle_t *pSPIHandle) {
  uint32_t control_register1 = 0;
  uint32_t control_register2 = 0;

  pSPIHandle->pSPIx->CR1 = 0;
  pSPIHandle->pSPIx->CR2 = 0;

  pSPIHandle->pSPIx->CR1 &= ~(ENABLE << SPI_CR1_SPE);

  control_register1 |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

  if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD) {
    control_register1 &= ~(ENABLE << SPI_CR1_BIDIMODE);
  } else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD) {
    control_register1 |= ENABLE << SPI_CR1_BIDIMODE;
  } else if (pSPIHandle->SPIConfig.SPI_BusConfig ==
             SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
    control_register1 &= ~(ENABLE << SPI_CR1_BIDIMODE);
    control_register1 |= ENABLE << SPI_CR1_RXONLY;
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

void SPI_Disable(SPI_Handle_t *pSPIHandle){
  pSPIHandle->pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

void SPI_Enable(SPI_Handle_t *pSPIHandle) {
  pSPIHandle->pSPIx->CR1 |= (1 << SPI_CR1_SPE);
}

void SPI_DeInit(SPI_RegDef_t *pSPIx) {
  if (pSPIx == SPI1) {
    SPI1_REG_RESET();
  } else if (pSPIx == SPI2) {
    SPI2_REG_RESET();
  } else if (pSPIx == SPI3) {
    SPI3_REG_RESET();
  } else if (pSPIx == SPI4) {
    SPI4_REG_RESET();
  }
}

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len) {
  while (Len > 0) {
    while (!(pSPIx->SR & (1 << SPI_SR_TXE)));
    *(volatile uint8_t *)&pSPIx->DR =
        *pTxBuffer; // this approach work for setting DS to just 8 bits
    while (!(pSPIx->SR & (1 << SPI_SR_RXNE)))
      ;
    volatile uint8_t temp = *(volatile uint8_t *)&pSPIx->DR;
    (void)temp;

    Len--;
    pTxBuffer++;
  }
  while (pSPIx->SR & (1 << SPI_SR_BSY)) {
  }
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len) {
  for (int i = 0; i < Len ; i++) {
    while (!(pSPIx->SR & (1 << SPI_SR_TXE)))
      ;
    *(volatile uint8_t *)&pSPIx->DR = 0xff;
    while (!(pSPIx->SR & (1 << SPI_SR_RXNE)));
    uint8_t data = *(volatile uint8_t *)&pSPIx->DR;
    *(pRxBuffer + i) = data;
  }
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len) {
  uint8_t state = pSPIHandle->TxState;

  if(state != SPI_BUSY_IN_TX){
    // save tx buffer in global variable
    pSPIHandle->pTxBuffer = pTxBuffer;
    pSPIHandle->TxLen = Len; 
    
    // save state in global variable
    pSPIHandle->TxState = SPI_BUSY_IN_TX;
  
    // enable the txeie control bit in order to generate a interrupt when TXE flag is set
    pSPIHandle->pSPIx->CR2 |= (ENABLE << SPI_CR2_TXEIE);
  }
  
  return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len) {
  uint8_t state = pSPIHandle->RxState;
  
  if(state != SPI_BUSY_IN_RX){

    // save rx buffer in global variable
    pSPIHandle->pRxBuffer = pRxBuffer;
    pSPIHandle->RxLen = Len;

    // save state in global variable
    pSPIHandle->RxState = SPI_BUSY_IN_RX;
    
    // enable rxneie control bit in order to generate a interrupt when RXNE flag is set
    pSPIHandle->pSPIx->CR2 |= (ENABLE << SPI_CR2_RXNEIE); 
  }

  return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
  uint8_t temp1, temp2;

  // check for TXE
  temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
  temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
  if (temp1 && temp2){
    spi_txe_interrupt_handler(pSPIHandle);
    // handler of TXE
  }
  
  // check for RXNE
  temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
  temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
  if (temp1 && temp2){
    spi_rxne_interrupt_handler(pSPIHandle);
    // handler for RXNE
  }

  // check for overrun flag
  temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
  temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
  if(temp1 && temp2){
    spi_ovr_interrupt_handler(pSPIHandle);
    // handler for ERROR
  }
}

static void spi_txe_interrupt_handler(SPI_Handle_t *pSPIHandle){
  // send data
  *(volatile uint8_t *)&pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
  pSPIHandle->TxLen--;
  pSPIHandle->pTxBuffer++;


  if( !pSPIHandle->TxLen ){
   // if txLen is 0
   // TX is over
   // prevent interrupts from setting up txe flag
   SPI_CloseTransmission(pSPIHandle); 
   // callback?
  }
}

static void spi_rxne_interrupt_handler(SPI_Handle_t *pSPIHandle){
  *pSPIHandle->pRxBuffer = *(volatile uint8_t *)&pSPIHandle->pSPIx->DR;
  pSPIHandle->RxLen--;
  pSPIHandle->pRxBuffer++;
  
 if( !pSPIHandle->RxLen ){
  SPI_CloseReception(pSPIHandle);
  // callback
 } 
}

static void spi_ovr_interrupt_handler(SPI_Handle_t *pSPIHandle){
  SPI_ClearOVRFlag(pSPIHandle);

  // inform application by callback 
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
  pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
  pSPIHandle->pTxBuffer = NULL;
  pSPIHandle->TxLen = 0;
  pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
  pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
  pSPIHandle->pRxBuffer = NULL; 
  pSPIHandle->RxLen = 0;
  pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle){
  uint8_t temp;
  if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
    temp = pSPIHandle->pSPIx->DR;
    temp = pSPIHandle->pSPIx->SR;
  }
  (void)temp;
}


__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
  
}
