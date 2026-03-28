#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>
#define CMD_ID_READ 5
#define CMD_PRINT 4
#define CMD_LED_READ 3
#define CMD_SENSOR_READ 2
#define CMD_LED_CTRL 1  
#define ACK 0xF5 
#define NACK 0xA5 

int main() {
  stdio_init_all();

  spi_init(spi0, 1000); // baudrate ignored in slave mode
  spi_set_slave(spi0, true);
  spi_set_format(
      spi0, 8, SPI_CPOL_0, SPI_CPHA_1,
      SPI_MSB_FIRST); // Lesson learned! CPHA_1 is more stable than CPHA_0 for
                      // some reason CPHA_0 is getting only the first byte

  gpio_set_function(2, GPIO_FUNC_SPI);
  gpio_set_function(3, GPIO_FUNC_SPI);
  gpio_set_function(4, GPIO_FUNC_SPI);
  gpio_set_function(5, GPIO_FUNC_SPI);

  while (true) {
    uint8_t command = 0x0;
    uint8_t ackToSend = ACK;
    spi_read_blocking(spi0, 0x00, &command, 1);

    switch(command){
      case CMD_LED_CTRL: {
        spi_write_blocking(spi0, &ackToSend, 1);
        uint8_t response = 0xff;
        uint8_t commandArgs[] = {0x0, 0x0}; 
        spi_read_blocking(spi0, 0x00, commandArgs, 2); 
        if(commandArgs[0] && commandArgs[1]){
          spi_write_blocking(spi0, &response, 1);
        } 
        break;
      }
      case CMD_SENSOR_READ: {
        spi_write_blocking(spi0, &ackToSend, 1);
        uint8_t response = 0x12;
        uint8_t commandArg = 0x00;
        spi_read_blocking(spi0, 0x00, &commandArg, 1);
        if(commandArg){
          spi_write_blocking(spi0, &response, 1);
        }
        break;
      }
      case CMD_LED_READ: {
        spi_write_blocking(spi0, &ackToSend, 1);
        uint8_t ledResponse = 0x32;
        uint8_t commandArg = 0x00;
        spi_read_blocking(spi0, 0x00, &commandArg, 1);
        if(commandArg){
          spi_write_blocking(spi0, &ledResponse, 1);
        }
        break;
      }
      case CMD_PRINT: {
        spi_write_blocking(spi0, &ackToSend, 1);
        uint8_t messageLen = 0x00;
        char* message;
        uint8_t printAck = 0x10;
        spi_read_blocking(spi0, 0x00, &messageLen, 1);
        if(messageLen){
          spi_read_blocking(spi0, 0x00, (uint8_t *)message, messageLen);
          spi_write_blocking(spi0, &printAck, 1);
        }
        break;
      }
      case CMD_ID_READ: {
        spi_write_blocking(spi0, &ackToSend, 1);
        uint8_t responseData = 0x1;
        spi_write_blocking(spi0, &responseData,1); 
        break;
      }
    }
  }
}