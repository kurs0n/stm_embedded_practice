#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#define CMD_ID_READ 5
#define CMD_PRINT 4
#define CMD_LED_READ 3
#define CMD_SENSOR_READ 2
#define CMD_LED_CTRL 1  
#define ACK 0xF5 
#define NACK 0xA5 

int main(){
   stdio_init_all();
   const uint GPIO_OUT_PIN = 22;

  gpio_init(GPIO_OUT_PIN);
  gpio_set_dir(GPIO_OUT_PIN, GPIO_OUT);

  spi_init(spi0, 1000); // baudrate ignored in slave mode
  spi_set_slave(spi0, true);
  spi_set_format(
      spi0, 8, SPI_CPOL_0, SPI_CPHA_1,
      SPI_MSB_FIRST);
  gpio_set_function(2, GPIO_FUNC_SPI);
  gpio_set_function(3, GPIO_FUNC_SPI);
  gpio_set_function(4, GPIO_FUNC_SPI);
  gpio_set_function(5, GPIO_FUNC_SPI);  

  const char test_data[] = "hello worlddd";
  const int len = strlen(test_data);
  int i = 0;
  while(i<len){
    gpio_put(GPIO_OUT_PIN, 1);
    sleep_ms(500);// not sure if this needs to be there.
    
    spi_write_blocking(spi0, (uint8_t *)test_data[i], 1);

    gpio_put(GPIO_OUT_PIN, 0);
    sleep_ms(500);
    i++;
  }
 
}