#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

int main() {
    stdio_init_all();

    spi_init(spi0, 1000); // baudrate ignored in slave mode 
    spi_set_slave(spi0, true);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // Lesson learned! CPHA_1 is more stable than CPHA_0 for some reason CPHA_0 is getting only the first byte

    gpio_set_function(2, GPIO_FUNC_SPI); 
    gpio_set_function(4, GPIO_FUNC_SPI); 
    gpio_set_function(5, GPIO_FUNC_SPI); 
   
    uint8_t buf[12];

    while (true) {
        spi_read_blocking(spi0, 0x00, buf, sizeof(buf));

        for (int i = 0; i < sizeof(buf); i++) {
            printf("%02X ", buf[i]);
        }
        printf("\n");
    }
}