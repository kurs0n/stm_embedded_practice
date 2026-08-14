#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/i2c_slave.h"
#include <stdio.h>
#include <string.h>

#define I2C_SLAVE_ADDRESS 0x17
#define PIN_SDA 0
#define PIN_SCL 1

static uint8_t reg_address = 0;
static uint8_t mem[256] = {0};

static void i2c_slave_handler(i2c_inst_t *i2c,i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: // Master is writing to the slave
            break;
        case I2C_SLAVE_REQUEST: // Master wants a data from the slave
            break;
        case I2C_SLAVE_FINISH:
            break;
        default:
            break;
    }
}

int main(){
    stdio_init_all();
    gpio_pull_up(PIN_SDA);
    gpio_pull_up(PIN_SCL);
    gpio_set_function(PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(PIN_SCL, GPIO_FUNC_I2C);

    i2c_init(i2c0, 100 * 1000); // normal mode 100kHz


    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
    while(1){ 
        tight_loop_contents();
    }
}