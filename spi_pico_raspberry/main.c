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
        case I2C_SLAVE_RECEIVE:{ // receive from master
            uint8_t data = i2c_read_byte_raw(i2c);
            mem[reg_address++] = data;
            break;
        }
        case I2C_SLAVE_REQUEST:{ // send to master
            i2c_write_byte_raw(i2c, mem[reg_address++]);
            break;
        } 
        case I2C_SLAVE_FINISH:{
            for(int i=0; i<reg_address; i++){
                printf((char *)mem[i]);
            }
            break;
        }
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