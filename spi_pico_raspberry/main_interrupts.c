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

int main(){
    
}