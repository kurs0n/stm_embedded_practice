#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"

#ifndef PICO_DEFAULT_SPI_SCK_PIN  
#define SCK_PIN  2  
#else  
#define SCK_PIN  PICO_DEFAULT_SPI_SCK_PIN  
#endif  
  
#ifndef PICO_DEFAULT_SPI_TX_PIN  
#define TX_PIN   3  
#else  
#define TX_PIN   PICO_DEFAULT_SPI_TX_PIN  
#endif  
  
#ifndef PICO_DEFAULT_SPI_RX_PIN  
#define RX_PIN   4  
#else  
#define RX_PIN   PICO_DEFAULT_SPI_RX_PIN  
#endif  
  
#ifndef PICO_DEFAULT_SPI_CSN_PIN  
#define CSN_PIN  5  
#else  
#define CSN_PIN  PICO_DEFAULT_SPI_CSN_PIN  
#endif  
  

int main() {
    stdio_init_all();

    // Inicjalizacja architektury Wi-Fi (wymagane dla diody w Pico W)
    if (cyw43_arch_init()) {
        printf("Błąd inicjalizacji Wi-Fi\n");
        return -1;
    }

    while (true) {
        // Włącz diodę przez układ cyw43
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        printf("LED ON\n");
        
        sleep_ms(500);

        // Wyłącz diodę
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        printf("LED OFF\n");
        
        sleep_ms(500);
    }
}
