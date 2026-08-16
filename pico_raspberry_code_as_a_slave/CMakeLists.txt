cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_spi_driver_test)

pico_sdk_init()


add_executable(main
    main.c
)
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

target_link_libraries(main pico_stdlib pico_cyw43_arch_none hardware_spi hardware_gpio hardware_i2c pico_i2c_slave)

pico_add_extra_outputs(main)