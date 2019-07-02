#!/bin/bash

gpio_high() {
    if ! (( $# == 1 )); then
        return 1
    fi
    echo $1 > /sys/class/gpio/export
    echo out > /sys/class/gpio/gpio${1}/direction
    echo 0 > /sys/class/gpio/gpio${1}/value
}

# map devices to SSC / SLPI
gpio_high 34  # EXT_IMU_I2C_MUX_SEL
gpio_high 73  # SONAR_UART_MUX_SEL
gpio_high 77  # IMU1_SPI_MUX_SEL
#gpio_high 97  # EAGLENEST_UART_MUX_SEL
gpio_high 110 # ESC_UART_MUX_SEL
gpio_high 115 # BAROM_I2C_MUX_SEL
