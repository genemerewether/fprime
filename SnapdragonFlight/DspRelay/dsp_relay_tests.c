/*
 * dsp_relay_tests.c
 *
 *  Created on: Oct 21, 2016
 *      Author: tcanham
 */

#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "dsp_relay_local.h"
#include "gpio_defs.h"

#include <dev_fs_lib_serial.h>

#include <dspal/test/include/dspal_log.h>

unsigned char test_data[10];

int dsp_relay_test(void) {
    LOG_INFO("Relay test");

#if 0
    LOG_INFO("GPIO Test");

    int fd = dsp_relay_gpio_relay_open(6);
    LOG_INFO("GPIO fd %d opened",fd);

    dsp_relay_gpio_relay_configure(fd,DSP_RELAY_GPIO_DIRECTION_OUTPUT);

    int iter;

    for (iter = 0; iter < 25; iter++) {

        LOG_INFO("Iter %d",iter);

        dsp_relay_gpio_relay_write(fd,1);
        dsp_relay_gpio_relay_read(fd);

        usleep(1000000);

        dsp_relay_gpio_relay_write(fd,0);
        dsp_relay_gpio_relay_read(fd);

        usleep(1000000);

    }

    dsp_relay_gpio_relay_close(fd);

#endif

#if 1

    LOG_INFO("UART Test");

//    dsp_relay_uart_check_devs();

    LOG_INFO("UART Test - Opening");

    int fd = dsp_relay_uart_relay_open(1);
    if (-1 == fd) {
        return -1;
    }

    dsp_relay_uart_relay_configure(fd, DSPAL_SIO_BITRATE_230400, 0,0,0);

    for (unsigned int byte = 0; byte < sizeof(test_data); byte++) {
        test_data[byte] = byte;
    }

    LOG_INFO("UART Test - Writing");

    dsp_relay_uart_relay_write(fd,test_data,sizeof(test_data));

    //usleep(10000000); // 10 seconds

#if 0

    LOG_INFO("UART Test - Reading");

    int bytes = 4;

    dsp_relay_uart_relay_read(fd,test_data,bytes);

    for (int byte = 0; byte < bytes; byte++) {
        printf("Byte %d: 0x%02X\n",byte,test_data[byte]);
    }
#endif

//    LOG_INFO("UART Test - Closing");
//
//    dsp_relay_uart_relay_close(fd);
//
//    return 0;
#endif
#if 0

    LOG_INFO("SPI Test");

    dsp_relay_spi_check_devs();

    LOG_INFO("SPI Test - Opening");

    int fd = dsp_relay_uart_relay_open(1);

    dsp_relay_uart_relay_configure(fd,0);

    unsigned char test_data[128];

    for (unsigned int byte = 0; byte < sizeof(test_data); byte++) {
        test_data[byte] = byte;
    }

    LOG_INFO("SPI Test - Writing");

    dsp_relay_uart_write(fd,test_data,sizeof(test_data));

    LOG_INFO("SPI Test - Closing");

    dsp_relay_uart_relay_close(fd);
#endif
    return 0;


}
