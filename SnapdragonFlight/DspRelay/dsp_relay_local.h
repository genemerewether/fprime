/*
 * dsp_relay_local.h
 *
 *  Created on: Oct 21, 2016
 *      Author: tcanham
 */

#ifndef DSPRELAY_TEST_DSP_RELAY_ADSP_PROC_DSP_RELAY_LOCAL_H_
#define DSPRELAY_TEST_DSP_RELAY_ADSP_PROC_DSP_RELAY_LOCAL_H_

// GPIO functions

int dsp_relay_gpio_relay_open(int gpio);

int dsp_relay_gpio_relay_close(int fd);

int dsp_relay_gpio_relay_configure(int fd, int type);

int dsp_relay_gpio_relay_write(int fd, int val);

int dsp_relay_gpio_relay_read(int fd);

// UART functions

int dsp_relay_uart_check_devs(void);

int dsp_relay_uart_relay_open(int uart);

int dsp_relay_uart_relay_close(int fd);

int dsp_relay_uart_relay_read(int device, unsigned char* buff, int buffLen);

int dsp_relay_uart_relay_write(int fd, const unsigned char* buff, int buffLen);

int dsp_relay_uart_relay_configure(int fd, int baud, int parity, int bits, int stop_bits);

// SPI functions

int dsp_relay_spi_check_devs(void);

int dsp_relay_spi_relay_open(int uart);

int dsp_relay_spi_relay_close(int fd);

int dsp_relay_spi_relay_read_write(int fd, const unsigned char* write_data, int write_dataLen, unsigned char* read_data, int read_dataLen);

int dsp_relay_spi_relay_configure(int fd, int clock);



#endif /* DSPRELAY_TEST_DSP_RELAY_ADSP_PROC_DSP_RELAY_LOCAL_H_ */
