// ======================================================================
// \title  dsp_relay_main.c
// \author tcanham
// \brief  main Hexagon file for dsp_relay
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
//
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ======================================================================

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "adspmsgd.h"
#include "rpcmem.h"

#include "termios.h"

#include "SnapdragonFlight/DspRelay/dsp_relay.h"

#include "SnapdragonFlight/DspRelay/uart_defs.h"

int main(int argc, char *argv[])
{

#if 1 // flow control
    int device = 5;
    int toggles = 6;

    printf("Opening UART device %d\n",device);

    int fd = dsp_relay_uart_relay_open(device);
    if (-1 == fd) {
        printf("Error opening device %d\n",device);
        return -1;
    }

    printf("Configuring device\n");

    int stat = dsp_relay_uart_relay_configure(fd, device, UART_BITRATE_115200, 0,0,0,0,1);
    if (-1 == stat) {
        printf("Error configuring device %d\n",device);
        return -1;
    }

    int toggle;
    for (toggle = 0; toggle < toggles; toggle++) {
        if (toggle % 2 == 0) {
            tcflow(fd,TCOOFF);
        } else {
            tcflow(fd,TCOON);
        }

    }

#endif

#if 0
    int device = 1;

    if (argc != 2) {
    	printf("No args!\n");
    	return -1;
    }

    int expected_bytes = atoi(argv[1]);
    printf("Expecting to read %d bytes\n",expected_bytes);

    // DSP sends 120 bytes at a time, so double just for safety
    int stat = dsp_relay_uart_receive_allocate(device,256);
    if (-1 == stat) {
        printf("Error allocating for device %d\n",device);
        return -1;
    }

    printf("Opening UART device %d\n",device);

    int fd = dsp_relay_uart_relay_open(device);
    if (-1 == fd) {
        printf("Error opening device %d\n",device);
        return -1;
    }

    printf("Configuring device\n");

    stat = dsp_relay_uart_relay_configure(fd, device, UART_BITRATE_115200, 0,0,0,0);
    if (-1 == stat) {
        printf("Error configuring device %d\n",device);
        return -1;
    }

    unsigned char buffer[512];

    unsigned int byte;
    for (byte = 0; byte < sizeof(buffer); byte++) {
        buffer[byte] = byte;
    }

    printf("Writing device\n");

    unsigned int trans = 0;
    for (trans = 0; trans < 20; trans++) {

        stat = dsp_relay_uart_relay_write(fd, device, buffer,sizeof(buffer));

        if (-1 == stat) {
            printf("Error writing device %d\n",device);
            return -1;
        }

    }
    printf("Reading device\n");

    unsigned char readBuff[256];

    int msg = 0;
    int total_bytes = 0;
    while (1) {
//    for (msg = 0; msg < 3; msg++) {
        int bytes_received = 0;
    	stat = dsp_relay_uart_relay_read(device,readBuff,sizeof(readBuff),&bytes_received);
    	int index = 0;
#if 0
    	for (index = 0; index < bytes_received; index++) {
    		printf("0x%02X ",readBuff[index]);
    	}
#endif
    	// expect a ramping pattern
    	for (index = 0; index < bytes_received; index++) {
    		if (readBuff[index] != (total_bytes+index)%256) {
    			printf("Mismatch (%d)! Expected: 0x%02X Actual: 0x%02X\n",total_bytes+index,(total_bytes+index)%255,readBuff[index]);
    			goto exit;
    		}
    	}

//    	printf(" (%d/%d/%d)\n",bytes_received,total_bytes,expected_bytes);
    	total_bytes += bytes_received;
    	if (total_bytes >= expected_bytes) {
    		break;
    	}
    }

exit:
    printf("Closing device\n");

    stat = dsp_relay_uart_relay_close(fd,device);

    if (-1 == stat) {
        printf("Error closing device %d\n",device);
        return -1;
    }

#endif

#if 0


    int spiDevice = 8;

    printf("Opening SPI device %d\n",spiDevice);

    int fd = dsp_relay_spi_relay_open(spiDevice);
    if (-1 == fd) {
        printf("Error opening SPI device %d\n",spiDevice);
        return -1;
    }

    printf("Configuring SPI device\n");

    int stat = dsp_relay_spi_relay_configure(fd, 1000000);
    if (-1 == stat) {
        printf("Error configuring device %d\n",spiDevice);
        close(fd);
        return -1;
    }

    unsigned char out_buffer[10];
    unsigned char in_buffer[10];

    unsigned int byte;
    for (byte = 0; byte < sizeof(out_buffer); byte++) {
    	out_buffer[byte] = byte;
        in_buffer[byte] = 0xA5;
    }

    printf("Writing SPI device\n");

    stat = dsp_relay_spi_relay_read_write(fd,out_buffer,sizeof(out_buffer),in_buffer,sizeof(in_buffer));

    if (-1 == stat) {
        printf("Error writing/reading SPI device %d\n",spiDevice);
        close(fd);
        return -1;
    }

    printf("SPI: ");
    for (byte = 0; byte < sizeof(out_buffer); byte++) {
        printf("0x%02X ",in_buffer[byte]);
    }
    printf("\n");


    printf("Closing SPI device\n");

    stat = dsp_relay_spi_relay_close(fd);

    if (-1 == stat) {
        printf("Error closing SPI device %d\n",spiDevice);
        return -1;
    }



#endif

#if 0
    int gpio = 47;

    printf("Opening GPIO device %d\n",gpio);

    int fd = dsp_relay_gpio_relay_open(gpio);

    if (-1 == fd) {
        printf("Error opening SPI device %d\n",gpio);
        return -1;
    }

    printf("Starting GPIO interrupt handling for device %d\n",gpio);

    int stat = dsp_relay_gpio_relay_start_int(fd);

    if (-1 == stat) {
        printf("Error starting GPIO device %d interrupt\n",gpio);
        return -1;
    }

    int wait = 0;
    for (wait = 0; wait < 3; wait++) {

    printf("Waiting for interrupt from GPIO %d\n",gpio);

		stat = dsp_relay_gpio_relay_isr_wait(fd);
		if (-1 == stat) {
			printf("Error waiting for GPIO device %d interrupt\n",gpio);
			return -1;
		}

		printf("Got interrupt from GPIO %d\n",gpio);

    }


    printf("Closing GPIO device %d\n",gpio);

    stat = dsp_relay_gpio_relay_close(fd);

    if (-1 == stat) {
        printf("Error closing GPIO device %d\n",gpio);
        return -1;
    }


#endif
    return 0;
}
