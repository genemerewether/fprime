// ======================================================================
// \title  spi_relay.c
// \author tcanham
// \brief  spi relay for dsp_relay
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

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <dev_fs_lib_spi.h>
#include <errno.h>

#include <dspal_platform.h>
#include <dspal/test/include/dspal_log.h>

int dsp_relay_spi_relay_open(int device) {
    /* Request 100% of max clock speed, 100% of max bus speed, and max of 1us
     * hardware wakeup latency
     * See:
     * /opt/tools/leo/Qualcomm/Hexagon_SDK/3.0/incs/HAP_power.h
     * https://github.com/PX4/Firmware/blob/master/src/modules/muorb/adsp/px4muorb.cpp#L58
     */
    HAP_power_request(100, 100, 1);

    int fd = -1;


    char devName[256];
    snprintf(devName,sizeof(devName),DEV_FS_SPI_DEVICE_TYPE_STRING "%d",device);
    // null terminate
    devName[sizeof(devName)-1] = 0;
    LOG_INFO("Opening SPI device %s",devName);
    // GG: If this is failing check that you have the latest flight controller add on install on your board
    fd = open(devName, O_RDWR);
    if (fd == -1) {
        LOG_ERR("open SPI device %d failed. %d: %s",device,errno,strerror(errno));
    } else {
        LOG_INFO("Successfully opened SPI device %s fd %d",devName,fd);
    }

    return fd;
}


int dsp_relay_spi_check_devs(void) {

    for (int spi = 1; spi < 10; spi++) {

        char devName[256];
        snprintf(devName,sizeof(devName),"/dev/spi-%d",spi);
        // null terminate
        devName[sizeof(devName)-1] = 0;
        LOG_INFO("Opening SPI device %s",devName);
        int fd = open(devName, 0);
        if (fd == -1) {
            LOG_ERR("open SPI device %d failed. %d: %s",spi,errno,strerror(errno));
        } else {
            LOG_INFO("Successfully opened SPI device %s fd %d",devName,fd);
        }

        // check rates
        for (int rate = 1000000; rate <= 10000000; rate+= 1000000) {

        	struct dspal_spi_ioctl_set_bus_frequency bus_freq;

        	bus_freq.bus_frequency_in_hz = rate;

        	if (ioctl(fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq)  != 0) {
                LOG_ERR("SPI rate %d failed",rate);
            } else {
                LOG_INFO("SPI rate %d passed",rate);
            }

        }

        if (fd != -1) {
            close(fd);
        }

    }

    return 0;

}

int dsp_relay_spi_relay_read_write(int fd, const unsigned char* write_data, int write_dataLen, unsigned char* read_data, int read_dataLen) {

	struct dspal_spi_ioctl_read_write read_write;

	read_write.read_buffer = (void*) read_data;
	read_write.read_buffer_length = read_dataLen;
	read_write.write_buffer = (void*) write_data;
	read_write.write_buffer_length = write_dataLen;

	unsigned int byte;
    for (byte = 0; byte < read_write.read_buffer_length; byte++) {
    	read_data[byte] = 0xA5;
    }

    // We must update the slave address before/after writing to get the chip
    // select behavior desired for the FPGA:
	struct dspal_spi_ioctl_set_options options = {
		.slave_address = 0,
		.is_tx_data_synchronous = 0,
		.tx_data_callback = 0,
		.rx_data_callback = 0,
	};

	int result = ioctl(fd, SPI_IOCTL_SET_OPTIONS, &options);

	if (result < 0) {
		LOG_ERR("SPI %d slave set 1 error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

	// For some reason we need to re-assert this every write.  Otherwise, we
	// see the clk line go idle high after the first write.
    struct dspal_spi_ioctl_set_spi_mode mode = {
    		.eClockPolarity = SPI_CLOCK_IDLE_LOW,
			.eShiftMode = SPI_INPUT_FIRST,
    };

    // configure SPI mode:
    if (ioctl(fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != 0) {
        LOG_ERR("ioctl SPI SET MODE 1 fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    }

    // Finally, we can write:
    LOG_INFO("Writing %d bytes to SPI",read_write.write_buffer_length);

	result = ioctl(fd, SPI_IOCTL_RDWR, &read_write);

	if (result < 0) {
		LOG_ERR("SPI %d read/write error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

	// Once again to get the desired chip select behavior after writing:
	options.slave_address = 1;

	result = ioctl(fd, SPI_IOCTL_SET_OPTIONS, &options);

	if (result < 0) {
		LOG_ERR("SPI %d slave set 2 error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

    // Once again re-asserting the SPI mode, so that the clk is idle low after writing:
    if (ioctl(fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != 0) {
        LOG_ERR("ioctl SPI SET MODE 2 fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    }

#if 1
	LOG_INFO("SPI: ");
    for (byte = 0; byte < read_write.write_buffer_length; byte++) {
    	LOG_INFO("write_data: 0x%02X ",write_data[byte]);
    }
    for (byte = 0; byte < read_write.read_buffer_length; byte++) {
    	LOG_INFO("read_data: 0x%02X ",read_data[byte]);
    }
    LOG_INFO("\n");
#endif

	return 0;
}

int dsp_relay_spi_relay_configure(int fd, int clock) {

    struct dspal_spi_ioctl_set_bus_frequency rate = {
            .bus_frequency_in_hz = clock
        };

	struct dspal_spi_ioctl_loopback loopback;


    // configure SPI clock rate
    if (ioctl(fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, (void *)&rate) != 0) {
        LOG_ERR("ioctl SPI SET FREQ fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    } else {
        LOG_INFO("SPI fd %d freq successfully configured to %d",fd,clock);
    }

    struct dspal_spi_ioctl_set_spi_mode mode = {
    		.eClockPolarity = SPI_CLOCK_IDLE_LOW,
			.eShiftMode = SPI_INPUT_FIRST,
    };

    // configure SPI clock rate
    if (ioctl(fd, SPI_IOCTL_SET_SPI_MODE, (void *)&mode) != 0) {
        LOG_ERR("ioctl SPI SET MODE fd %d failed. %d: %s",fd,errno,strerror(errno));
        return errno;
    } else {
        LOG_INFO("SPI fd %d mode successfully configured",fd);
    }

	loopback.state = SPI_LOOPBACK_STATE_DISABLED;

	int result = ioctl(fd, SPI_IOCTL_LOOPBACK_TEST, &loopback);

	if (result < 0) {
		LOG_ERR("error: unable to activate spi %d loopback mode. %d: %s",fd,errno,strerror(errno));
		return errno;
	} else {
		LOG_INFO("%s SPI loopback mode",loopback.state == SPI_LOOPBACK_STATE_DISABLED?"Disabled":"Enabled");
	}

	// Updating the slave address to get the desired chip select behavior:
	struct dspal_spi_ioctl_set_options options = {
		.slave_address = 1,
		.is_tx_data_synchronous = 0,
		.tx_data_callback = 0,
		.rx_data_callback = 0,
	};

	result = ioctl(fd, SPI_IOCTL_SET_OPTIONS, &options);

	if (result < 0) {
		LOG_ERR("SPI %d slave set error! %d: %s",fd,errno,strerror(errno));
		return errno;
	}

	return 0;
}

int dsp_relay_spi_relay_close(int fd) {
    LOG_INFO("Closing SPI device %d",fd);
    return close(fd);
}
