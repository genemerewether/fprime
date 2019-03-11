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


#if 0

#define SPI_TEST_CYCLES 10

#define MPU_SPI_BUF_LEN   512
/**
 * Supported SPI frequency to talk to MPU9x50 slave device
 * MPU9x50 SPI interface supports upto 20MHz frequency. However 20MHz is not
 * reliable in our test and corrupted data is observed.
 */
enum MPU_SPI_FREQUENCY
{
   MPU_SPI_FREQUENCY_1MHZ = 1000000UL,
   MPU_SPI_FREQUENCY_5MHZ = 5000000UL,
   MPU_SPI_FREQUENCY_10MHZ = 10000000UL,
   MPU_SPI_FREQUENCY_15MHZ = 15000000UL,
   MPU_SPI_FREQUENCY_20MHZ = 20000000UL,
};

static uint8_t spiTxBuf[MPU_SPI_BUF_LEN];
static uint8_t spiRxBuf[MPU_SPI_BUF_LEN];

/**
 * NOTE: DO NOT send more than 64 bytes in loopback test. SPI bus automatically
 * switches to DMA mode to send more than 64 bytes. However, DMA mode in
 * loopback transfer results in system crash or hang. Transfering more than 64
 * bytes to/from peripheral device using DMA mode is supported.
 */
#define SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH  20

/**
 * @brief Helper function  for 'dspal_tester_spi_test', checks if 2 data buffers are equal.
 *
 *
 * @param buffer1[in]  pointer to first buffer
 * @param buffer2[in]  pointer to second buffer
 * @param length[in]   length of each buffers
 *
 * @return
 * true  ------ data buffers match
 * false ------ data buffers do not match
*/
bool dpsal_tester_is_memory_matching(uint8_t *buffer1, uint8_t *buffer2, int length)
{
	if (memcmp(buffer1, buffer2, length) != 0) {
		LOG_ERR("error: the bytes read to not match the bytes written");
		LOG_ERR("bytes read: %c, %c, %c, %c, %c", buffer1[0], buffer1[1], buffer1[2],
			buffer1[3], buffer1[4]);
		LOG_ERR("bytes written: %c, %c, %c, %c, %c", buffer2[0], buffer2[1], buffer2[2],
			buffer2[3], buffer2[4]);
		return false;
	}

	return true;
}

void init_write_buffer(uint8_t *buffer, int length)
{
	int i;
	char c = 'a';

	for (i = 0; i < length; i++) {
		buffer[i] = c;

		if (c == 'z') {
			c = 'a';
		}

		c++;
	}
}

int mpu_spi_configure_speed(int fd, enum MPU_SPI_FREQUENCY freq)
{
   struct dspal_spi_ioctl_set_bus_frequency bus_freq;

   bus_freq.bus_frequency_in_hz = freq;

   return ioctl(fd, SPI_IOCTL_SET_BUS_FREQUENCY_IN_HZ, &bus_freq);
}


int mpu_spi_get_reg(int fd, int reg, uint8_t* val)
{
   int retVal;
   struct dspal_spi_ioctl_read_write read_write;


   retVal = mpu_spi_configure_speed(fd, MPU_SPI_FREQUENCY_1MHZ);
   if (retVal != 0)
   {
      LOG_ERR("mpu_spi_get_reg: error configuring speed %d", retVal);
      return retVal;
   }

   spiTxBuf[0] = reg | 0x80; //register high bit=1 for read

   read_write.read_buffer = spiRxBuf;
   read_write.read_buffer_length = 2;
   read_write.write_buffer = spiTxBuf;
   read_write.write_buffer_length = 2;
   retVal = ioctl(fd, SPI_IOCTL_RDWR, &read_write);
   if (retVal != 2)
   {
      FARF(ALWAYS, "mpu_spi_get_reg error read/write ioctl: %d", retVal);
      return retVal;
   }

   *val = spiRxBuf[1];

   FARF(LOW, "mpu_spi_get_reg %d=%d", reg, *val);

   return 0;
}

/**
* @brief Test read/write functionality of spi by using loopback
*
* @par Detailed Description:
* Tests the read and write functionality of the spi device by putting the device
* in loopback mode.  This is tested in 2 ways: writing the data then reading it
* back from the read buffer or by doing the read/write at the same time using ioctl
*
* Test:
* 1) Opens file for spi device ('/dev/spi-8')
* 2) Sets up the spi device in loopback mode using ioctl
* 3) Write to the spi bus
* 4) Read from the spi bus buffer
* 5) Commented Out ---- Check if data written matches data read
* 6) Loop though steps 4-5 for  SPI_TEST_CYCLES number of cycles
* 7) So ioctl read/write operation and check if data written matches data read
* 8) Close spi bus
*
* @return
* SUCCESS  ------ Test Passes
* ERROR ------ Test Failed
*/
int dspal_tester_spi_loopback_test(void)
{
	int spi_fildes = 0;
	int cycle_count;
	int result = 0;
	uint8_t write_data_buffer[SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH];
	uint8_t read_data_buffer[SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH];
	int test_data_length_in_bytes = SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH - 1;
	struct dspal_spi_ioctl_loopback loopback;
	struct dspal_spi_ioctl_read_write read_write;
	struct dspal_spi_ioctl_set_spi_mode bus_mode;

	LOG_DEBUG("testing spi open for: %s", SPI_DEVICE_PATH);
	spi_fildes = open(SPI_DEVICE_PATH, 0);

	if (spi_fildes < 0) {
		LOG_ERR("error: failed to open spi device path: %s", SPI_DEVICE_PATH);
		result = -1;
		goto exit;
	}

	/*
	 * Initialize the write buffers in preparation for a read/write sequence.
	 */
	write_data_buffer[SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH - 1] = 0;
	init_write_buffer(write_data_buffer, SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH - 1);

	/*
	 * Enable loopback mode to allow write/reads to be tested internally.
	 */
	LOG_DEBUG("enabling spi loopback mode");
	loopback.state = SPI_LOOPBACK_STATE_ENABLED;
	result = ioctl(spi_fildes, SPI_IOCTL_LOOPBACK_TEST, &loopback);

	if (result < 0) {
		LOG_ERR("error: unable to activate spi loopback mode");
		goto exit;
	}

	/* set bus mode, don't goto exit for downward compatible */
	bus_mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
	bus_mode.eShiftMode = SPI_OUTPUT_FIRST;
	result = ioctl(spi_fildes, SPI_IOCTL_SET_SPI_MODE, &bus_mode);
	if (result < 0)
	{
		LOG_ERR("error: unable to set bus mode");
	}

	/*
	 * Test loopback mode using combined read/write mode.
	 */
	LOG_DEBUG("testing spi write/read for %d cycles", SPI_TEST_CYCLES);

	for (cycle_count = 0; cycle_count < SPI_TEST_CYCLES; cycle_count++) {
		memset(read_data_buffer, 0, sizeof(read_data_buffer));
		read_write.read_buffer = &read_data_buffer[0];
		read_write.read_buffer_length = test_data_length_in_bytes;
		read_write.write_buffer = &write_data_buffer[0];
		read_write.write_buffer_length = test_data_length_in_bytes;

		LOG_DEBUG("writing bytes: (%d bytes)",
			  test_data_length_in_bytes);

		result = ioctl(spi_fildes, SPI_IOCTL_RDWR, &read_write);

		if (result < 0) {
			LOG_ERR("error: unable to activate read/write ioctl");
			goto exit;
		}

		if (!dpsal_tester_is_memory_matching(write_data_buffer, read_data_buffer, test_data_length_in_bytes)) {
			LOG_ERR("error: read/write memory buffers do not match");
			goto exit;
		}

		LOG_DEBUG("written data matches read data");
	}

	result = 0;
	LOG_DEBUG("SPI lookback test passed");

exit:

	if (spi_fildes > 0) {
		close(spi_fildes);
	}

	return result;
}

int dspal_tester_spi_exceed_max_length_test(void)
{
	int spi_fildes = 0;
	int result = 0;
	uint8_t write_data_buffer[DSPAL_SPI_TRANSMIT_BUFFER_LENGTH + 1];
	uint8_t read_data_buffer[DSPAL_SPI_RECEIVE_BUFFER_LENGTH + 1];
	struct dspal_spi_ioctl_loopback loopback;
	struct dspal_spi_ioctl_read_write read_write;
	struct dspal_spi_ioctl_set_spi_mode bus_mode;

	LOG_DEBUG("testing spi open for: %s", SPI_DEVICE_PATH);
	spi_fildes = open(SPI_DEVICE_PATH, 0);

	if (spi_fildes < 0) {
		LOG_ERR("error: failed to open spi device path: %s", SPI_DEVICE_PATH);
		result = -1;
		goto exit;
	}

	/*
	 * Enable loopback mode to allow write/reads to be tested internally.
	 */
	LOG_DEBUG("enabling spi loopback mode");
	loopback.state = SPI_LOOPBACK_STATE_ENABLED;
	result = ioctl(spi_fildes, SPI_IOCTL_LOOPBACK_TEST, &loopback);

	if (result < 0) {
		LOG_ERR("error: unable to activate spi loopback mode");
		goto exit;
	}

	/* set bus mode, don't goto exit for downward compatible */
	bus_mode.eClockPolarity = SPI_CLOCK_IDLE_HIGH;
	bus_mode.eShiftMode = SPI_OUTPUT_FIRST;
	result = ioctl(spi_fildes, SPI_IOCTL_SET_SPI_MODE, &bus_mode);
	if (result < 0)
	{
		LOG_ERR("error: unable to set bus mode");
	}

	read_write.read_buffer = &read_data_buffer[0];
	read_write.read_buffer_length = sizeof(read_data_buffer);
	read_write.write_buffer = &write_data_buffer[0];
	read_write.write_buffer_length = sizeof(write_data_buffer);
	result = ioctl(spi_fildes, SPI_IOCTL_RDWR, &read_write);

	if (result == 0) {
		LOG_ERR("error: SPI_IOCTL_RDWR transfer overly large data should "
			"have failed but didn't. ");
		goto exit;
	}

	result = 0;
	LOG_DEBUG("SPI exceed max write length test passed");

exit:

	if (spi_fildes > 0) {
		close(spi_fildes);
	}

	return result;
}

#define MPU9250_REG_WHOAMI		 117

int dspal_tester_spi_whoami_test(void)
{
	int spi_fildes = 0;
	int result = 0;
	uint8_t write_data_buffer[DSPAL_SPI_TRANSMIT_BUFFER_LENGTH + 1];
	uint8_t read_data_buffer[DSPAL_SPI_RECEIVE_BUFFER_LENGTH + 1];
	struct dspal_spi_ioctl_loopback loopback;
	struct dspal_spi_ioctl_read_write read_write;
	struct dspal_spi_ioctl_set_spi_mode bus_mode;

	LOG_DEBUG("testing spi open for: %s", SPI_DEVICE_PATH);
	spi_fildes = open(SPI_DEVICE_PATH, 0);

	if (spi_fildes < 0) {
		LOG_ERR("error: failed to open spi device path: %s", SPI_DEVICE_PATH);
		result = -1;
		goto exit;
	}

	int retry = 0;
	uint8_t b = 0;
	while (retry < 10)
	{
	   // get version (expecting 0x71 for the 9250)
	   mpu_spi_get_reg(spi_fildes, MPU9250_REG_WHOAMI, &b);
	   if ((b == 0x70) || (b == 0x71)) {
		  break;
	   }
	   retry++;
	}

	if (retry >= 10)
		result = -1;


exit:

	if (spi_fildes > 0) {
		close(spi_fildes);
	}

	return result;
}

/**
 * Main entry point for the SPI automated test.
 * @return
 * - ERROR: Indicates that the test has failed.
 * - 0: Test has passed
 */
int dspal_tester_spi_test(void)
{
	int result;

	LOG_INFO("beginning spi loopback test");

	if ((result = dspal_tester_spi_loopback_test()) < 0) {
		LOG_ERR("error: spi loopback test failed: %d", result);
		return result;
	}

	LOG_INFO("beginning spi exceed max write length test");

	if ((result = dspal_tester_spi_exceed_max_length_test()) < 0) {
		LOG_ERR("error: spi exceed max write length test failed: %d", result);
		return result;
	}
	LOG_INFO("beginning whoami test");
	if ((result = dspal_tester_spi_whoami_test()) < 0) {
		LOG_ERR("error: spi whoami test failed: %d", result);
		return result;
	}
	return 0;
}

#endif
