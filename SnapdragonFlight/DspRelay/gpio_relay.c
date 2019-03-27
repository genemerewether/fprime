// ======================================================================
// \title  gpio_relay.c
// \author tcanham
// \brief  gpio relay for dsp_relay
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

#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_gpio.h>
#include <gpio_defs.h>
#include <semaphore.h>
#include <errno.h>
#include <string.h>
#include <dspal_platform.h>
#include <dspal/test/include/dspal_log.h>

/**
 * @brief Open specified GPIO
 *
 * @param gpio - gpio # to open
 *
 * @return
 * file descriptor
*/

static const int NUM_DEVICES = 10;

int dsp_relay_gpio_relay_open(int gpio) {
    /* Request 100% of max clock speed, 100% of max bus speed, and max of 1us
     * hardware wakeup latency
     * See:
     * /opt/tools/leo/Qualcomm/Hexagon_SDK/3.0/incs/HAP_power.h
     * https://github.com/PX4/Firmware/blob/master/src/modules/muorb/adsp/px4muorb.cpp#L58
     */
    HAP_power_request(100, 100, 1);

//    if (gpio >= NUM_DEVICES) {
//        LOG_ERR("Invalid device %d",gpio);
//    }

    int fd;

    char devName[256];
    snprintf(devName,sizeof(devName),"/dev/gpio-%d",gpio);
    // null terminate
    devName[sizeof(devName)-1] = 0;
    LOG_INFO("Opening GPIO device %s",devName);
    fd = open(devName, 0);
    if (fd == -1) {
        LOG_ERR("open gpio device %d failed: %s",gpio,strerror(errno));
        return -1;
    } else {
        LOG_INFO("Successfully opened GPIO device %s fd %d",devName,fd);
        return fd;
    }

}

int dsp_relay_gpio_relay_close(int fd) {
    LOG_INFO("Closing GPIO device %d",fd);
    return close(fd);
}


int dsp_relay_gpio_relay_configure(int fd, int type) {
    // skip input/output setting if it is an interrupt input.
    // Interrupt callback registered in dsp_relay_gpio_relay_start_int()
    if (type != 2) {
        struct dspal_gpio_ioctl_config_io config = {
                .direction = (enum DSPAL_GPIO_DIRECTION_TYPE)type,
                .pull = DSPAL_GPIO_NO_PULL,
                .drive = DSPAL_GPIO_2MA,
            };

        // configure GPIO device into general purpose IO mode
        if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != 0) {
            LOG_ERR("ioctl gpio fd %d failed: %s",fd,strerror(errno));
            return -1;
        } else {
            LOG_INFO("GPIO fd %d successfully configured");
            return 0;
        }
    }
    return 0;
}

int dsp_relay_gpio_relay_write(int fd, int val) {

    enum DSPAL_GPIO_VALUE_TYPE value_written = (enum DSPAL_GPIO_VALUE_TYPE) val;

    // set output value
    int bytes = write(fd, &value_written, 1);

    if (bytes != 1) {
        LOG_ERR("error: GPIO %d write failed: %d: %s",fd,bytes,strerror(errno));
        return bytes;
    } else {
        LOG_INFO("GPIO %d Write %d successful",fd,val);
        return 0;
    }
}

int dsp_relay_gpio_relay_read(int fd) {

    enum DSPAL_GPIO_VALUE_TYPE value_read;

    // verify the write result by reading the output from the same GPIO
    int bytes = read(fd, &value_read, 1);

    if (bytes != 1) {
        LOG_ERR("GPIO read %d failure: %d: %s",fd,bytes,strerror(errno));
        return -1;
    } else {
        LOG_INFO("GPIO %d value %d read",fd,value_read);
        return value_read;
    }

}

// struct for signal waiting
static struct GpioWaitInfo {
    sem_t semId; // semaphore for signaling waiting thread
    int quitRequested; // used to signal exit of semaphore waits
} gpioInfo[NUM_DEVICES];;

void *gpio_int_isr(DSPAL_GPIO_INT_ISR_CTX context)
{
    uint32_t gpio = (int)context;
	LOG_INFO("GPIO ISR for gpio %d",gpio);
	int stat = sem_post(&gpioInfo[gpio].semId);
	if (-1 == stat) {
		LOG_ERR("Error posting semaphore for device %d: %s",gpio,strerror(errno));
		return NULL;
	}

	return NULL;
}
long dsp_relay_gpio_relay_quit(int gpio) {

    gpioInfo[gpio].quitRequested = 1;
    int stat = sem_post(&gpioInfo[gpio].semId);
    if (-1 == stat) {
        LOG_ERR("Error posting semaphore for device %d: %s",gpio,strerror(errno));
        return -1;
    }

    return 0;
}


long dsp_relay_gpio_relay_start_int(int gpio, int fd) {

    // set quit to false
    gpioInfo[gpio].quitRequested = 0;

    LOG_INFO("dsp_relay_gpio_relay_start_int");
	// initialize semaphore
	int stat = sem_init(&gpioInfo[gpio].semId,0,0);
	if (-1 == stat) {
		LOG_ERR("Error initializing semaphore for gpio %d: %s",gpio,strerror(errno));
		return -1;
	}

	// Configure this GPIO device as interrupt source
	struct dspal_gpio_ioctl_reg_int int_config = {
		.trigger = DSPAL_GPIOINT_TRIGGER_RISING,
		.isr = (DSPAL_GPIO_INT_ISR) &gpio_int_isr,
		.isr_ctx = (DSPAL_GPIO_INT_ISR_CTX) gpio,
	};

	stat = ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_REG_INT, (void *)&int_config);
	if (stat != 0) {
		LOG_ERR("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_REG_INT failed for device %d: %s",gpio,strerror(stat));
		return -1;
	}

	LOG_INFO("GPIO ISR registered for gpio %d",gpio);

	return 0;

}

long dsp_relay_gpio_relay_isr_wait(int gpio, int fd) {

    // quit if quit flag set
    if (1 == gpioInfo[gpio].quitRequested) {
        return 1;
    }

    LOG_INFO("Waiting for GPIO %d\n",gpio);

	// wait on semaphore for ISR
	int stat = sem_wait(&gpioInfo[gpio].semId);
	if (-1 == stat) {
		LOG_ERR("Error waiting for semaphore for gpio %d: %s",gpio,strerror(errno));
		return -1;
	}

    LOG_INFO("Received GPIO %d\n",gpio);

    // quit if quit flag set
    if (1 == gpioInfo[gpio].quitRequested) {
        return 1;
    }

    return 0;

}


#if 0
/****************************************************************************
 *   Copyright (c) 2015 James Wilson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <dev_fs_lib_gpio.h>

#include "test_status.h"
#include "test_utils.h"

#include "platform.h"

bool isr_called = false;


/**
 * @brief Test Open and Close functionality of the GPIO Device (gpio-56)
 *
 * @par Test:
 * 1) Opens file for GPIO device
 * 2) If devices file opens return TEST_PASS
 *
 * @return
 * TEST_PASS ------ Test Passes (file opened successfully)
 * TEST_FAIL ------ Test Failed (file failed to open)
*/
int dspal_tester_test_gpio_open_close(void)
{
	int result = TEST_PASS;
	int fd;
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

exit:
	close(fd);
	return result;
}

/**
* @brief Test IOCTL function to configure GPIO device into IO mode
*
* @par Test:
* 1) Opens file for GPIO device  (gpio-56)
* 2) Uses ioctl to set the GPIO pin to me in IO mode and checks that this succeeds
* 3) Repeat step 2 but check that it fails.  ioctl should fail the 2nd time it tries to configure the pin
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
*/
int dspal_tester_test_gpio_ioctl_io(void)
{
	int result = TEST_PASS;
	int fd;
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	// configure GPIO device into general purpose IO mode
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != 0) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Test if ioctl() is rejected if called more than once
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) == 0) {
		LOG_ERR("duplicate ioctl call test failed");
		result = TEST_FAIL;
		goto exit;

	} else {
		LOG_INFO("duplicate ioctl call expected to be rejected. PASSED");
	}

exit:
	close(fd);
	return result;
}


/**
* @brief This test toggles the GPIO pin at 10Hz.
*
* @par Tests:
* This test toggles the GPIO pin at 10Hz. After each write, we read the value
* from GPIO port to validate the write result.
* Test:
* 1) Opens file for GPIO device  (gpio-56)
* 2) Uses ioctl to set the GPIO pin to me in IO mode and checks that this succeeds
* 3) Write the value LOW to the GPIO pin (exit if fails to write)
* 4) Read the value of the GPIO pin back to see if the value from step 3 has been
*    written (exit if fails to read or if the value read was not the same as the value
*    written in step 3)
* 5) Loop steps 3-4 100 times at 100 milliseconds per loop, invert signal to write to GPIO Pin
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
*/
int dspal_tester_test_gpio_read_write(void)
{
	int result = TEST_PASS;
	int fd;
	int bytes;
	enum DSPAL_GPIO_VALUE_TYPE value_written = DSPAL_GPIO_LOW_VALUE;
	enum DSPAL_GPIO_VALUE_TYPE value_read;
	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	// Open GPIO device
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	// Configure GPIO device into general purpose I/O mode
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != 0) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Toggle GPIO device output value for a number of repetitions
	for (int i = 0; i < 100; i++) {
		value_written = value_written ^ 0x01;

		LOG_DEBUG("write GPIO %s: %d", GPIO_DEVICE_PATH, value_written);

		// set output value
		bytes = write(fd, &value_written, 1);

		if (bytes != 1) {
			LOG_ERR("error: write failed");
			result = TEST_FAIL;
			goto exit;
		}

		// verify the write result by reading the output from the same GPIO
		bytes = read(fd, &value_read, 1);

		if (bytes != 1) {
			LOG_ERR("error: read failed");
			result = TEST_FAIL;
			goto exit;
		}

		LOG_DEBUG("read from GPIO %s: %d", GPIO_DEVICE_PATH, value_read);

		if (value_read != value_written) {
			LOG_ERR("error: read inconsistent value");
			result = TEST_FAIL;
			goto exit;
		}

		// sleep between each toggle
		usleep(100000);
	}

exit:
	close(fd);
	return result;
}

/**
* @brief This test toggles one GPIO pin at read from another GPIO pin which it is connected by wire externally
*
* @par Tests:
* This tests uses 2 GPIO pins. It toggles one GPIO pin at 10Hz. After each write, we read the value
* from this GPIO port to validate the write result, and also read from the other GPIO pin connected externally
* Test:
* 1) Opens file for GPIO device  (gpio-10), and the other GPIO device (gpio-11)
* 2) Uses ioctl to set the two GPIO pin to me in IO mode and checks that this succeeds
* 3) Write the value LOW to the GPIO pin (exit if fails to write)
* 4) Read the value of these two GPIO pins back to see if the value from step 3 has been
*    written (exit if fails to read or if the value read was not the same as the value
*    written in step 3)
* 5) Loop steps 3-4 100 times at 100 milliseconds per loop, invert signal to write to GPIO Pin
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
*/
int dspal_tester_test_gpio_read_write_extern_loopback(void)
{
	int result = TEST_PASS;
	int fd, fd_loopback;
	int bytes, bytes2;
	enum DSPAL_GPIO_VALUE_TYPE value_written = DSPAL_GPIO_LOW_VALUE;
	enum DSPAL_GPIO_VALUE_TYPE value_read, value_read2;
	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};
	struct dspal_gpio_ioctl_config_io config2 = {
		.direction = DSPAL_GPIO_DIRECTION_INPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	// Open GPIO device
	fd = open(GPIO_DEVICE_PATH, 0);
	fd_loopback = open(GPIO_DEVICE_PATH_LOOPBACK, 0);
	if (fd == -1 || fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	// Configure GPIO device into general purpose I/O mode
	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != 0 ||
		(ioctl(fd_loopback, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config2) != 0)) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Toggle GPIO device output value for a number of repetitions
	for (int i = 0; i < 100; i++) {
		value_written = value_written ^ 0x01;

		LOG_DEBUG("write GPIO %s: %d", GPIO_DEVICE_PATH, value_written);

		// set output value
		bytes = write(fd, &value_written, 1);

		if (bytes != 1) {
			LOG_ERR("error: write failed");
			result = TEST_FAIL;
			goto exit;
		}

		// verify the write result by reading the output from the same GPIO
		bytes = read(fd, &value_read, 1);
		bytes2 = read(fd_loopback, &value_read2, 1);
		if (bytes != 1 || bytes2 != 1) {
			LOG_ERR("error: read failed");
			result = TEST_FAIL;
			goto exit;
		}

		LOG_DEBUG("read from GPIO %s: %d, %s: %d", GPIO_DEVICE_PATH, value_read, GPIO_DEVICE_PATH_LOOPBACK, value_read2);

		if (value_read != value_written || value_read != value_read2) {
			LOG_ERR("error: read inconsistent value");
			result = TEST_FAIL;
			goto exit;
		}

		// sleep between each toggle
		usleep(100000);
	}

exit:
	close(fd);
	return result;
}

/**
* @brief Interrupt service routine for the GPIO interrupt test.
*
* @return
* NULL --- Always
*/
void *gpio_int_isr(DSPAL_GPIO_INT_ISR_CTX context)
{
	bool *val = (bool *)context;

	LOG_DEBUG("gpio_int_isr called");

	*val = true;

	return NULL;
}


/**
* @brief Test to see if a GPIO hardware interrupt can be setup and used correctly
*
* @par Detailed Description:
* This tests uses 2 GPIO pins.  1 of the pins is setup as a hardware interrupt pin
* that will be triggered on a rising edge.  The other pin is set to be a GPIO IO
* pin.  It is initially set to be LOW and then the interrupt pin is configured.
* After that the IO pin state is changed from LOW to HIGH to trigger the interupt
* GPIO interrupt (the 2 pins should be wired together)

* Test:
* 1) Opens file for GPIO A device (IO Pin) (gpio-56)
* 2) Uses ioctl to set the GPIO pin A to me in IO mode and checks that this succeed
* 3) Sets the GPIO pin A to be LOW
* 4) Opens file for GPIO B device (interrupt Pin) (gpio-55)
* 5) ioctl the pin B to be a hardware interrupt pin (triggered on a rising edge)
*    using the 'gpio_int_isr' function as the interrupt service routine (ISR)
* 6) Set the GPIO pin A to be HIGH to generate a rising edge for GPIO pin B to trigger
* 7) wait 1 second of the interrupt to process
* 8) checks to see if a flag that is set by the ISR was set (AKA ISR called)
* 9) Close both GPIO devices for clean close
*
* @return
* TEST_PASS ------ Test Passes
* TEST_FAIL ------ Test Failed
* TEST_SKIP ------ Test Skipped
*/
int dspal_tester_test_gpio_int(void)
{
	int result = TEST_PASS;
#ifdef DO_JIG_TEST
	enum DSPAL_GPIO_VALUE_TYPE value_written;
	int fd;
	int int_fd = -1;
	int bytes = 0;

	// Open GPIO device at GPIO_DEVICE_PATH for general purpose I/O
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	struct dspal_gpio_ioctl_config_io config = {
		.direction = DSPAL_GPIO_DIRECTION_OUTPUT,
		.pull = DSPAL_GPIO_NO_PULL,
		.drive = DSPAL_GPIO_2MA,
	};

	if (ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != 0) {
		LOG_ERR("ioctl gpio device failed");
		result = TEST_FAIL;
		goto exit;
	}

	// set initial output value to LOW
	value_written = DSPAL_GPIO_LOW_VALUE;
	bytes = write(fd, &value_written, 1);

	if (bytes != 1) {
		LOG_ERR("error: write failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Open GPIO Device at GPIO_INT_DEVICE_PATH
	int_fd = open(GPIO_INT_DEVICE_PATH, 0);

	if (int_fd == -1) {
		LOG_ERR("open gpio device failed.");
		result = TEST_FAIL;
		goto exit;
	}

	// Configure this GPIO device as interrupt source
	struct dspal_gpio_ioctl_reg_int int_config = {
		.trigger = DSPAL_GPIOINT_TRIGGER_RISING,
		.isr = (DSPAL_GPIO_INT_ISR) &gpio_int_isr,
		.isr_ctx = (DSPAL_GPIO_INT_ISR_CTX) &isr_called,
	};

	if (ioctl(int_fd, DSPAL_GPIO_IOCTL_CONFIG_REG_INT, (void *)&int_config) != 0) {
		LOG_ERR("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_REG_INT failed");
		result = TEST_FAIL;
		goto exit;
	}

	// Set output to HIGH to generate RISING edge to trigger the interrupt on
	// the interrupt GPIO device
	value_written = DSPAL_GPIO_HIGH_VALUE;
	bytes = write(fd, &value_written, 1);

	if (bytes != 1) {
		LOG_ERR("error: write failed");
		result = TEST_FAIL;
		goto exit;
	}

	usleep(1000000);

	// check if isr was called
	if (!isr_called) {
		LOG_ERR("error: isr is not called");
		result = TEST_FAIL;
		goto exit;
	}

exit:
	close(int_fd);
	close(fd);
#else
	result = TEST_SKIP;
#endif
	return result;
}
#endif