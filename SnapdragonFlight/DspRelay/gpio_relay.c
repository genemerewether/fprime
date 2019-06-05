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
#ifdef DSPAL_sdsp
    snprintf(devName, sizeof(devName), DEV_FS_GPIO_SSC_DEVICE_TYPE_STRING "%d",
#else
    snprintf(devName, sizeof(devName), DEV_FS_GPIO_DEVICE_TYPE_STRING "%d",
#endif
	     gpio);
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
        LOG_INFO("GPIO %u value %u read",fd,value_read);
	/* NOTE(mereweth) - observed weird case on 801 where DSP gpio
	 * read value was large number, odd or even to indicate state.
	 * even is low, odd is high
	 */
        return (value_read % 2);
    }

}

// struct for signal waiting
static struct GpioWaitInfo {
    sem_t semId; // semaphore for signaling waiting thread
    volatile int quitRequested; // used to signal exit of semaphore waits
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
 
long dsp_relay_gpio_relay_quit(int gpio, int fd) {

    gpioInfo[gpio].quitRequested = 1;
    int stat = sem_post(&gpioInfo[gpio].semId);
    if (-1 == stat) {
        LOG_ERR("Error posting semaphore for device %d: %s",gpio,strerror(errno));
        return -1;
    }

    struct dspal_gpio_ioctl_reg_int int_config = {
		.trigger = DSPAL_GPIOINT_TRIGGER_RISING,
		.isr = (DSPAL_GPIO_INT_ISR) &gpio_int_isr,
		.isr_ctx = (DSPAL_GPIO_INT_ISR_CTX) gpio,
	};
	stat = ioctl(fd, DSPAL_GPIO_IOCTL_CONFIG_DEREG_INT, (void *)&int_config);
	if (stat != 0) {
		LOG_ERR("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_DEREG_INT failed for device %d: %s",fd,strerror(errno));
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
		LOG_ERR("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_REG_INT failed for device %d: %s",gpio,strerror(errno));
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
