// ======================================================================
// \title  pwm_relay.c
// \author tcanham
// \brief  pwm relay for dsp_relay
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
#include <dev_fs_lib_pwm.h>
#include <semaphore.h>
#include <errno.h>
#include <string.h>
#include <dspal_platform.h>
#include <dspal/test/include/dspal_log.h>

int dsp_relay_pwm_relay_open(int device) {
    /* Request 100% of max clock speed, 100% of max bus speed, and max of 1us
     * hardware wakeup latency
     * See:
     * /opt/tools/leo/Qualcomm/Hexagon_SDK/3.0/incs/HAP_power.h
     * https://github.com/PX4/Firmware/blob/master/src/modules/muorb/adsp/px4muorb.cpp#L58
     */
    HAP_power_request(100, 100, 1);

    int fd;

    char devName[256];
    snprintf(devName, sizeof(devName), DEV_FS_PWM_DEVICE_TYPE_STRING "%d", device);
    // null terminate
    devName[sizeof(devName)-1] = 0;
    LOG_INFO("Opening PWM device %s",devName);
    fd = open(devName, 0);
    if (fd == -1) {
        LOG_ERR("open PWM device %d failed: %s",device,strerror(errno));
        return -1;
    } else {
        LOG_INFO("Successfully opened PWM device %s fd %d",devName,fd);
        return fd;
    }
}

int dsp_relay_pwm_relay_close(int fd) {
    LOG_INFO("Closing PWM device %d",fd);
    return close(fd);
}

int dsp_relay_pwm_relay_configure(int fd, uint64* handle, const unsigned int* channel, unsigned int channelLen, unsigned int period_in_usecs) {
    struct dspal_pwm pwm_gpio[DEV_FS_PWM_MAX_NUM_SIGNALS];
    struct dspal_pwm_ioctl_signal_definition signal_definition;
    struct dspal_pwm_ioctl_update_buffer *update_buffer;

    for (int i = 0; i < channelLen; i++) {
        pwm_gpio[i].gpio_id = channel[i];
        pwm_gpio[i].pulse_width_in_usecs = 0;
    }

    // Describe the overall signal and reference the above array.
    signal_definition.num_gpios = channelLen;
    signal_definition.period_in_usecs = period_in_usecs;
    signal_definition.pwm_signal = &pwm_gpio[0];

    // Send the signal definition to the DSP.
    if (ioctl(fd, PWM_IOCTL_SIGNAL_DEFINITION, &signal_definition) != 0) {
        LOG_ERR("failed to send pwm signal def to DSP\n");
        return -2;
    }

    // Retrieve the shared buffer which will be used below to update the desired
    // pulse width.
    if (ioctl(fd, PWM_IOCTL_GET_UPDATE_BUFFER, &update_buffer) != 0)
    {
        LOG_ERR("failed to get shared pwm buffer from DSP\n");
        return -3;
    }
    *handle = (uint64) update_buffer;
    LOG_INFO("Successfully configured PWM fd %d; handle %llu",fd,*handle);
    return 0;
}

int dsp_relay_pwm_relay_set_duty(uint64 handle, const unsigned int* pulse_width_in_usecs, unsigned int pulse_width_in_usecsLen, unsigned int bitmask) {
    if (0 == handle) {
        LOG_ERR("bad handle in pwm set duty\n");
        return -2;
    }

    struct dspal_pwm_ioctl_update_buffer * updateBuf =
        (struct dspal_pwm_ioctl_update_buffer *) handle;

    for (int i = 0; i < updateBuf->num_gpios; i++) {
        if (i >= pulse_width_in_usecsLen) {  return -3;  }
        if (bitmask & (1 << i)) {
            LOG_INFO("setting pwm %d to %u\n", i, pulse_width_in_usecs[i]);
            updateBuf->pwm_signal[i].pulse_width_in_usecs = pulse_width_in_usecs[i];
        }
    }
    return 0;
}
