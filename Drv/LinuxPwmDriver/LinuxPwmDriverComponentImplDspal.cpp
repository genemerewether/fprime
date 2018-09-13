// ======================================================================
// \title  LinuxPwmDriverImplDspal.cpp
// \author tcanham
// \brief  cpp file for LinuxPwmDriver component implementation class
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


#include <Drv/LinuxPwmDriver/LinuxPwmDriverComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Os/TaskString.hpp>

#include <dev_fs_lib_pwm.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

// TODO make proper static constants for these
#define DSPAL_PWM_PATH "/dev/pwm-"
#define MAX_BUF 64

#define MAX_NUM_CHANNELS 8

#include <HAP_farf.h>
//#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#define DEBUG_PRINT(x,...)

namespace Drv {

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void LinuxPwmDriverComponentImpl ::
      pwmConfig_handler(
          const NATIVE_INT_TYPE portNum,
          PwmConfig pwmConfig
      )
    {
        // TODO(mereweth) - can't do this on DSPAL without reissuing ioctls
    }

    void LinuxPwmDriverComponentImpl ::
      pwmSetDuty_handler(
          const NATIVE_INT_TYPE portNum,
          PwmSetDutyCycle pwmSetDutyCycle
      )
    {
        if (!this->m_handle) {
            //TODO(mereweth) - issue EVR
            return;
        }

        struct dspal_pwm_ioctl_update_buffer * updateBuf =
            (struct dspal_pwm_ioctl_update_buffer *) this->m_handle;
        FW_ASSERT(updateBuf->num_gpios == this->m_numGpios);
        U32 bitmask = pwmSetDutyCycle.getbitmask();

        NATIVE_INT_TYPE dutySize = 0;
        const F32* duty = pwmSetDutyCycle.getdutyCycle(dutySize);

        for (int i = 0; i < FW_MIN(dutySize, m_numGpios); i++) {
            if (bitmask & (1 << i)) {
                 updateBuf->pwm_signal[i].pulse_width_in_usecs =
                      (U32) (m_periodInUsecs * duty[i]);
            }
        }
    }

    bool LinuxPwmDriverComponentImpl ::
      open(NATIVE_UINT_TYPE pwmchip,
           NATIVE_UINT_TYPE * channel,
           NATIVE_UINT_TYPE channelSize,
           NATIVE_UINT_TYPE period_in_usecs) {
        // TODO check for invalid pwm device?

        // Configure:
        int fd, len;
        char buf[MAX_BUF];

        if (channelSize > MAX_NUM_CHANNELS) {
            DEBUG_PRINT("not enough channel slots: %d < %d!\n",
                        channelSize, MAX_NUM_CHANNELS);
            return false;
        }

        len = snprintf(buf, sizeof(buf), DSPAL_PWM_PATH "%d", pwmchip);
        FW_ASSERT(len > 0, len);

        fd = ::open(buf, 0);
        if (fd < 0) {
            //this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
            DEBUG_PRINT("pwm/fd_open error!\n");
            return false;
        }
        this->m_fd = fd;

        struct dspal_pwm pwm_gpio[MAX_NUM_CHANNELS];
        struct dspal_pwm_ioctl_signal_definition signal_definition;
        struct dspal_pwm_ioctl_update_buffer *update_buffer;

        this->m_pwmchip = pwmchip;

        for (int i = 0; i < channelSize; i++) {
            pwm_gpio[i].gpio_id = channel[i];
            pwm_gpio[i].pulse_width_in_usecs = 0;
        }

        // Describe the overall signal and reference the above array.
        signal_definition.num_gpios = channelSize;
        m_numGpios = channelSize;
        signal_definition.period_in_usecs = period_in_usecs;
        this->m_periodInUsecs = period_in_usecs;
        signal_definition.pwm_signal = &pwm_gpio[0];

        // Send the signal definition to the DSP.
        if (ioctl(fd, PWM_IOCTL_SIGNAL_DEFINITION, &signal_definition) != 0) {
            return false;
        }

        // Retrieve the shared buffer which will be used below to update the desired
        // pulse width.
        if (ioctl(fd, PWM_IOCTL_GET_UPDATE_BUFFER, &update_buffer) != 0)
        {
            return false;
        }
        FW_ASSERT(update_buffer->num_gpios == m_numGpios);
        this->m_handle = (void *) update_buffer;

        return true;
    }

    LinuxPwmDriverComponentImpl ::
      ~LinuxPwmDriverComponentImpl(void)
    {
        if (this->m_fd != -1) {
            DEBUG_PRINT("Closing PWM %d fd %d\n",this->m_pwmchip, this->m_fd);
            close(this->m_fd);
        }

    }


} // end namespace Drv
