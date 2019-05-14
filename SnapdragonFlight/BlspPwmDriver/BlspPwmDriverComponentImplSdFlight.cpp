// ======================================================================
// \title  BlspPwmDriverImplSdFlight.cpp
// \author tcanham
// \brief  cpp file for BlspPwmDriver component implementation class
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


#include <SnapdragonFlight/BlspPwmDriver/BlspPwmDriverComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include "SnapdragonFlight/DspRelay/dsp_relay.h"

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

// TODO(Mereweth) - from dspal, but can't include
#define DEV_FS_PWM_MAX_NUM_SIGNALS 8

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void BlspPwmDriverComponentImpl ::
      pwmConfig_handler(
          const NATIVE_INT_TYPE portNum,
          Drv::PwmConfig pwmConfig
      )
    {
        // TODO(mereweth) - can't do this on DSPAL without reissuing ioctls
    }

    void BlspPwmDriverComponentImpl ::
      pwmSetDuty_handler(
          const NATIVE_INT_TYPE portNum,
          Drv::PwmSetDutyCycle pwmSetDutyCycle
      )
    {
        DEBUG_PRINT("pwm set duty\n");
        if (!this->m_handle) {
            //TODO(mereweth) - issue EVR
            return;
        }

        U64 handle = (U64) this->m_handle;
        U32 pulse_width_in_usecs[DEV_FS_PWM_MAX_NUM_SIGNALS];

        NATIVE_INT_TYPE dutySize = 0;
        const F32* duty = pwmSetDutyCycle.getdutyCycle(dutySize);

        for (int i = 0; i < FW_MIN(dutySize, m_numGpios); i++) {
            pulse_width_in_usecs[i] = (U32) (this->m_periodInUsecs * duty[i]);
            DEBUG_PRINT("setting pwm %d to duty %f, usec %u\n",
                        i, duty[i], pulse_width_in_usecs[i]);
        }

        U32 bitmask = pwmSetDutyCycle.getbitmask();
        int stat = dsp_relay_pwm_relay_set_duty(handle,
                                                pulse_width_in_usecs,
                                                DEV_FS_PWM_MAX_NUM_SIGNALS,
                                                bitmask);
    }

    bool BlspPwmDriverComponentImpl ::
      open(NATIVE_UINT_TYPE pwmchip,
           NATIVE_UINT_TYPE * channel,
           NATIVE_UINT_TYPE channelSize,
           F32 * initDutyCycle,
           NATIVE_UINT_TYPE period_in_usecs) {
        // TODO check for invalid pwm device?

        if (channelSize > DEV_FS_PWM_MAX_NUM_SIGNALS) {
            DEBUG_PRINT("not enough channel slots: %d < %d!\n",
                        channelSize, DEV_FS_PWM_MAX_NUM_SIGNALS);
            this->log_WARNING_HI_PWM_OpenError(pwmchip, 0, -2);
            return false;
        }

        int fd = dsp_relay_pwm_relay_open(pwmchip);
        if (-1 == fd) {
            this->log_WARNING_HI_PWM_OpenError(pwmchip, 0, fd);
            return false;
        }
        this->m_pwmchip = pwmchip;

        U64 handle = 0u;
        int stat = dsp_relay_pwm_relay_configure(fd, &handle,
                                                 channel, channelSize,
                                                 period_in_usecs);

        if ((0 != stat) ||
            (0 == handle)) {
            this->log_WARNING_HI_PWM_OpenError(pwmchip, 0, stat);
            return false;
        }
        this->m_fd = fd;
        this->m_periodInUsecs = period_in_usecs;
        this->m_handle = (void *) handle;
        this->m_numGpios = channelSize;

        U32 pulse_width_in_usecs[DEV_FS_PWM_MAX_NUM_SIGNALS];
        for (int i = 0; i < this->m_numGpios; i++) {
            pulse_width_in_usecs[i] = this->m_periodInUsecs * initDutyCycle[i];
        }
        stat = dsp_relay_pwm_relay_set_duty(handle,
                                            pulse_width_in_usecs,
                                            DEV_FS_PWM_MAX_NUM_SIGNALS,
                                            0xff);

        if (0 != stat) {
            this->log_WARNING_HI_PWM_OpenError(pwmchip, 0, stat);
            return false;
        }

        return true;
    }

    BlspPwmDriverComponentImpl ::
      ~BlspPwmDriverComponentImpl(void)
    {
        if (this->m_fd != -1) {
            DEBUG_PRINT("Closing PWM %d fd %d\n",this->m_pwmchip, this->m_fd);
            dsp_relay_pwm_relay_close(this->m_fd);
        }

    }


} // end namespace SnapdragonFlight
