// ======================================================================
// \title  LinuxPwmDriverImpl.cpp
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

#include <HAP_farf.h>
//#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#define DEBUG_PRINT(x,...)

namespace Drv {

  /****************************************************************
   * pwm_fd_open
   ****************************************************************/

  int pwm_fd_open(unsigned int pwmchip)
  {
      int fd, len;
      char buf[MAX_BUF];

      len = snprintf(buf, sizeof(buf), DSPAL_PWM_PATH "%d", pwmchip);
      FW_ASSERT(len > 0, len);

      fd = open(buf, 0);
      if (fd < 0) {
          DEBUG_PRINT("pwm/fd_open error!\n");
          return -1;
      }
      return fd;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void LinuxPwmDriverComponentImpl ::
    pwmConfig_handler(
        const NATIVE_INT_TYPE portNum,
        U32 period
    )
  {
    // TODO
  }

  void LinuxPwmDriverComponentImpl ::
    pwmSetDuty_handler(
        const NATIVE_INT_TYPE portNum,
        F32 dutyCycle,
        U32 bitmask
    )
  {
    // TODO
  }

  bool LinuxPwmDriverComponentImpl ::
    open(NATIVE_INT_TYPE pwmchip) {
      // TODO check for invalid pwm device?

      // Configure:
      this->m_fd = pwm_fd_open(pwmchip);

      if (-1 == this->m_fd) {
          //this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
          return false;
      }

      this->m_pwmchip = pwmchip;

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
