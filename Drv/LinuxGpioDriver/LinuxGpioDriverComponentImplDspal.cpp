// ======================================================================
// \title  LinuxGpioDriverImpl.cpp
// \author tcanham
// \brief  cpp file for LinuxGpioDriver component implementation class
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


#include <Drv/LinuxGpioDriver/LinuxGpioDriverComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Os/TaskString.hpp>

#include <dev_fs_lib_gpio.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>

// TODO make proper static constants for these
#define DSPAL_GPIO_PATH "/dev/gpio-"
#define MAX_BUF 64

#include <HAP_farf.h>
//#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#define DEBUG_PRINT(x,...)

namespace Drv {

    void *gpio_int_isr(DSPAL_GPIO_INT_ISR_CTX context)
    {
        void * val = (void *)context;

        LinuxGpioDriverComponentImpl::intTaskEntry(val);

        return NULL;
    }

    /****************************************************************
     * gpio_fd_open
     ****************************************************************/

    int gpio_fd_open(unsigned int gpio)
    {
        int fd, len;
        char buf[MAX_BUF];

        len = snprintf(buf, sizeof(buf), DSPAL_GPIO_PATH "%d", gpio);
        FW_ASSERT(len > 0, len);

        fd = open(buf, 0);
        if (fd < 0) {
            DEBUG_PRINT("gpio/fd_open error!\n");
            return -1;
        }
        return fd;
    }

    /****************************************************************
     * gpio_fd_close
     ****************************************************************/

    int gpio_fd_close(int fd, unsigned int gpio)
    {
        return 0;
    }


  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void LinuxGpioDriverComponentImpl ::
    gpioRead_handler(
        const NATIVE_INT_TYPE portNum,
        bool &state
    )
  {
      FW_ASSERT(this->m_fd != -1);

      enum DSPAL_GPIO_VALUE_TYPE val;
      int bytes = read(this->m_fd, &val, 1);
      if (bytes != 1) {
          this->log_WARNING_HI_GP_WriteError(this->m_gpio,bytes);
          return;
      } else {
          state = (val == DSPAL_GPIO_HIGH_VALUE)?true:false;
      }
  }

  void LinuxGpioDriverComponentImpl ::
    gpioWrite_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
      FW_ASSERT(this->m_fd != -1);

      enum DSPAL_GPIO_VALUE_TYPE val = state ? DSPAL_GPIO_HIGH_VALUE : DSPAL_GPIO_LOW_VALUE;
      int bytes = write(this->m_fd, &val, 1);

      if (bytes != 1) {
          this->log_WARNING_HI_GP_WriteError(this->m_gpio,bytes);
          return;
      }
  }

  bool LinuxGpioDriverComponentImpl ::
    open(NATIVE_INT_TYPE gpio, GpioDirection direction) {
      // TODO check for invalid gpio?

      // Configure:
      this->m_fd = gpio_fd_open(gpio);

      struct dspal_gpio_ioctl_config_io config;
      config.direction = DSPAL_GPIO_DIRECTION_INPUT;
      config.pull = DSPAL_GPIO_NO_PULL;
      config.drive = DSPAL_GPIO_2MA;

      switch (direction) {
          case GPIO_INT:
          {
              // Configure this GPIO device as interrupt source
              struct dspal_gpio_ioctl_reg_int int_config = {
                .trigger = DSPAL_GPIOINT_TRIGGER_RISING,
                .isr = (DSPAL_GPIO_INT_ISR) &gpio_int_isr,
                .isr_ctx = (DSPAL_GPIO_INT_ISR_CTX) this,
              };

              if (ioctl(this->m_fd, DSPAL_GPIO_IOCTL_CONFIG_REG_INT, (void *)&int_config) != 0) {
                  this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
                  DEBUG_PRINT("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_REG_INT failed\n");
              }
          }
              break;
          case GPIO_OUT:
              config.direction = DSPAL_GPIO_DIRECTION_OUTPUT;
              // NOTE(mereweth) - direction set to input above; fall through to finish handling
          case GPIO_IN:
              if (ioctl(this->m_fd, DSPAL_GPIO_IOCTL_CONFIG_IO, (void *)&config) != 0) {
                  this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
                  DEBUG_PRINT("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_IO failed\n");
              }
              break;
          default:
              DEBUG_PRINT("Unhandled direction %d in LinuxGpioDriver open\n", direction);
              FW_ASSERT(0, direction);
              break;
      }

      this->m_gpio = gpio;

      return true;
  }

  //! Reuse interrupt task from Linux - but this gets called from DSPAL interrupt
  void LinuxGpioDriverComponentImpl ::
    intTaskEntry(void * ptr) {

      FW_ASSERT(ptr);
      LinuxGpioDriverComponentImpl* compPtr = (LinuxGpioDriverComponentImpl*) ptr;

      if (compPtr->m_quitThread) {
          return;
      }

      // call interrupt ports
      Svc::TimerVal timerVal;
      timerVal.take();

      for (NATIVE_INT_TYPE port = 0; port < compPtr->getNum_intOut_OutputPorts(); port++) {
          if (compPtr->isConnected_intOut_OutputPort(port)) {
              compPtr->intOut_out(port,timerVal);
          }
      }
  }

  Os::Task::TaskStatus LinuxGpioDriverComponentImpl ::
  startIntTask(NATIVE_INT_TYPE priority, NATIVE_INT_TYPE cpuAffinity) {
      // NOTE(mereweth) - no task needed on DSPAL; instead use this to turn on/off
      this->m_quitThread = false;

      return Os::Task::TASK_OK;
  }

  void LinuxGpioDriverComponentImpl ::
    exitThread(void) {
      this->m_quitThread = true;
  }

  LinuxGpioDriverComponentImpl ::
    ~LinuxGpioDriverComponentImpl(void)
  {
      if (this->m_fd != -1) {
          DEBUG_PRINT("Closing GPIO %d fd %d\n",this->m_gpio, this->m_fd);
          (void) gpio_fd_close(this->m_fd, this->m_gpio);
      }

  }


} // end namespace Drv
