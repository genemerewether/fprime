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

#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
//#define DEBUG_PRINT(x,...)

namespace Drv {


    /****************************************************************
    * gpio_export
    ****************************************************************/
    int gpio_export(unsigned int gpio)
    {
        return 0;
    }

    /****************************************************************
     * gpio_unexport
     ****************************************************************/
    int gpio_unexport(unsigned int gpio)
    {
        return 0;
    }

    /****************************************************************
     * gpio_set_dir
     ****************************************************************/
    int gpio_set_dir(unsigned int gpio, unsigned int out_flag)
    {
        return 0;
    }

    /****************************************************************
     * gpio_set_value
     ****************************************************************/
    int gpio_set_value(int fd, unsigned int value)
    {
        return 0;
    }

    /****************************************************************
     * gpio_get_value
     ****************************************************************/
    int gpio_get_value(int fd, unsigned int *value)
    {
        return 0;
    }


    /****************************************************************
     * gpio_set_edge
     ****************************************************************/

    int gpio_set_edge(unsigned int gpio, char *edge)
    {
        return 0;
    }

    /****************************************************************
     * gpio_fd_open
     ****************************************************************/

    int gpio_fd_open(unsigned int gpio)
    {
        return 0;
    }

    /****************************************************************
     * gpio_fd_close
     ****************************************************************/

    int gpio_fd_close(int fd, unsigned int gpio)
    {
        // TODO is this needed? w/o this the edge file and others can retain the state from
        // previous settings.
        (void) gpio_unexport(gpio); // TODO check return value

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

      unsigned int val;
      NATIVE_INT_TYPE stat = gpio_get_value(this->m_fd, &val);
      if (-1 == stat) {
          this->log_WARNING_HI_GP_ReadError(this->m_gpio,stat);
          return;
      } else {
          state = val?true:false;
      }
  }

  void LinuxGpioDriverComponentImpl ::
    gpioWrite_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
      FW_ASSERT(this->m_fd != -1);

      NATIVE_INT_TYPE stat;

      stat = gpio_set_value(this->m_fd,state?1:0);

      if (0 != stat) {
          this->log_WARNING_HI_GP_WriteError(this->m_gpio,stat);
          return;
      }
  }

  bool LinuxGpioDriverComponentImpl ::
    open(NATIVE_INT_TYPE gpio, GpioDirection direction) {

      // TODO check for invalid gpio?
      NATIVE_INT_TYPE stat;

      // Configure:
      stat = gpio_export(gpio);
      if (-1 == stat) {
          this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
      }
      stat = gpio_set_dir(gpio, direction == GPIO_OUT ? 1 : 0);
      if (-1 == stat) {
          this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
      }

      // If needed, set edge to rising in intTaskEntry()

      // Open:
      this->m_fd = gpio_fd_open(gpio);
      if (-1 == this->m_fd) {
          this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
      } else {
          this->m_gpio = gpio;
      }

      return true;
  }

  void *gpio_int_isr(DSPAL_GPIO_INT_ISR_CTX context)
  {
      void * val = (void *)context;

      LinuxGpioDriverComponentImpl::intTaskEntry(val);

      return NULL;
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

      // Configure this GPIO device as interrupt source
      struct dspal_gpio_ioctl_reg_int int_config = {
        .trigger = DSPAL_GPIOINT_TRIGGER_RISING,
        .isr = (DSPAL_GPIO_INT_ISR) &gpio_int_isr,
        .isr_ctx = (DSPAL_GPIO_INT_ISR_CTX) this,
      };

      if (ioctl(this->m_fd, DSPAL_GPIO_IOCTL_CONFIG_REG_INT, (void *)&int_config) != 0) {
          DEBUG_PRINT("error: ioctl DSPAL_GPIO_IOCTL_CONFIG_REG_INT failed\n");
          return Os::Task::TASK_UNKNOWN_ERROR;
      }

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
