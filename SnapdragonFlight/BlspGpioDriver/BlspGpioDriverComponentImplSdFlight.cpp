// ====================================================================== 
// \title  BlspGpioDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspGpioDriver component implementation class
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


#include <SnapdragonFlight/BlspGpioDriver/BlspGpioDriverComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <SnapdragonFlight/DspRelay/dsp_relay.h>
#include <Os/TaskString.hpp>

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void BlspGpioDriverComponentImpl ::
    gpioRead_handler(
        const NATIVE_INT_TYPE portNum,
        bool &state
    )
  {
      FW_ASSERT(this->m_fd != -1);
      NATIVE_INT_TYPE val = dsp_relay_gpio_relay_read(this->m_fd);
      if (-1 == val) {
          this->log_WARNING_HI_GP_ReadError(this->m_gpio,val);
          return;
      } else {
          state = val?true:false;
      }

  }

  void BlspGpioDriverComponentImpl ::
    gpioWrite_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
      FW_ASSERT(this->m_fd != -1);


      NATIVE_INT_TYPE stat = dsp_relay_gpio_relay_write(this->m_fd,state?1:0);

      if (0 != stat) {
          this->log_WARNING_HI_GP_WriteError(this->m_gpio,stat);
          return;
      }
  }

  bool BlspGpioDriverComponentImpl ::
    open(NATIVE_INT_TYPE gpio, GpioDirection direction) {
      this->m_fd = dsp_relay_gpio_relay_open(gpio);
      if (-1 == this->m_fd) {
          this->log_WARNING_HI_GP_OpenError(gpio,this->m_fd);
          return false;
      } else {
          this->m_gpio = gpio;
      }

      // set direction
      int dir = 0;
      switch (direction) {
          case GPIO_IN:
              dir = 0;
              break;
          case GPIO_OUT:
              dir = 1;
              break;
          case GPIO_INT:
              dir = 2;
              break;
          default:
              FW_ASSERT(0,direction);
              break;
      }
      NATIVE_INT_TYPE stat = dsp_relay_gpio_relay_configure(this->m_fd,dir);
      if (-1 == stat) {
          this->log_WARNING_HI_GP_OpenError(gpio,stat);
          return false;
      }

      return true;
  }
  //! Entry point for task waiting for RTI
  void BlspGpioDriverComponentImpl ::
    intTaskEntry(void * ptr) {

      FW_ASSERT(ptr);
      BlspGpioDriverComponentImpl* compPtr = (BlspGpioDriverComponentImpl*) ptr;

      FW_ASSERT(compPtr->m_fd != -1);

      // start GPIO interrupt
      NATIVE_INT_TYPE stat = dsp_relay_gpio_relay_start_int(compPtr->m_gpio,compPtr->m_fd);
      if (-1 == stat) {
          compPtr->log_WARNING_HI_GP_IntStartError(compPtr->m_gpio);
          return;
      }

      // spin waiting for interrupt
      while(not compPtr->m_quitThread) {
          // wait for interrupt from DSP
          stat = dsp_relay_gpio_relay_isr_wait(compPtr->m_gpio,compPtr->m_fd);
          if (-1 == stat) {
              compPtr->log_WARNING_HI_GP_IntWaitError(compPtr->m_gpio);
              return;
          } else if (1 == stat) {
              // check for timeout
              continue;
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

  }

  Os::Task::TaskStatus BlspGpioDriverComponentImpl ::
    startIntTask(NATIVE_INT_TYPE priority, NATIVE_INT_TYPE cpuAffinity) {
      Os::TaskString name;
      name.format("GPINT_%s",this->getObjName()); // The task name can only be 16 chars including null
      Os::Task::TaskStatus stat = this->m_intTask.start(name,0,priority,20*1024,BlspGpioDriverComponentImpl::intTaskEntry,this,cpuAffinity);

      if (stat != Os::Task::TASK_OK) {
          printf("Task start error: %d\n",stat);
      }

      return stat;

  }

  void BlspGpioDriverComponentImpl ::
    exitThread(void) {
      dsp_relay_gpio_relay_quit(this->m_gpio);
      this->m_quitThread = true;
  }

  void BlspGpioDriverComponentImpl ::
    joinThread(void **value_ptr) {
      Os::Task::TaskStatus stat = this->m_intTask.join(value_ptr);

      if (stat != Os::Task::TASK_OK) {
          printf("Task join error: %d\n",stat);
      }
  }


  BlspGpioDriverComponentImpl ::
    ~BlspGpioDriverComponentImpl(void)
  {
      if (this->m_fd != -1) {
          dsp_relay_gpio_relay_close(this->m_fd);
      }

  }


} // end namespace SnapdragonFlight
