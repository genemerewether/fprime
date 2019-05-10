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
#include "Fw/Types/BasicTypes.hpp"

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
    // TODO
  }

  void BlspGpioDriverComponentImpl ::
    gpioWrite_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
    // TODO
  }

  bool BlspGpioDriverComponentImpl ::
    open(NATIVE_INT_TYPE gpio, GpioDirection direction) {
      return false;
  }

  Os::Task::TaskStatus BlspGpioDriverComponentImpl ::
     startIntTask(NATIVE_INT_TYPE priority, NATIVE_INT_TYPE cpuAffinity) {
     return Os::Task::TASK_OK;
   }

  void BlspGpioDriverComponentImpl ::
    exitThread(void) {
  }
  
  void BlspGpioDriverComponentImpl ::
    joinThread(void **value_ptr) {
  }
  
  BlspGpioDriverComponentImpl ::
    ~BlspGpioDriverComponentImpl(void)
  {

  }


} // end namespace SnapdragonFlight
