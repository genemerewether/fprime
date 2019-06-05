// ====================================================================== 
// \title  ActuatorControlsImpl.cpp
// \author mereweth
// \brief  cpp file for ActuatorControls component implementation class
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


#include <Drv/Mavlink/ActuatorControls/ActuatorControlsComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#define MAVLINK_NO_CONVERSION_HELPERS

#ifdef BUILD_TIR5

#ifndef __GNUC__
#define __GNUC__
#include <Drv/Mavlink/c_library_v2/mavlink_types.h>
#undef __GNUC__
#endif //__GNUC__

#else //BUILD_TIR5

#include <Drv/Mavlink/c_library_v2/mavlink_types.h>

#endif //BUILD_TIR5


extern mavlink_system_t mavlink_system;

#include <Drv/Mavlink/c_library_v2/standard/mavlink.h>

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ActuatorControlsComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ActuatorControlsComponentImpl(
        const char *const compName
    ) :
      ActuatorControlsComponentBase(compName)
#else
    ActuatorControlsImpl(void)
#endif
  {

  }

  void ActuatorControlsComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    ActuatorControlsComponentBase::init(instance);
  }

  ActuatorControlsComponentImpl ::
    ~ActuatorControlsComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ActuatorControlsComponentImpl ::
    SerReadPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        SerialReadStatus &status
    )
  {
    // TODO
  }
  
  void ActuatorControlsComponentImpl ::
    pwmSetDuty_handler(
        const NATIVE_INT_TYPE portNum,
        PwmSetDutyCycle pwmSetDutyCycle
    )
  {
    // TODO (mereweth) - store duty cycle commands
  }

  void ActuatorControlsComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO (mereweth) - assemble mavlink message and send on uart port
  }

} // end namespace Drv
