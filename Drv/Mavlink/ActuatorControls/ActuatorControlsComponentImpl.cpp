// ======================================================================
// \title  ActuatorControlsComponentImpl.cpp
// \author mereweth
// \brief  cpp file for ActuatorControls component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
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
    ActuatorControlsComponentImpl(void)
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
    prmTrigger_handler(
        const NATIVE_INT_TYPE portNum,
        FwPrmIdType dummy
    )
  {
      this->loadParameters();
  }
 
  void ActuatorControlsComponentImpl ::
    rateThrust_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::RateThrust &RateThrust
    )
  {
    // TODO
  }

  void ActuatorControlsComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void ActuatorControlsComponentImpl ::
    SerReadPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        SerialReadStatus &status
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void ActuatorControlsComponentImpl ::
    ACTCTRL_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

} // end namespace Drv
