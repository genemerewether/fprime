// ====================================================================== 
// \title  R5TimeImpl.cpp
// \author tcanham
// \brief  cpp file for R5Time component implementation class
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

#include <R5/R5Time/R5TimeComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <R5/TiHal/include/HL_rti.h>

static const U32 R5_USEC_DIV = 75; // R5 subseconds clock is running at 75Mhz, so divide by 75 to get usec

namespace R5 {

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5TimeComponentImpl ::
    timeGetPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
      RtiTime rTime;
      rtiGetHighResTime(&rTime);
      time.set(TB_WORKSTATION_TIME,rTime.seconds,rTime.subseconds/R5_USEC_DIV);
  }

  void R5TimeComponentImpl ::
    timeSetPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
      rtiSetBothCounters(time.getSeconds(),time.getUSeconds());
  }

  void R5TimeComponentImpl ::
    timeStartPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::TimeAction action
    )
  {
    switch (action) {
        case Fw::START_CLOCK:
            rtiStartBothCounters();
            break;
        case Fw::STOP_CLOCK:
            rtiStopCounter(rtiREG1, rtiCOUNTER_BLOCK0);
            rtiStopCounter(rtiREG1, rtiCOUNTER_BLOCK1);
            break;
        default:
            FW_ASSERT(0,action);
            break;
    }
  }

} // end namespace R5
