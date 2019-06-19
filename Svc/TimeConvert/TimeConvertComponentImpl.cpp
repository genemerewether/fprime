// ======================================================================
// \title  TimeConvertComponentImpl.cpp
// \author mereweth
// \brief  cpp file for TimeConvert component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Svc/TimeConvert/TimeConvertComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  TimeConvertComponentImpl ::
#if FW_OBJECT_NAMES == 1
    TimeConvertComponentImpl(
        const char *const compName
    ) :
      TimeConvertComponentBase(compName)
#else
    TimeConvertComponentImpl(void)
#endif
  {

  }

  void TimeConvertComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    TimeConvertComponentBase::init(instance);
  }

  TimeConvertComponentImpl ::
    ~TimeConvertComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void TimeConvertComponentImpl ::
    SchedIn_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void TimeConvertComponentImpl ::
    ClockTimes_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time time1,
        Fw::Time time2
    )
  {
    // TODO
  }

  Fw::Time TimeConvertComponentImpl ::
    ConvertTime_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time time,
        U32 timeBase,
        U32 timeContext
    )
  {
    // TODO
  }

} // end namespace Svc
