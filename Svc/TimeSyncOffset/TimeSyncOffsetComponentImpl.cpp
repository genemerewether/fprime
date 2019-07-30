// ======================================================================
// \title  TimeSyncOffsetComponentImpl.cpp
// \author kedelson
// \brief  cpp file for TimeSyncOffset component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Svc/TimeSyncOffset/TimeSyncOffsetComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  TimeSyncOffsetComponentImpl ::
#if FW_OBJECT_NAMES == 1
    TimeSyncOffsetComponentImpl(
        const char *const compName
    ) :
      TimeSyncOffsetComponentBase(compName)
#else
    TimeSyncOffsetComponentImpl(void)
#endif
  {

  }

  void TimeSyncOffsetComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    TimeSyncOffsetComponentBase::init(queueDepth, instance);
  }

  TimeSyncOffsetComponentImpl ::
    ~TimeSyncOffsetComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetComponentImpl ::
    SchedIn_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
    num_sched_calls++;    
    if (num_sched_calls >= sched_timeout)
    {
      // if timeout, send warning event
      this->log_WARNING_LO_SchedIn_Timeout(num_sched_calls);
    }
    this->GPIOPulse_out(0, true);
    HLTime = this->getTime();
    this->GPIOPulse_out(0, false);
  }

  void TimeSyncOffsetComponentImpl ::
    LLTime_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &LLTime
    )
  {
    // TODO

    // reset sched call counter
    num_sched_calls = 0;

    // // should i just rename this above???
    // Fw::Time LLTime = time;

    // output HL and LL times
    this->ClockTimes_out(0, LLTime, HLTime);

    // // compute times in F64 for tlm channels
    // U32 LLsec = LLTime.getSeconds();
    // U32 LLusec = LLTime.getUSeconds();

    // U32 HLsec = HLTime.getSeconds();
    // U32 HLusec = HLTime.getUSeconds();

    double LLTimeF64 = (double)LLTime.getSeconds() + (double)LLTime.getUSeconds() * 1.0e-6;
    double HLTimeF64 = (double)HLTime.getSeconds() + (double)HLTime.getUSeconds() * 1.0e-6;

    // send tlm message, takes F64
    this->tlmWrite_LLTime(LLTimeF64);
    this->tlmWrite_HLTime(HLTimeF64);

    // reset gpio pulse high
    this->GPIOPulse_out(0, true);
  }

} // end namespace Svc
