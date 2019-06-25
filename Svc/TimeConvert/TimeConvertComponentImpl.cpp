// ======================================================================
// \title  TimeConvertComponentImpl.cpp
// \author kedelson
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

    if {time1.getTimeBase() == WS_TB}
    {
      int idx1 = 0;
    } else if {time1.getTimeBase() == PROC_TB}
    {
      int idx1 = 1;
    } else if {time1.getTimeBase() == ROS_TB}
    {
      int idx1 = 2;
    } else {
      // error out or ignore this
    }

    if {time2.getTimeBase() == WS_TB}
    {
      int idx1 = 0;
    } else if {time2.getTimeBase() == PROC_TB}
    {
      int idx1 = 1;
    } else if {time2.getTimeBase() == ROS_TB}
    {
      int idx1 = 2;
    } else {
      // error out or ignore this
    }

    // adjacency graph
    // 0 - WS_TB, 1 - PROC_TB, 2 - ROS_TB
    // row = from, col = to
    // Graph[idx_from][idx_to]
    // from + edge_weight = to
    Graph[idx1][idx2] = time2 - time1;
    Graph[idx2][idx1] = time1 - time2;

  }

  Fw::Time TimeConvertComponentImpl ::
    ConvertTime_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time time,
        TimeBase timeBase,
        FwTimeContextStoreType timeContext,
        bool& success
    )
  {
    // TODO

    if {time.getTimeBase() == WS_TB}
    {
      int idx_from = 0;
    } else if {time.getTimeBase() == PROC_TB}
    {
      int idx_from = 1;
    } else if {time.getTimeBase() == ROS_TB}
    {
      int idx_from = 2;
    } else {
      // error out or ignore this
    }

    if {timeBase == WS_TB}
    {
      int idx_to = 0;
    } else if {timeBase == PROC_TB}
    {
      int idx_to = 1;
    } else if {timeBase == ROS_TB}
    {
      int idx_to = 2;
    } else {
      // error out or ignore this
    }
    
    ConvertedTime = time + Graph[idx_from][idx_to];
    // ConvertedTime.setTimeBase(timeBase);
    // ConvertedTime.setTimeContext(timeContext);
    this->Time_out(0, ConvertedTime);
    // what do i do with the success flag????
  }

} // end namespace Svc
