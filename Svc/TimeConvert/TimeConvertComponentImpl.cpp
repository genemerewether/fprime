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

#include <stdio.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

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
    U32 TimeBase1 = time1.getTimeBase();
    U32 TimeBase2 = time2.getTimeBase();
    DEBUG_PRINT("TimeConvert add edge from %d to %d\n", TimeBase1, TimeBase2);

    I64 time1_usec = (I64)time1.getSeconds() * 1000LL * 1000LL + (I64)time1.getUSeconds();
    I64 time2_usec = (I64)time2.getSeconds() * 1000LL * 1000LL + (I64)time2.getUSeconds();
 
    // adjacency graph
    // Graph[from][to]
    // from + edge_weight = to
    adjGraph[TimeBase1][TimeBase2] = time2_usec - time1_usec;
    adjGraph[TimeBase2][TimeBase1] = time1_usec - time2_usec;
    boolAdjGraph[TimeBase1][TimeBase2] = true;
    boolAdjGraph[TimeBase2][TimeBase1] = true;

    // TODO(mereweth) - non brute-force method for finding paths, support length longer than 2 hops
    for (int ii = 0; ii < FW_NUM_ARRAY_ELEMENTS(adjGraph); ii++) {
        for (int jj = 0; jj < FW_NUM_ARRAY_ELEMENTS(adjGraph); jj++) {
            for (int kk = 0; kk < FW_NUM_ARRAY_ELEMENTS(adjGraph[0]); kk++) {
                if (boolAdjGraph[ii][jj] && boolAdjGraph[jj][kk] && !boolAdjGraph[ii][kk]) {
                    boolAdjGraph[ii][kk] = true;
                    boolAdjGraph[kk][ii] = true;
                    adjGraph[ii][kk] = adjGraph[ii][jj] + adjGraph[jj][kk];
                    adjGraph[kk][ii] = -adjGraph[ii][kk];
                }
            }
        }
    }
  }

  Fw::Time TimeConvertComponentImpl ::
    ConvertTime_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time time,
        U32 timeBase,
        U32 timeContext,
        bool& success
    )
  {

    U32 fromTimeBase = time.getTimeBase();

    if (fromTimeBase >= FW_NUM_ARRAY_ELEMENTS(adjGraph))
    {
      success = false;
      DEBUG_PRINT("TimeConvert fromTimeBase %d too big\n", fromTimeBase);
      // TODO: event to user
      return Fw::Time();
    }

    if (timeBase >= FW_NUM_ARRAY_ELEMENTS(adjGraph[0]))
    {
      success = false;
      DEBUG_PRINT("TimeConvert timeBase %d too big\n", timeBase);
      // TODO: event to user
      return Fw::Time();
    }

    I64 time_usec = (I64)time.getSeconds() * 1000LL * 1000LL + (I64)time.getUSeconds();

    if (boolAdjGraph[fromTimeBase][timeBase])
    {
      I64 convertedTime_usec = time_usec + adjGraph[fromTimeBase][timeBase];
      Fw::Time convertedTime = Fw::Time((TimeBase)timeBase, (FwTimeContextStoreType)timeContext, 
                                (U32)(convertedTime_usec / 1000LL / 1000LL), 
                                (U32)(convertedTime_usec % (1000LL * 1000LL)));
      success = true;
      return convertedTime;
    }
    else 
    {
      DEBUG_PRINT("TimeConvert no edge from %d to %d\n", fromTimeBase, timeBase);
      success = false;
      // TODO: event to user
      return Fw::Time();
    }
  
  }

} // end namespace Svc
