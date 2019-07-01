// ====================================================================== 
// \title  R5EventCaptureImpl.cpp
// \author kubiak
// \brief  cpp file for R5EventCapture component implementation class
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


#include <R5/R5EventCapture/R5EventCaptureImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <R5/R5EventCapture/R5EventCapture.hpp>

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  R5EventCaptureComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5EventCaptureComponentImpl(
        const char *const compName
    ) :
      R5EventCaptureComponentBase(compName)
#else
    R5EventCaptureImpl(void)
#endif
  {

  }

  void R5EventCaptureComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    R5EventCaptureComponentBase::init(instance);
  }

  void R5EventCaptureComponentImpl ::
    initDriver(
        const NATIVE_UINT_TYPE memSize,
        Fw::MemAllocator& allocator
    )
  {
    U8* memAPtr = 0;
    U8* memBPtr = 0;

    memAPtr = static_cast<U8*>(allocator.allocate(0, memSize));
    memBPtr = static_cast<U8*>(allocator.allocate(0, memSize));

    FW_ASSERT(memAPtr != NULL);
    FW_ASSERT(memBPtr != NULL);

    EventCaptureInit(memAPtr, memBPtr, memSize);
  }

  R5EventCaptureComponentImpl ::
    ~R5EventCaptureComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5EventCaptureComponentImpl ::
    time_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    ECEvent event;
    bool status;

    status = EventCaptureGetEvent(event);

    if (status) {
        time.set(TB_PROC_TIME, event.seconds, event.useconds);
    } else {
        time.set(TB_NONE, 0, 0);
    }
  }

} // end namespace 
