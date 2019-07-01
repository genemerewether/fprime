// ====================================================================== 
// \title  R5TimeForwardImpl.cpp
// \author kubiak
// \brief  cpp file for R5TimeForward component implementation class
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


#include <R5/R5TimeForward/R5TimeForwardComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  R5TimeForwardComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5TimeForwardComponentImpl(
        const char *const compName
    ) :
      R5TimeForwardComponentBase(compName)
#else
    R5TimeForwardImpl(void)
#endif
  {

  }

  void R5TimeForwardComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    R5TimeForwardComponentBase::init(instance);
  }

  R5TimeForwardComponentImpl ::
    ~R5TimeForwardComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5TimeForwardComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    Fw::Time eventTime;

    do {
        this->GetEventTime_out(0, eventTime);

        if (eventTime.getTimeBase() != TB_NONE) {
            // Valid event
            this->SendEventTime_out(0, eventTime);
        }
    } while (eventTime.getTimeBase() != TB_NONE);
  }

} // end namespace R5
