// ====================================================================== 
// \title  EventExpanderImpl.cpp
// \author tcanham
// \brief  cpp file for EventExpander component implementation class
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


#include <HLProc/EventExpander/EventExpanderComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace HLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  EventExpanderComponentImpl ::
#if FW_OBJECT_NAMES == 1
    EventExpanderComponentImpl(
        const char *const compName
    ) :
      EventExpanderComponentBase(compName)
#else
    EventExpanderImpl(void)
#endif
  {

  }

  void EventExpanderComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    EventExpanderComponentBase::init(instance);
  }

  EventExpanderComponentImpl ::
    ~EventExpanderComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void EventExpanderComponentImpl ::
    LogRecv_handler(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        U8 severity,
        Fw::LogBuffer &args
    )
  {
      // Take shortened event and convert it to full-sized event
      Fw::Time time;
      // get current time for event
      this->Time_out(0,time);
      // forward event
      this->LogSend_out(0,id,time,static_cast<Fw::LogSeverity>(severity),args);
  }

} // end namespace HLProc
