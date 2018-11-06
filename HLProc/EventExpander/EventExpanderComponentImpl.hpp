// ====================================================================== 
// \title  EventExpanderImpl.hpp
// \author tcanham
// \brief  hpp file for EventExpander component implementation class
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

#ifndef EventExpander_HPP
#define EventExpander_HPP

#include "HLProc/EventExpander/EventExpanderComponentAc.hpp"

namespace HLProc {

  class EventExpanderComponentImpl :
    public EventExpanderComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object EventExpander
      //!
      EventExpanderComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object EventExpander
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object EventExpander
      //!
      ~EventExpanderComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for LogRecv
      //!
      void LogRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          U8 severity, /*!< The shorter severity argument*/
          Fw::LogBuffer &args /*!< Buffer containing serialized log entry*/
      );


    };

} // end namespace HLProc

#endif
