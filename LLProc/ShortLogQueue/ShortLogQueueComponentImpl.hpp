// ====================================================================== 
// \title  ShortLogQueueImpl.hpp
// \author tcanham
// \brief  hpp file for ShortLogQueue component implementation class
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

#ifndef ShortLogQueue_HPP
#define ShortLogQueue_HPP

#include "LLProc/ShortLogQueue/ShortLogQueueComponentAc.hpp"
#include "LLProc/ShortLogQueue/ShortLogQueueComponentImplCfg.hpp"

namespace LLProc {

  class ShortLogQueueComponentImpl :
    public ShortLogQueueComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ShortLogQueue
      //!
      ShortLogQueueComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ShortLogQueue
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ShortLogQueue
      //!
      ~ShortLogQueueComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for LogRecv
      //!
      void LogRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::LogSeverity severity, /*!< The severity argument*/
          Fw::LogBuffer &args /*!< Buffer containing serialized log entry*/
      );

      //! Handler implementation for Run
      //!
      void Run_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Log Queue record

      struct LogQueueRecord {
          FwEventIdType id;
          U8 severity;
          Fw::LogBuffer args;
      } m_logQueue[SHORT_LOG_QUEUE_DEPTH];

      NATIVE_UINT_TYPE m_head; //!< current head of circular buffer
      NATIVE_UINT_TYPE m_tail; //!< current tail of circular buffer

    };

} // end namespace LLProc

#endif
