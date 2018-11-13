// ====================================================================== 
// \title  ShortLogQueueImpl.cpp
// \author tcanham
// \brief  cpp file for ShortLogQueue component implementation class
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


#include <LLProc/ShortLogQueue/ShortLogQueueComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/Assert.hpp>

namespace LLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ShortLogQueueComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ShortLogQueueComponentImpl(
        const char *const compName
    ) :
      ShortLogQueueComponentBase(compName)
#else
    ShortLogQueueImpl(void)
#endif
    ,m_head(0)
    ,m_tail(0)
  {

  }

  void ShortLogQueueComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    ShortLogQueueComponentBase::init(instance);
  }

  ShortLogQueueComponentImpl ::
    ~ShortLogQueueComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ShortLogQueueComponentImpl ::
    LogRecv_handler(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
      // assert if we would overrun
      FW_ASSERT((this->m_head+1)%SHORT_LOG_QUEUE_DEPTH != this->m_tail);
      // store the event at the head of the queue
      this->m_logQueue[this->m_head].id = id;
      this->m_logQueue[this->m_head].severity = static_cast<U8>(severity);
      this->m_logQueue[this->m_head].args = args;
      // increment the head to the next record
      this->m_head = (this->m_head + 1)%SHORT_LOG_QUEUE_DEPTH;
  }

  void ShortLogQueueComponentImpl ::
    Run_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      // if present, take an event from the tail of the queue and send it
      if (this->m_tail != this->m_head) {
          this->LogSend_out(0,
                  this->m_logQueue[this->m_tail].id,
                  static_cast<Fw::LogSeverity>(this->m_logQueue[this->m_tail].severity),
                  this->m_logQueue[this->m_tail].args);
          // increment the tail to the next record
          this->m_tail = (this->m_tail + 1)%SHORT_LOG_QUEUE_DEPTH;
      }
  }

} // end namespace LLProc
