// ======================================================================
// \title  ShortLogQueue/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for ShortLogQueue component test harness base class
//
// \copyright
// Copyright 2009-2016, by the California Institute of Technology.
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

#include <stdlib.h>
#include <string.h>
#include "TesterBase.hpp"

namespace LLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ShortLogQueueTesterBase ::
    ShortLogQueueTesterBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) :
#if FW_OBJECT_NAMES == 1
      Fw::PassiveComponentBase(compName)
#else
      Fw::PassiveComponentBase()
#endif
  {
    // Initialize histories for typed user output ports
    this->fromPortHistory_LogSend =
      new History<FromPortEntry_LogSend>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  ShortLogQueueTesterBase ::
    ~ShortLogQueueTesterBase(void) 
  {
  }

  void ShortLogQueueTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port LogSend

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_LogSend();
        ++_port
    ) {

      this->m_from_LogSend[_port].init();
      this->m_from_LogSend[_port].addCallComp(
          this,
          from_LogSend_static
      );
      this->m_from_LogSend[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_LogSend[%d]",
          this->m_objName,
          _port
      );
      this->m_from_LogSend[_port].setObjName(_portName);
#endif

    }

    // Initialize output port LogRecv

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_LogRecv();
        ++_port
    ) {
      this->m_to_LogRecv[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_LogRecv[%d]",
          this->m_objName,
          _port
      );
      this->m_to_LogRecv[_port].setObjName(_portName);
#endif

    }

    // Initialize output port Run

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_Run();
        ++_port
    ) {
      this->m_to_Run[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_Run[%d]",
          this->m_objName,
          _port
      );
      this->m_to_Run[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE ShortLogQueueTesterBase ::
    getNum_to_LogRecv(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_LogRecv);
  }

  NATIVE_INT_TYPE ShortLogQueueTesterBase ::
    getNum_from_LogSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogSend);
  }

  NATIVE_INT_TYPE ShortLogQueueTesterBase ::
    getNum_to_Run(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Run);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void ShortLogQueueTesterBase ::
    connect_to_LogRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::InputLogPort *const LogRecv
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_LogRecv(),static_cast<AssertArg>(portNum));
    this->m_to_LogRecv[portNum].addCallPort(LogRecv);
  }

  void ShortLogQueueTesterBase ::
    connect_to_Run(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const Run
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Run(),static_cast<AssertArg>(portNum));
    this->m_to_Run[portNum].addCallPort(Run);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void ShortLogQueueTesterBase ::
    invoke_to_LogRecv(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(portNum < this->getNum_to_LogRecv(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_LogRecv(),static_cast<AssertArg>(portNum));
    this->m_to_LogRecv[portNum].invoke(
        id, timeTag, severity, args
    );
  }

  void ShortLogQueueTesterBase ::
    invoke_to_Run(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_Run(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_Run(),static_cast<AssertArg>(portNum));
    this->m_to_Run[portNum].invoke(
        context
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool ShortLogQueueTesterBase ::
    isConnected_to_LogRecv(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_LogRecv(), static_cast<AssertArg>(portNum));
    return this->m_to_LogRecv[portNum].isConnected();
  }

  bool ShortLogQueueTesterBase ::
    isConnected_to_Run(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Run(), static_cast<AssertArg>(portNum));
    return this->m_to_Run[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Common::InputShortLogPort *ShortLogQueueTesterBase ::
    get_from_LogSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogSend(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogSend[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void ShortLogQueueTesterBase ::
    from_LogSend_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        U8 severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(callComp);
    ShortLogQueueTesterBase* _testerBase = 
      static_cast<ShortLogQueueTesterBase*>(callComp);
    _testerBase->from_LogSend_handlerBase(
        portNum,
        id, severity, args
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void ShortLogQueueTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_LogSend->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: LogSend
  // ---------------------------------------------------------------------- 

  void ShortLogQueueTesterBase ::
    pushFromPortEntry_LogSend(
        FwEventIdType id,
        U8 severity,
        Fw::LogBuffer &args
    )
  {
    FromPortEntry_LogSend _e = {
      id, severity, args
    };
    this->fromPortHistory_LogSend->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void ShortLogQueueTesterBase ::
    from_LogSend_handlerBase(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        U8 severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(portNum < this->getNum_from_LogSend(),static_cast<AssertArg>(portNum));
    this->from_LogSend_handler(
        portNum,
        id, severity, args
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void ShortLogQueueTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace LLProc
