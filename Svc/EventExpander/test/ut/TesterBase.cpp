// ======================================================================
// \title  EventExpander/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for EventExpander component test harness base class
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

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  EventExpanderTesterBase ::
    EventExpanderTesterBase(
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
    this->fromPortHistory_Time =
      new History<FromPortEntry_Time>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  EventExpanderTesterBase ::
    ~EventExpanderTesterBase(void) 
  {
  }

  void EventExpanderTesterBase ::
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

    // Attach input port Time

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_Time();
        ++_port
    ) {

      this->m_from_Time[_port].init();
      this->m_from_Time[_port].addCallComp(
          this,
          from_Time_static
      );
      this->m_from_Time[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_Time[%d]",
          this->m_objName,
          _port
      );
      this->m_from_Time[_port].setObjName(_portName);
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

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE EventExpanderTesterBase ::
    getNum_to_LogRecv(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_LogRecv);
  }

  NATIVE_INT_TYPE EventExpanderTesterBase ::
    getNum_from_LogSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogSend);
  }

  NATIVE_INT_TYPE EventExpanderTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void EventExpanderTesterBase ::
    connect_to_LogRecv(
        const NATIVE_INT_TYPE portNum,
        Common::InputShortLogPort *const LogRecv
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_LogRecv(),static_cast<AssertArg>(portNum));
    this->m_to_LogRecv[portNum].addCallPort(LogRecv);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void EventExpanderTesterBase ::
    invoke_to_LogRecv(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        U8 severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(portNum < this->getNum_to_LogRecv(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_LogRecv(),static_cast<AssertArg>(portNum));
    this->m_to_LogRecv[portNum].invoke(
        id, severity, args
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool EventExpanderTesterBase ::
    isConnected_to_LogRecv(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_LogRecv(), static_cast<AssertArg>(portNum));
    return this->m_to_LogRecv[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputLogPort *EventExpanderTesterBase ::
    get_from_LogSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogSend(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogSend[portNum];
  }

  Fw::InputTimePort *EventExpanderTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void EventExpanderTesterBase ::
    from_LogSend_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(callComp);
    EventExpanderTesterBase* _testerBase = 
      static_cast<EventExpanderTesterBase*>(callComp);
    _testerBase->from_LogSend_handlerBase(
        portNum,
        id, timeTag, severity, args
    );
  }

  void EventExpanderTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(callComp);
    EventExpanderTesterBase* _testerBase = 
      static_cast<EventExpanderTesterBase*>(callComp);
    _testerBase->from_Time_handlerBase(
        portNum,
        time
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void EventExpanderTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_LogSend->clear();
    this->fromPortHistory_Time->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: LogSend
  // ---------------------------------------------------------------------- 

  void EventExpanderTesterBase ::
    pushFromPortEntry_LogSend(
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    FromPortEntry_LogSend _e = {
      id, timeTag, severity, args
    };
    this->fromPortHistory_LogSend->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: Time
  // ---------------------------------------------------------------------- 

  void EventExpanderTesterBase ::
    pushFromPortEntry_Time(
        Fw::Time &time
    )
  {
    FromPortEntry_Time _e = {
      time
    };
    this->fromPortHistory_Time->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void EventExpanderTesterBase ::
    from_LogSend_handlerBase(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(portNum < this->getNum_from_LogSend(),static_cast<AssertArg>(portNum));
    this->from_LogSend_handler(
        portNum,
        id, timeTag, severity, args
    );
  }

  void EventExpanderTesterBase ::
    from_Time_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    this->from_Time_handler(
        portNum,
        time
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void EventExpanderTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace Svc
