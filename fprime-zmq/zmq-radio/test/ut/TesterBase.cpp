// ======================================================================
// \title  ZmqRadio/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for ZmqRadio component test harness base class
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

namespace Zmq {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ZmqRadioTesterBase ::
    ZmqRadioTesterBase(
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
    // Initialize telemetry histories
    this->tlmHistory_ZR_NumDisconnects = 
      new History<TlmEntry_ZR_NumDisconnects>(maxHistorySize);
    this->tlmHistory_ZR_NumConnects = 
      new History<TlmEntry_ZR_NumConnects>(maxHistorySize);
    this->tlmHistory_ZR_NumDisconnectRetries = 
      new History<TlmEntry_ZR_NumDisconnectRetries>(maxHistorySize);
    this->tlmHistory_ZR_PktsSent = 
      new History<TlmEntry_ZR_PktsSent>(maxHistorySize);
    this->tlmHistory_ZR_PktsRecv = 
      new History<TlmEntry_ZR_PktsRecv>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_ZR_ContextError =
      new History<EventEntry_ZR_ContextError>(maxHistorySize);
    this->eventHistory_ZR_SocketError =
      new History<EventEntry_ZR_SocketError>(maxHistorySize);
    this->eventHistory_ZR_BindError =
      new History<EventEntry_ZR_BindError>(maxHistorySize);
    this->eventHistory_ZR_SendError =
      new History<EventEntry_ZR_SendError>(maxHistorySize);
    this->eventHistory_ZR_ReceiveError =
      new History<EventEntry_ZR_ReceiveError>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_fileUplinkBufferSendOut =
      new History<FromPortEntry_fileUplinkBufferSendOut>(maxHistorySize);
    this->fromPortHistory_uplinkPort =
      new History<FromPortEntry_uplinkPort>(maxHistorySize);
    this->fromPortHistory_fileDownlinkBufferSendOut =
      new History<FromPortEntry_fileDownlinkBufferSendOut>(maxHistorySize);
    this->fromPortHistory_fileUplinkBufferGet =
      new History<FromPortEntry_fileUplinkBufferGet>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  ZmqRadioTesterBase ::
    ~ZmqRadioTesterBase(void) 
  {
    // Destroy telemetry histories
    delete this->tlmHistory_ZR_NumDisconnects;
    delete this->tlmHistory_ZR_NumConnects;
    delete this->tlmHistory_ZR_NumDisconnectRetries;
    delete this->tlmHistory_ZR_PktsSent;
    delete this->tlmHistory_ZR_PktsRecv;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_ZR_ContextError;
    delete this->eventHistory_ZR_SocketError;
    delete this->eventHistory_ZR_BindError;
    delete this->eventHistory_ZR_SendError;
    delete this->eventHistory_ZR_ReceiveError;
  }

  void ZmqRadioTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port fileUplinkBufferSendOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_fileUplinkBufferSendOut();
        ++_port
    ) {

      this->m_from_fileUplinkBufferSendOut[_port].init();
      this->m_from_fileUplinkBufferSendOut[_port].addCallComp(
          this,
          from_fileUplinkBufferSendOut_static
      );
      this->m_from_fileUplinkBufferSendOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_fileUplinkBufferSendOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_fileUplinkBufferSendOut[_port].setObjName(_portName);
#endif

    }

    // Attach input port Log

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_Log();
        ++_port
    ) {

      this->m_from_Log[_port].init();
      this->m_from_Log[_port].addCallComp(
          this,
          from_Log_static
      );
      this->m_from_Log[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_Log[%d]",
          this->m_objName,
          _port
      );
      this->m_from_Log[_port].setObjName(_portName);
#endif

    }

    // Attach input port LogText

#if FW_ENABLE_TEXT_LOGGING == 1
    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_LogText();
        ++_port
    ) {

      this->m_from_LogText[_port].init();
      this->m_from_LogText[_port].addCallComp(
          this,
          from_LogText_static
      );
      this->m_from_LogText[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_LogText[%d]",
          this->m_objName,
          _port
      );
      this->m_from_LogText[_port].setObjName(_portName);
#endif

    }
#endif

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

    // Attach input port uplinkPort

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_uplinkPort();
        ++_port
    ) {

      this->m_from_uplinkPort[_port].init();
      this->m_from_uplinkPort[_port].addCallComp(
          this,
          from_uplinkPort_static
      );
      this->m_from_uplinkPort[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_uplinkPort[%d]",
          this->m_objName,
          _port
      );
      this->m_from_uplinkPort[_port].setObjName(_portName);
#endif

    }

    // Attach input port fileDownlinkBufferSendOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_fileDownlinkBufferSendOut();
        ++_port
    ) {

      this->m_from_fileDownlinkBufferSendOut[_port].init();
      this->m_from_fileDownlinkBufferSendOut[_port].addCallComp(
          this,
          from_fileDownlinkBufferSendOut_static
      );
      this->m_from_fileDownlinkBufferSendOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_fileDownlinkBufferSendOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_fileDownlinkBufferSendOut[_port].setObjName(_portName);
#endif

    }

    // Attach input port fileUplinkBufferGet

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_fileUplinkBufferGet();
        ++_port
    ) {

      this->m_from_fileUplinkBufferGet[_port].init();
      this->m_from_fileUplinkBufferGet[_port].addCallComp(
          this,
          from_fileUplinkBufferGet_static
      );
      this->m_from_fileUplinkBufferGet[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_fileUplinkBufferGet[%d]",
          this->m_objName,
          _port
      );
      this->m_from_fileUplinkBufferGet[_port].setObjName(_portName);
#endif

    }

    // Attach input port tlmOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_tlmOut();
        ++_port
    ) {

      this->m_from_tlmOut[_port].init();
      this->m_from_tlmOut[_port].addCallComp(
          this,
          from_tlmOut_static
      );
      this->m_from_tlmOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_tlmOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_tlmOut[_port].setObjName(_portName);
#endif

    }

    // Initialize output port reconnect

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_reconnect();
        ++_port
    ) {
      this->m_to_reconnect[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_reconnect[%d]",
          this->m_objName,
          _port
      );
      this->m_to_reconnect[_port].setObjName(_portName);
#endif

    }

    // Initialize output port downlinkPort

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_downlinkPort();
        ++_port
    ) {
      this->m_to_downlinkPort[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_downlinkPort[%d]",
          this->m_objName,
          _port
      );
      this->m_to_downlinkPort[_port].setObjName(_portName);
#endif

    }

    // Initialize output port fileDownlinkBufferSendIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_fileDownlinkBufferSendIn();
        ++_port
    ) {
      this->m_to_fileDownlinkBufferSendIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_fileDownlinkBufferSendIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_fileDownlinkBufferSendIn[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_fileUplinkBufferSendOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_fileUplinkBufferSendOut);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_to_reconnect(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_reconnect);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_to_downlinkPort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_downlinkPort);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_uplinkPort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_uplinkPort);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_fileDownlinkBufferSendOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_fileDownlinkBufferSendOut);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_to_fileDownlinkBufferSendIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_fileDownlinkBufferSendIn);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_fileUplinkBufferGet(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_fileUplinkBufferGet);
  }

  NATIVE_INT_TYPE ZmqRadioTesterBase ::
    getNum_from_tlmOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_tlmOut);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    connect_to_reconnect(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const reconnect
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_reconnect(),static_cast<AssertArg>(portNum));
    this->m_to_reconnect[portNum].addCallPort(reconnect);
  }

  void ZmqRadioTesterBase ::
    connect_to_downlinkPort(
        const NATIVE_INT_TYPE portNum,
        Fw::InputComPort *const downlinkPort
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_downlinkPort(),static_cast<AssertArg>(portNum));
    this->m_to_downlinkPort[portNum].addCallPort(downlinkPort);
  }

  void ZmqRadioTesterBase ::
    connect_to_fileDownlinkBufferSendIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputBufferSendPort *const fileDownlinkBufferSendIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_fileDownlinkBufferSendIn(),static_cast<AssertArg>(portNum));
    this->m_to_fileDownlinkBufferSendIn[portNum].addCallPort(fileDownlinkBufferSendIn);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    invoke_to_reconnect(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_reconnect(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_reconnect(),static_cast<AssertArg>(portNum));
    this->m_to_reconnect[portNum].invoke(
        context
    );
  }

  void ZmqRadioTesterBase ::
    invoke_to_downlinkPort(
        const NATIVE_INT_TYPE portNum,
        Fw::ComBuffer &data,
        U32 context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_downlinkPort(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_downlinkPort(),static_cast<AssertArg>(portNum));
    this->m_to_downlinkPort[portNum].invoke(
        data, context
    );
  }

  void ZmqRadioTesterBase ::
    invoke_to_fileDownlinkBufferSendIn(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_fileDownlinkBufferSendIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_fileDownlinkBufferSendIn(),static_cast<AssertArg>(portNum));
    this->m_to_fileDownlinkBufferSendIn[portNum].invoke(
        fwBuffer
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool ZmqRadioTesterBase ::
    isConnected_to_reconnect(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_reconnect(), static_cast<AssertArg>(portNum));
    return this->m_to_reconnect[portNum].isConnected();
  }

  bool ZmqRadioTesterBase ::
    isConnected_to_downlinkPort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_downlinkPort(), static_cast<AssertArg>(portNum));
    return this->m_to_downlinkPort[portNum].isConnected();
  }

  bool ZmqRadioTesterBase ::
    isConnected_to_fileDownlinkBufferSendIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_fileDownlinkBufferSendIn(), static_cast<AssertArg>(portNum));
    return this->m_to_fileDownlinkBufferSendIn[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputBufferSendPort *ZmqRadioTesterBase ::
    get_from_fileUplinkBufferSendOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_fileUplinkBufferSendOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_fileUplinkBufferSendOut[portNum];
  }

  Fw::InputLogPort *ZmqRadioTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *ZmqRadioTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  Fw::InputTimePort *ZmqRadioTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputComPort *ZmqRadioTesterBase ::
    get_from_uplinkPort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_uplinkPort(),static_cast<AssertArg>(portNum));
    return &this->m_from_uplinkPort[portNum];
  }

  Fw::InputBufferSendPort *ZmqRadioTesterBase ::
    get_from_fileDownlinkBufferSendOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_fileDownlinkBufferSendOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_fileDownlinkBufferSendOut[portNum];
  }

  Fw::InputBufferGetPort *ZmqRadioTesterBase ::
    get_from_fileUplinkBufferGet(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_fileUplinkBufferGet(),static_cast<AssertArg>(portNum));
    return &this->m_from_fileUplinkBufferGet[portNum];
  }

  Fw::InputTlmPort *ZmqRadioTesterBase ::
    get_from_tlmOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_tlmOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_tlmOut[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    from_fileUplinkBufferSendOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(callComp);
    ZmqRadioTesterBase* _testerBase = 
      static_cast<ZmqRadioTesterBase*>(callComp);
    _testerBase->from_fileUplinkBufferSendOut_handlerBase(
        portNum,
        fwBuffer
    );
  }

  void ZmqRadioTesterBase ::
    from_uplinkPort_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::ComBuffer &data,
        U32 context
    )
  {
    FW_ASSERT(callComp);
    ZmqRadioTesterBase* _testerBase = 
      static_cast<ZmqRadioTesterBase*>(callComp);
    _testerBase->from_uplinkPort_handlerBase(
        portNum,
        data, context
    );
  }

  void ZmqRadioTesterBase ::
    from_fileDownlinkBufferSendOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(callComp);
    ZmqRadioTesterBase* _testerBase = 
      static_cast<ZmqRadioTesterBase*>(callComp);
    _testerBase->from_fileDownlinkBufferSendOut_handlerBase(
        portNum,
        fwBuffer
    );
  }

  Fw::Buffer ZmqRadioTesterBase ::
    from_fileUplinkBufferGet_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    FW_ASSERT(callComp);
    ZmqRadioTesterBase* _testerBase = 
      static_cast<ZmqRadioTesterBase*>(callComp);
    return _testerBase->from_fileUplinkBufferGet_handlerBase(
        portNum,
        size
    );
  }

  void ZmqRadioTesterBase ::
    from_tlmOut_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    ZmqRadioTesterBase* _testerBase =
      static_cast<ZmqRadioTesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void ZmqRadioTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    ZmqRadioTesterBase* _testerBase =
      static_cast<ZmqRadioTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void ZmqRadioTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    ZmqRadioTesterBase* _testerBase =
      static_cast<ZmqRadioTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void ZmqRadioTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    ZmqRadioTesterBase* _testerBase =
      static_cast<ZmqRadioTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_fileUplinkBufferSendOut->clear();
    this->fromPortHistory_uplinkPort->clear();
    this->fromPortHistory_fileDownlinkBufferSendOut->clear();
    this->fromPortHistory_fileUplinkBufferGet->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: fileUplinkBufferSendOut
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    pushFromPortEntry_fileUplinkBufferSendOut(
        Fw::Buffer fwBuffer
    )
  {
    FromPortEntry_fileUplinkBufferSendOut _e = {
      fwBuffer
    };
    this->fromPortHistory_fileUplinkBufferSendOut->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: uplinkPort
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    pushFromPortEntry_uplinkPort(
        Fw::ComBuffer &data,
        U32 context
    )
  {
    FromPortEntry_uplinkPort _e = {
      data, context
    };
    this->fromPortHistory_uplinkPort->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: fileDownlinkBufferSendOut
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    pushFromPortEntry_fileDownlinkBufferSendOut(
        Fw::Buffer fwBuffer
    )
  {
    FromPortEntry_fileDownlinkBufferSendOut _e = {
      fwBuffer
    };
    this->fromPortHistory_fileDownlinkBufferSendOut->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: fileUplinkBufferGet
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    pushFromPortEntry_fileUplinkBufferGet(
        U32 size
    )
  {
    FromPortEntry_fileUplinkBufferGet _e = {
      size
    };
    this->fromPortHistory_fileUplinkBufferGet->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    from_fileUplinkBufferSendOut_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_fileUplinkBufferSendOut(),static_cast<AssertArg>(portNum));
    this->from_fileUplinkBufferSendOut_handler(
        portNum,
        fwBuffer
    );
  }

  void ZmqRadioTesterBase ::
    from_uplinkPort_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::ComBuffer &data,
        U32 context
    )
  {
    FW_ASSERT(portNum < this->getNum_from_uplinkPort(),static_cast<AssertArg>(portNum));
    this->from_uplinkPort_handler(
        portNum,
        data, context
    );
  }

  void ZmqRadioTesterBase ::
    from_fileDownlinkBufferSendOut_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_fileDownlinkBufferSendOut(),static_cast<AssertArg>(portNum));
    this->from_fileDownlinkBufferSendOut_handler(
        portNum,
        fwBuffer
    );
  }

  Fw::Buffer ZmqRadioTesterBase ::
    from_fileUplinkBufferGet_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    FW_ASSERT(portNum < this->getNum_from_fileUplinkBufferGet(),static_cast<AssertArg>(portNum));
    return this->from_fileUplinkBufferGet_handler(
        portNum,
        size
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    clearHistory()
  {
    this->clearTlm();
    this->textLogHistory->clear();
    this->clearEvents();
    this->clearFromPortHistory();
  }

  // ----------------------------------------------------------------------
  // Time
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    dispatchTlm(
        const FwChanIdType id,
        const Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {

    val.resetDeser();

    const U32 idBase = this->getIdBase();
    FW_ASSERT(id >= idBase, id, idBase);

    switch (id - idBase) {

      case ZmqRadioComponentBase::CHANNELID_ZR_NUMDISCONNECTS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing ZR_NumDisconnects: %d\n", _status);
          return;
        }
        this->tlmInput_ZR_NumDisconnects(timeTag, arg);
        break;
      }

      case ZmqRadioComponentBase::CHANNELID_ZR_NUMCONNECTS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing ZR_NumConnects: %d\n", _status);
          return;
        }
        this->tlmInput_ZR_NumConnects(timeTag, arg);
        break;
      }

      case ZmqRadioComponentBase::CHANNELID_ZR_NUMDISCONNECTRETRIES:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing ZR_NumDisconnectRetries: %d\n", _status);
          return;
        }
        this->tlmInput_ZR_NumDisconnectRetries(timeTag, arg);
        break;
      }

      case ZmqRadioComponentBase::CHANNELID_ZR_PKTSSENT:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing ZR_PktsSent: %d\n", _status);
          return;
        }
        this->tlmInput_ZR_PktsSent(timeTag, arg);
        break;
      }

      case ZmqRadioComponentBase::CHANNELID_ZR_PKTSRECV:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing ZR_PktsRecv: %d\n", _status);
          return;
        }
        this->tlmInput_ZR_PktsRecv(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void ZmqRadioTesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_ZR_NumDisconnects->clear();
    this->tlmHistory_ZR_NumConnects->clear();
    this->tlmHistory_ZR_NumDisconnectRetries->clear();
    this->tlmHistory_ZR_PktsSent->clear();
    this->tlmHistory_ZR_PktsRecv->clear();
  }

  // ---------------------------------------------------------------------- 
  // Channel: ZR_NumDisconnects
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    tlmInput_ZR_NumDisconnects(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_ZR_NumDisconnects e = { timeTag, val };
    this->tlmHistory_ZR_NumDisconnects->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: ZR_NumConnects
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    tlmInput_ZR_NumConnects(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_ZR_NumConnects e = { timeTag, val };
    this->tlmHistory_ZR_NumConnects->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: ZR_NumDisconnectRetries
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    tlmInput_ZR_NumDisconnectRetries(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_ZR_NumDisconnectRetries e = { timeTag, val };
    this->tlmHistory_ZR_NumDisconnectRetries->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: ZR_PktsSent
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    tlmInput_ZR_PktsSent(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_ZR_PktsSent e = { timeTag, val };
    this->tlmHistory_ZR_PktsSent->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: ZR_PktsRecv
  // ---------------------------------------------------------------------- 

  void ZmqRadioTesterBase ::
    tlmInput_ZR_PktsRecv(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_ZR_PktsRecv e = { timeTag, val };
    this->tlmHistory_ZR_PktsRecv->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    dispatchEvents(
        const FwEventIdType id,
        Fw::Time &timeTag,
        const Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {

    args.resetDeser();

    const U32 idBase = this->getIdBase();
    FW_ASSERT(id >= idBase, id, idBase);
    switch (id - idBase) {

      case ZmqRadioComponentBase::EVENTID_ZR_CONTEXTERROR: 
      {

        Fw::SerializeStatus _status = Fw::FW_SERIALIZE_OK;
#if FW_AMPCS_COMPATIBLE
        // Deserialize the number of arguments.
        U8 _numArgs;
        _status = args.deserialize(_numArgs);
        FW_ASSERT(
          _status == Fw::FW_SERIALIZE_OK,
          static_cast<AssertArg>(_status)
        );
        // verify they match expected.
        FW_ASSERT(_numArgs == 1,_numArgs,1);
        
#endif    
        Fw::LogStringArg error;
        _status = args.deserialize(error);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_ZR_ContextError(error);

        break;

      }

      case ZmqRadioComponentBase::EVENTID_ZR_SOCKETERROR: 
      {

        Fw::SerializeStatus _status = Fw::FW_SERIALIZE_OK;
#if FW_AMPCS_COMPATIBLE
        // Deserialize the number of arguments.
        U8 _numArgs;
        _status = args.deserialize(_numArgs);
        FW_ASSERT(
          _status == Fw::FW_SERIALIZE_OK,
          static_cast<AssertArg>(_status)
        );
        // verify they match expected.
        FW_ASSERT(_numArgs == 1,_numArgs,1);
        
#endif    
        Fw::LogStringArg error;
        _status = args.deserialize(error);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_ZR_SocketError(error);

        break;

      }

      case ZmqRadioComponentBase::EVENTID_ZR_BINDERROR: 
      {

        Fw::SerializeStatus _status = Fw::FW_SERIALIZE_OK;
#if FW_AMPCS_COMPATIBLE
        // Deserialize the number of arguments.
        U8 _numArgs;
        _status = args.deserialize(_numArgs);
        FW_ASSERT(
          _status == Fw::FW_SERIALIZE_OK,
          static_cast<AssertArg>(_status)
        );
        // verify they match expected.
        FW_ASSERT(_numArgs == 1,_numArgs,1);
        
#endif    
        Fw::LogStringArg error;
        _status = args.deserialize(error);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_ZR_BindError(error);

        break;

      }

      case ZmqRadioComponentBase::EVENTID_ZR_DISCONNECTION: 
      {

#if FW_AMPCS_COMPATIBLE
        // For AMPCS, decode zero arguments
        Fw::SerializeStatus _zero_status = Fw::FW_SERIALIZE_OK;
        U8 _noArgs;
        _zero_status = args.deserialize(_noArgs);
        FW_ASSERT(
            _zero_status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_zero_status)
        );
#endif    
        this->logIn_WARNING_HI_ZR_Disconnection();

        break;

      }

      case ZmqRadioComponentBase::EVENTID_ZR_CONNECTION: 
      {

#if FW_AMPCS_COMPATIBLE
        // For AMPCS, decode zero arguments
        Fw::SerializeStatus _zero_status = Fw::FW_SERIALIZE_OK;
        U8 _noArgs;
        _zero_status = args.deserialize(_noArgs);
        FW_ASSERT(
            _zero_status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_zero_status)
        );
#endif    
        this->logIn_ACTIVITY_HI_ZR_Connection();

        break;

      }

      case ZmqRadioComponentBase::EVENTID_ZR_SENDERROR: 
      {

        Fw::SerializeStatus _status = Fw::FW_SERIALIZE_OK;
#if FW_AMPCS_COMPATIBLE
        // Deserialize the number of arguments.
        U8 _numArgs;
        _status = args.deserialize(_numArgs);
        FW_ASSERT(
          _status == Fw::FW_SERIALIZE_OK,
          static_cast<AssertArg>(_status)
        );
        // verify they match expected.
        FW_ASSERT(_numArgs == 1,_numArgs,1);
        
#endif    
        Fw::LogStringArg error;
        _status = args.deserialize(error);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_ZR_SendError(error);

        break;

      }

      case ZmqRadioComponentBase::EVENTID_ZR_RECEIVEERROR: 
      {

        Fw::SerializeStatus _status = Fw::FW_SERIALIZE_OK;
#if FW_AMPCS_COMPATIBLE
        // Deserialize the number of arguments.
        U8 _numArgs;
        _status = args.deserialize(_numArgs);
        FW_ASSERT(
          _status == Fw::FW_SERIALIZE_OK,
          static_cast<AssertArg>(_status)
        );
        // verify they match expected.
        FW_ASSERT(_numArgs == 1,_numArgs,1);
        
#endif    
        Fw::LogStringArg error;
        _status = args.deserialize(error);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_ZR_ReceiveError(error);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void ZmqRadioTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_ZR_ContextError->clear();
    this->eventHistory_ZR_SocketError->clear();
    this->eventHistory_ZR_BindError->clear();
    this->eventsSize_ZR_Disconnection = 0;
    this->eventsSize_ZR_Connection = 0;
    this->eventHistory_ZR_SendError->clear();
    this->eventHistory_ZR_ReceiveError->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    textLogIn(
        const U32 id,
        Fw::Time &timeTag,
        const Fw::TextLogSeverity severity,
        const Fw::TextLogString &text
    )
  {
    TextLogEntry e = { id, timeTag, severity, text };
    textLogHistory->push_back(e);
  }

  void ZmqRadioTesterBase ::
    printTextLogHistoryEntry(
        const TextLogEntry& e,
        FILE* file
    )
  {
    const char *severityString = "UNKNOWN";
    switch (e.severity) {
      case Fw::LOG_FATAL:
        severityString = "FATAL";
        break;
      case Fw::LOG_WARNING_HI:
        severityString = "WARNING_HI";
        break;
      case Fw::LOG_WARNING_LO:
        severityString = "WARNING_LO";
        break;
      case Fw::LOG_COMMAND:
        severityString = "COMMAND";
        break;
      case Fw::LOG_ACTIVITY_HI:
        severityString = "ACTIVITY_HI";
        break;
      case Fw::LOG_ACTIVITY_LO:
        severityString = "ACTIVITY_LO";
        break;
      case Fw::LOG_DIAGNOSTIC:
       severityString = "DIAGNOSTIC";
        break;
      default:
        severityString = "SEVERITY ERROR";
        break;
    }

    fprintf(
        file,
        "EVENT: (%d) (%d:%d,%d) %s: %s\n",
        e.id,
        const_cast<TextLogEntry&>(e).timeTag.getTimeBase(),
        const_cast<TextLogEntry&>(e).timeTag.getSeconds(),
        const_cast<TextLogEntry&>(e).timeTag.getUSeconds(),
        severityString,
        e.text.toChar()
    );

  }

  void ZmqRadioTesterBase ::
    printTextLogHistory(FILE *file) 
  {
    for (U32 i = 0; i < this->textLogHistory->size(); ++i) {
      this->printTextLogHistoryEntry(
          this->textLogHistory->at(i), 
          file
      );
    }
  }

#endif

  // ----------------------------------------------------------------------
  // Event: ZR_ContextError 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    logIn_WARNING_HI_ZR_ContextError(
        Fw::LogStringArg& error
    )
  {
    EventEntry_ZR_ContextError e = {
      error
    };
    eventHistory_ZR_ContextError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: ZR_SocketError 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    logIn_WARNING_HI_ZR_SocketError(
        Fw::LogStringArg& error
    )
  {
    EventEntry_ZR_SocketError e = {
      error
    };
    eventHistory_ZR_SocketError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: ZR_BindError 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    logIn_WARNING_HI_ZR_BindError(
        Fw::LogStringArg& error
    )
  {
    EventEntry_ZR_BindError e = {
      error
    };
    eventHistory_ZR_BindError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: ZR_Disconnection 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    logIn_WARNING_HI_ZR_Disconnection(
        void
    )
  {
    ++this->eventsSize_ZR_Disconnection;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: ZR_Connection 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    logIn_ACTIVITY_HI_ZR_Connection(
        void
    )
  {
    ++this->eventsSize_ZR_Connection;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: ZR_SendError 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    logIn_WARNING_HI_ZR_SendError(
        Fw::LogStringArg& error
    )
  {
    EventEntry_ZR_SendError e = {
      error
    };
    eventHistory_ZR_SendError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: ZR_ReceiveError 
  // ----------------------------------------------------------------------

  void ZmqRadioTesterBase ::
    logIn_WARNING_HI_ZR_ReceiveError(
        Fw::LogStringArg& error
    )
  {
    EventEntry_ZR_ReceiveError e = {
      error
    };
    eventHistory_ZR_ReceiveError->push_back(e);
    ++this->eventsSize;
  }

} // end namespace Zmq
