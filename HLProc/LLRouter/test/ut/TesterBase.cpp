// ======================================================================
// \title  LLRouter/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for LLRouter component test harness base class
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

namespace HLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  LLRouterTesterBase ::
    LLRouterTesterBase(
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
    // Initialize command history
    this->cmdResponseHistory = new History<CmdResponse>(maxHistorySize);
    // Initialize telemetry histories
    this->tlmHistory_LLR_CrcErrors = 
      new History<TlmEntry_LLR_CrcErrors>(maxHistorySize);
    this->tlmHistory_LLR_ResyncErrors = 
      new History<TlmEntry_LLR_ResyncErrors>(maxHistorySize);
    this->tlmHistory_LLR_NumPackets = 
      new History<TlmEntry_LLR_NumPackets>(maxHistorySize);
    this->tlmHistory_LLR_NumSerialErrors = 
      new History<TlmEntry_LLR_NumSerialErrors>(maxHistorySize);
    this->tlmHistory_LLR_NumGoodPackets = 
      new History<TlmEntry_LLR_NumGoodPackets>(maxHistorySize);
    this->tlmHistory_LLR_NumDecodeErrors = 
      new History<TlmEntry_LLR_NumDecodeErrors>(maxHistorySize);
    this->tlmHistory_LLR_NumInvalidPorts = 
      new History<TlmEntry_LLR_NumInvalidPorts>(maxHistorySize);
    this->tlmHistory_LLR_NumBadSerialPortCalls = 
      new History<TlmEntry_LLR_NumBadSerialPortCalls>(maxHistorySize);
    this->tlmHistory_LLR_NumOuputBufferOverflows = 
      new History<TlmEntry_LLR_NumOuputBufferOverflows>(maxHistorySize);
    this->tlmHistory_LLR_NumZeroPktSize = 
      new History<TlmEntry_LLR_NumZeroPktSize>(maxHistorySize);
    this->tlmHistory_LLR_MaxCycleTime = 
      new History<TlmEntry_LLR_MaxCycleTime>(maxHistorySize);
    this->tlmHistory_LLR_CycleTime = 
      new History<TlmEntry_LLR_CycleTime>(maxHistorySize);
    this->tlmHistory_LLR_PatchProgress = 
      new History<TlmEntry_LLR_PatchProgress>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_LLR_DataReceiveError =
      new History<EventEntry_LLR_DataReceiveError>(maxHistorySize);
    this->eventHistory_LLR_DataCrcError =
      new History<EventEntry_LLR_DataCrcError>(maxHistorySize);
    this->eventHistory_LLR_InvalidPortNum =
      new History<EventEntry_LLR_InvalidPortNum>(maxHistorySize);
    this->eventHistory_LLR_BadSerialPortCall =
      new History<EventEntry_LLR_BadSerialPortCall>(maxHistorySize);
    this->eventHistory_LLR_PatchFileOpen =
      new History<EventEntry_LLR_PatchFileOpen>(maxHistorySize);
    this->eventHistory_LLR_PatchModeChange =
      new History<EventEntry_LLR_PatchModeChange>(maxHistorySize);
    this->eventHistory_LLR_PatchEvent =
      new History<EventEntry_LLR_PatchEvent>(maxHistorySize);
    this->eventHistory_LLR_PatchError =
      new History<EventEntry_LLR_PatchError>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_SerWritePort =
      new History<FromPortEntry_SerWritePort>(maxHistorySize);
    this->fromPortHistory_SerialBufferSend =
      new History<FromPortEntry_SerialBufferSend>(maxHistorySize);
    this->fromPortHistory_pingOut =
      new History<FromPortEntry_pingOut>(maxHistorySize);
    this->fromPortHistory_PolyGet =
      new History<FromPortEntry_PolyGet>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  LLRouterTesterBase ::
    ~LLRouterTesterBase(void) 
  {
    // Destroy command history
    delete this->cmdResponseHistory;
    // Destroy telemetry histories
    delete this->tlmHistory_LLR_CrcErrors;
    delete this->tlmHistory_LLR_ResyncErrors;
    delete this->tlmHistory_LLR_NumPackets;
    delete this->tlmHistory_LLR_NumSerialErrors;
    delete this->tlmHistory_LLR_NumGoodPackets;
    delete this->tlmHistory_LLR_NumDecodeErrors;
    delete this->tlmHistory_LLR_NumInvalidPorts;
    delete this->tlmHistory_LLR_NumBadSerialPortCalls;
    delete this->tlmHistory_LLR_NumOuputBufferOverflows;
    delete this->tlmHistory_LLR_NumZeroPktSize;
    delete this->tlmHistory_LLR_MaxCycleTime;
    delete this->tlmHistory_LLR_CycleTime;
    delete this->tlmHistory_LLR_PatchProgress;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_LLR_DataReceiveError;
    delete this->eventHistory_LLR_DataCrcError;
    delete this->eventHistory_LLR_InvalidPortNum;
    delete this->eventHistory_LLR_BadSerialPortCall;
    delete this->eventHistory_LLR_PatchFileOpen;
    delete this->eventHistory_LLR_PatchModeChange;
    delete this->eventHistory_LLR_PatchEvent;
    delete this->eventHistory_LLR_PatchError;
  }

  void LLRouterTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port LLPortsOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_LLPortsOut();
        ++_port
    ) {

      this->m_from_LLPortsOut[_port].init();
      this->m_from_LLPortsOut[_port].addCallComp(
          this,
          from_LLPortsOut_static
      );
      this->m_from_LLPortsOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_LLPortsOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_LLPortsOut[_port].setObjName(_portName);
#endif

    }

    // Attach input port SerWritePort

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_SerWritePort();
        ++_port
    ) {

      this->m_from_SerWritePort[_port].init();
      this->m_from_SerWritePort[_port].addCallComp(
          this,
          from_SerWritePort_static
      );
      this->m_from_SerWritePort[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_SerWritePort[%d]",
          this->m_objName,
          _port
      );
      this->m_from_SerWritePort[_port].setObjName(_portName);
#endif

    }

    // Attach input port SerialBufferSend

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_SerialBufferSend();
        ++_port
    ) {

      this->m_from_SerialBufferSend[_port].init();
      this->m_from_SerialBufferSend[_port].addCallComp(
          this,
          from_SerialBufferSend_static
      );
      this->m_from_SerialBufferSend[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_SerialBufferSend[%d]",
          this->m_objName,
          _port
      );
      this->m_from_SerialBufferSend[_port].setObjName(_portName);
#endif

    }

    // Attach input port pingOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_pingOut();
        ++_port
    ) {

      this->m_from_pingOut[_port].init();
      this->m_from_pingOut[_port].addCallComp(
          this,
          from_pingOut_static
      );
      this->m_from_pingOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_pingOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_pingOut[_port].setObjName(_portName);
#endif

    }

    // Attach input port PolyGet

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_PolyGet();
        ++_port
    ) {

      this->m_from_PolyGet[_port].init();
      this->m_from_PolyGet[_port].addCallComp(
          this,
          from_PolyGet_static
      );
      this->m_from_PolyGet[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_PolyGet[%d]",
          this->m_objName,
          _port
      );
      this->m_from_PolyGet[_port].setObjName(_portName);
#endif

    }

    // Attach input port CmdStatus

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_CmdStatus();
        ++_port
    ) {

      this->m_from_CmdStatus[_port].init();
      this->m_from_CmdStatus[_port].addCallComp(
          this,
          from_CmdStatus_static
      );
      this->m_from_CmdStatus[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_CmdStatus[%d]",
          this->m_objName,
          _port
      );
      this->m_from_CmdStatus[_port].setObjName(_portName);
#endif

    }

    // Attach input port CmdReg

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_CmdReg();
        ++_port
    ) {

      this->m_from_CmdReg[_port].init();
      this->m_from_CmdReg[_port].addCallComp(
          this,
          from_CmdReg_static
      );
      this->m_from_CmdReg[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_CmdReg[%d]",
          this->m_objName,
          _port
      );
      this->m_from_CmdReg[_port].setObjName(_portName);
#endif

    }

    // Attach input port Tlm

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_Tlm();
        ++_port
    ) {

      this->m_from_Tlm[_port].init();
      this->m_from_Tlm[_port].addCallComp(
          this,
          from_Tlm_static
      );
      this->m_from_Tlm[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_Tlm[%d]",
          this->m_objName,
          _port
      );
      this->m_from_Tlm[_port].setObjName(_portName);
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

    // Initialize output port HLPortsIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_HLPortsIn();
        ++_port
    ) {
      this->m_to_HLPortsIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_HLPortsIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_HLPortsIn[_port].setObjName(_portName);
#endif

    }

    // Initialize output port SerReadPort

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_SerReadPort();
        ++_port
    ) {
      this->m_to_SerReadPort[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_SerReadPort[%d]",
          this->m_objName,
          _port
      );
      this->m_to_SerReadPort[_port].setObjName(_portName);
#endif

    }

    // Initialize output port Sched

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_Sched();
        ++_port
    ) {
      this->m_to_Sched[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_Sched[%d]",
          this->m_objName,
          _port
      );
      this->m_to_Sched[_port].setObjName(_portName);
#endif

    }

    // Initialize output port CycleDone

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_CycleDone();
        ++_port
    ) {
      this->m_to_CycleDone[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_CycleDone[%d]",
          this->m_objName,
          _port
      );
      this->m_to_CycleDone[_port].setObjName(_portName);
#endif

    }

    // Initialize output port pingIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_pingIn();
        ++_port
    ) {
      this->m_to_pingIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_pingIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_pingIn[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_to_HLPortsIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_HLPortsIn);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_LLPortsOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LLPortsOut);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_to_SerReadPort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SerReadPort);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_SerWritePort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SerWritePort);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_to_Sched(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Sched);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_to_CycleDone(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_CycleDone);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_SerialBufferSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SerialBufferSend);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_to_pingIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_pingIn);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_pingOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_pingOut);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_PolyGet(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_PolyGet);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_to_CmdDisp(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_CmdDisp);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_CmdStatus(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdStatus);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_CmdReg(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdReg);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE LLRouterTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    connect_to_HLPortsIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputSerializePort *const HLPortsIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_HLPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_HLPortsIn[portNum].registerSerialPort(HLPortsIn);
  }

  void LLRouterTesterBase ::
    connect_to_SerReadPort(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSerialReadPort *const SerReadPort
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(),static_cast<AssertArg>(portNum));
    this->m_to_SerReadPort[portNum].addCallPort(SerReadPort);
  }

  void LLRouterTesterBase ::
    connect_to_Sched(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const Sched
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(),static_cast<AssertArg>(portNum));
    this->m_to_Sched[portNum].addCallPort(Sched);
  }

  void LLRouterTesterBase ::
    connect_to_CycleDone(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const CycleDone
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_CycleDone(),static_cast<AssertArg>(portNum));
    this->m_to_CycleDone[portNum].addCallPort(CycleDone);
  }

  void LLRouterTesterBase ::
    connect_to_pingIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputPingPort *const pingIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(),static_cast<AssertArg>(portNum));
    this->m_to_pingIn[portNum].addCallPort(pingIn);
  }

  void LLRouterTesterBase ::
    connect_to_CmdDisp(
        const NATIVE_INT_TYPE portNum,
        Fw::InputCmdPort *const CmdDisp
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_CmdDisp(),static_cast<AssertArg>(portNum));
    this->m_to_CmdDisp[portNum].addCallPort(CmdDisp);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    invoke_to_HLPortsIn(
      NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_HLPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_HLPortsIn[portNum].invokeSerial(Buffer);
  }

  void LLRouterTesterBase ::
    invoke_to_SerReadPort(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        Drv::SerialReadStatus &status
    )
  {
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(),static_cast<AssertArg>(portNum));
    this->m_to_SerReadPort[portNum].invoke(
        serBuffer, status
    );
  }

  void LLRouterTesterBase ::
    invoke_to_Sched(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_Sched(),static_cast<AssertArg>(portNum));
    this->m_to_Sched[portNum].invoke(
        context
    );
  }

  void LLRouterTesterBase ::
    invoke_to_CycleDone(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_CycleDone(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_CycleDone(),static_cast<AssertArg>(portNum));
    this->m_to_CycleDone[portNum].invoke(
        context
    );
  }

  void LLRouterTesterBase ::
    invoke_to_pingIn(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_pingIn(),static_cast<AssertArg>(portNum));
    this->m_to_pingIn[portNum].invoke(
        key
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool LLRouterTesterBase ::
    isConnected_to_HLPortsIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_HLPortsIn(), static_cast<AssertArg>(portNum));
    return this->m_to_HLPortsIn[portNum].isConnected();
  }

  bool LLRouterTesterBase ::
    isConnected_to_SerReadPort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(), static_cast<AssertArg>(portNum));
    return this->m_to_SerReadPort[portNum].isConnected();
  }

  bool LLRouterTesterBase ::
    isConnected_to_Sched(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(), static_cast<AssertArg>(portNum));
    return this->m_to_Sched[portNum].isConnected();
  }

  bool LLRouterTesterBase ::
    isConnected_to_CycleDone(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_CycleDone(), static_cast<AssertArg>(portNum));
    return this->m_to_CycleDone[portNum].isConnected();
  }

  bool LLRouterTesterBase ::
    isConnected_to_pingIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(), static_cast<AssertArg>(portNum));
    return this->m_to_pingIn[portNum].isConnected();
  }

  bool LLRouterTesterBase ::
    isConnected_to_CmdDisp(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_CmdDisp(), static_cast<AssertArg>(portNum));
    return this->m_to_CmdDisp[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputSerializePort *LLRouterTesterBase ::
    get_from_LLPortsOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LLPortsOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_LLPortsOut[portNum];
  }

  Drv::InputSerialWritePort *LLRouterTesterBase ::
    get_from_SerWritePort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SerWritePort(),static_cast<AssertArg>(portNum));
    return &this->m_from_SerWritePort[portNum];
  }

  Fw::InputBufferSendPort *LLRouterTesterBase ::
    get_from_SerialBufferSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SerialBufferSend(),static_cast<AssertArg>(portNum));
    return &this->m_from_SerialBufferSend[portNum];
  }

  Svc::InputPingPort *LLRouterTesterBase ::
    get_from_pingOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_pingOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_pingOut[portNum];
  }

  Svc::InputPolyPort *LLRouterTesterBase ::
    get_from_PolyGet(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_PolyGet(),static_cast<AssertArg>(portNum));
    return &this->m_from_PolyGet[portNum];
  }

  Fw::InputCmdResponsePort *LLRouterTesterBase ::
    get_from_CmdStatus(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdStatus(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdStatus[portNum];
  }

  Fw::InputCmdRegPort *LLRouterTesterBase ::
    get_from_CmdReg(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdReg(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdReg[portNum];
  }

  Fw::InputTlmPort *LLRouterTesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputTimePort *LLRouterTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *LLRouterTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *LLRouterTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    from_SerWritePort_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    FW_ASSERT(callComp);
    LLRouterTesterBase* _testerBase = 
      static_cast<LLRouterTesterBase*>(callComp);
    _testerBase->from_SerWritePort_handlerBase(
        portNum,
        serBuffer
    );
  }

  void LLRouterTesterBase ::
    from_SerialBufferSend_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(callComp);
    LLRouterTesterBase* _testerBase = 
      static_cast<LLRouterTesterBase*>(callComp);
    _testerBase->from_SerialBufferSend_handlerBase(
        portNum,
        fwBuffer
    );
  }

  void LLRouterTesterBase ::
    from_pingOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    FW_ASSERT(callComp);
    LLRouterTesterBase* _testerBase = 
      static_cast<LLRouterTesterBase*>(callComp);
    _testerBase->from_pingOut_handlerBase(
        portNum,
        key
    );
  }

  void LLRouterTesterBase ::
    from_PolyGet_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 entry,
        Svc::MeasurementStatus &status,
        Fw::Time &time,
        Fw::PolyType &val
    )
  {
    FW_ASSERT(callComp);
    LLRouterTesterBase* _testerBase = 
      static_cast<LLRouterTesterBase*>(callComp);
    _testerBase->from_PolyGet_handlerBase(
        portNum,
        entry, status, time, val
    );
  }

  void LLRouterTesterBase ::
    from_LLPortsOut_static(
      Fw::PassiveComponentBase *const callComp, //!< The component instance
      const NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer //!< serialized data buffer
    )
  {
    FW_ASSERT(callComp);
    LLRouterTesterBase* _testerBase = 
      static_cast<LLRouterTesterBase*>(callComp);

    _testerBase->from_LLPortsOut_handlerBase(
        portNum,
        Buffer
    );
  }  

  void LLRouterTesterBase ::
    from_LLPortsOut_handlerBase(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    FW_ASSERT(portNum < this->getNum_from_LLPortsOut(),static_cast<AssertArg>(portNum));
    this->from_LLPortsOut_handler(
        portNum,
        Buffer
    );
  } 
   
  void LLRouterTesterBase ::
    from_CmdStatus_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
  {
    LLRouterTesterBase* _testerBase =
      static_cast<LLRouterTesterBase*>(component);
    _testerBase->cmdResponseIn(opCode, cmdSeq, response);
  }

  void LLRouterTesterBase ::
    from_CmdReg_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode
    )
  {

  }

  void LLRouterTesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    LLRouterTesterBase* _testerBase =
      static_cast<LLRouterTesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void LLRouterTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    LLRouterTesterBase* _testerBase =
      static_cast<LLRouterTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void LLRouterTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    LLRouterTesterBase* _testerBase =
      static_cast<LLRouterTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void LLRouterTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    LLRouterTesterBase* _testerBase =
      static_cast<LLRouterTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_SerWritePort->clear();
    this->fromPortHistory_SerialBufferSend->clear();
    this->fromPortHistory_pingOut->clear();
    this->fromPortHistory_PolyGet->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: SerWritePort
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    pushFromPortEntry_SerWritePort(
        Fw::Buffer &serBuffer
    )
  {
    FromPortEntry_SerWritePort _e = {
      serBuffer
    };
    this->fromPortHistory_SerWritePort->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: SerialBufferSend
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    pushFromPortEntry_SerialBufferSend(
        Fw::Buffer fwBuffer
    )
  {
    FromPortEntry_SerialBufferSend _e = {
      fwBuffer
    };
    this->fromPortHistory_SerialBufferSend->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: pingOut
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    pushFromPortEntry_pingOut(
        U32 key
    )
  {
    FromPortEntry_pingOut _e = {
      key
    };
    this->fromPortHistory_pingOut->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: PolyGet
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    pushFromPortEntry_PolyGet(
        U32 entry,
        Svc::MeasurementStatus &status,
        Fw::Time &time,
        Fw::PolyType &val
    )
  {
    FromPortEntry_PolyGet _e = {
      entry, status, time, val
    };
    this->fromPortHistory_PolyGet->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    from_SerWritePort_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_SerWritePort(),static_cast<AssertArg>(portNum));
    this->from_SerWritePort_handler(
        portNum,
        serBuffer
    );
  }

  void LLRouterTesterBase ::
    from_SerialBufferSend_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_SerialBufferSend(),static_cast<AssertArg>(portNum));
    this->from_SerialBufferSend_handler(
        portNum,
        fwBuffer
    );
  }

  void LLRouterTesterBase ::
    from_pingOut_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    FW_ASSERT(portNum < this->getNum_from_pingOut(),static_cast<AssertArg>(portNum));
    this->from_pingOut_handler(
        portNum,
        key
    );
  }

  void LLRouterTesterBase ::
    from_PolyGet_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 entry,
        Svc::MeasurementStatus &status,
        Fw::Time &time,
        Fw::PolyType &val
    )
  {
    FW_ASSERT(portNum < this->getNum_from_PolyGet(),static_cast<AssertArg>(portNum));
    this->from_PolyGet_handler(
        portNum,
        entry, status, time, val
    );
  }

  // ----------------------------------------------------------------------
  // Command response handling
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    cmdResponseIn(
        const FwOpcodeType opCode,
        const U32 seq,
        const Fw::CommandResponse response
    )
  {
    CmdResponse e = { opCode, seq, response };
    this->cmdResponseHistory->push_back(e);
  }

  // ---------------------------------------------------------------------- 
  // Command: LLR_SET_PATCH_MODE
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    sendCmd_LLR_SET_PATCH_MODE(
        const NATIVE_INT_TYPE instance,
        const U32 cmdSeq,
        LLRouterComponentBase::LLRPatchMode Mode
    )
  {

    // Serialize arguments

    Fw::CmdArgBuffer buff;
    Fw::SerializeStatus _status;
    _status = buff.serialize((FwEnumStoreType) Mode);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));

    // Call output command port
    
    FwOpcodeType _opcode;
    const U32 idBase = this->getIdBase();
    _opcode = LLRouterComponentBase::OPCODE_LLR_SET_PATCH_MODE + idBase;

    if (this->m_to_CmdDisp[0].isConnected()) {
      this->m_to_CmdDisp[0].invoke(
          _opcode,
          cmdSeq,
          buff
      );
    }
    else {
      printf("Test Command Output port not connected!\n");
    }

  }

  // ---------------------------------------------------------------------- 
  // Command: LLR_LOAD_PATCH_FILE
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    sendCmd_LLR_LOAD_PATCH_FILE(
        const NATIVE_INT_TYPE instance,
        const U32 cmdSeq,
        const Fw::CmdStringArg& file,
        LLRouterComponentBase::LL updatingLL,
        bool legacyBootloader,
        bool forceboot,
        U64 disabled_selftests
    )
  {

    // Serialize arguments

    Fw::CmdArgBuffer buff;
    Fw::SerializeStatus _status;
    _status = buff.serialize(file);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));
    _status = buff.serialize((FwEnumStoreType) updatingLL);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));
    _status = buff.serialize(legacyBootloader);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));
    _status = buff.serialize(forceboot);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));
    _status = buff.serialize(disabled_selftests);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));

    // Call output command port
    
    FwOpcodeType _opcode;
    const U32 idBase = this->getIdBase();
    _opcode = LLRouterComponentBase::OPCODE_LLR_LOAD_PATCH_FILE + idBase;

    if (this->m_to_CmdDisp[0].isConnected()) {
      this->m_to_CmdDisp[0].invoke(
          _opcode,
          cmdSeq,
          buff
      );
    }
    else {
      printf("Test Command Output port not connected!\n");
    }

  }

  // ---------------------------------------------------------------------- 
  // Command: LLR_CYCLE_SERIAL
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    sendCmd_LLR_CYCLE_SERIAL(
        const NATIVE_INT_TYPE instance,
        const U32 cmdSeq,
        bool on
    )
  {

    // Serialize arguments

    Fw::CmdArgBuffer buff;
    Fw::SerializeStatus _status;
    _status = buff.serialize(on);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));

    // Call output command port
    
    FwOpcodeType _opcode;
    const U32 idBase = this->getIdBase();
    _opcode = LLRouterComponentBase::OPCODE_LLR_CYCLE_SERIAL + idBase;

    if (this->m_to_CmdDisp[0].isConnected()) {
      this->m_to_CmdDisp[0].invoke(
          _opcode,
          cmdSeq,
          buff
      );
    }
    else {
      printf("Test Command Output port not connected!\n");
    }

  }

  
  void LLRouterTesterBase ::
    sendRawCmd(FwOpcodeType opcode, U32 cmdSeq, Fw::CmdArgBuffer& args) {
       
    const U32 idBase = this->getIdBase();   
    FwOpcodeType _opcode = opcode + idBase;
    if (this->m_to_CmdDisp[0].isConnected()) {
      this->m_to_CmdDisp[0].invoke(
          _opcode,
          cmdSeq,
          args
      );
    }
    else {
      printf("Test Command Output port not connected!\n");
    }
        
  }
  
  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    clearHistory()
  {
    this->cmdResponseHistory->clear();
    this->clearTlm();
    this->textLogHistory->clear();
    this->clearEvents();
    this->clearFromPortHistory();
  }

  // ----------------------------------------------------------------------
  // Time
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
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

      case LLRouterComponentBase::CHANNELID_LLR_CRCERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_CrcErrors: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_CrcErrors(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_RESYNCERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_ResyncErrors: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_ResyncErrors(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMPACKETS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumPackets: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumPackets(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMSERIALERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumSerialErrors: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumSerialErrors(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMGOODPACKETS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumGoodPackets: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumGoodPackets(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMDECODEERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumDecodeErrors: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumDecodeErrors(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMINVALIDPORTS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumInvalidPorts: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumInvalidPorts(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMBADSERIALPORTCALLS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumBadSerialPortCalls: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumBadSerialPortCalls(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMOUPUTBUFFEROVERFLOWS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumOuputBufferOverflows: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumOuputBufferOverflows(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_NUMZEROPKTSIZE:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_NumZeroPktSize: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_NumZeroPktSize(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_MAXCYCLETIME:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_MaxCycleTime: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_MaxCycleTime(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_CYCLETIME:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_CycleTime: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_CycleTime(timeTag, arg);
        break;
      }

      case LLRouterComponentBase::CHANNELID_LLR_PATCHPROGRESS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLR_PatchProgress: %d\n", _status);
          return;
        }
        this->tlmInput_LLR_PatchProgress(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void LLRouterTesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_LLR_CrcErrors->clear();
    this->tlmHistory_LLR_ResyncErrors->clear();
    this->tlmHistory_LLR_NumPackets->clear();
    this->tlmHistory_LLR_NumSerialErrors->clear();
    this->tlmHistory_LLR_NumGoodPackets->clear();
    this->tlmHistory_LLR_NumDecodeErrors->clear();
    this->tlmHistory_LLR_NumInvalidPorts->clear();
    this->tlmHistory_LLR_NumBadSerialPortCalls->clear();
    this->tlmHistory_LLR_NumOuputBufferOverflows->clear();
    this->tlmHistory_LLR_NumZeroPktSize->clear();
    this->tlmHistory_LLR_MaxCycleTime->clear();
    this->tlmHistory_LLR_CycleTime->clear();
    this->tlmHistory_LLR_PatchProgress->clear();
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_CrcErrors
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_CrcErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_CrcErrors e = { timeTag, val };
    this->tlmHistory_LLR_CrcErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_ResyncErrors
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_ResyncErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_ResyncErrors e = { timeTag, val };
    this->tlmHistory_LLR_ResyncErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumPackets
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumPackets(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumPackets e = { timeTag, val };
    this->tlmHistory_LLR_NumPackets->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumSerialErrors
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumSerialErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumSerialErrors e = { timeTag, val };
    this->tlmHistory_LLR_NumSerialErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumGoodPackets
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumGoodPackets(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumGoodPackets e = { timeTag, val };
    this->tlmHistory_LLR_NumGoodPackets->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumDecodeErrors
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumDecodeErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumDecodeErrors e = { timeTag, val };
    this->tlmHistory_LLR_NumDecodeErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumInvalidPorts
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumInvalidPorts(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumInvalidPorts e = { timeTag, val };
    this->tlmHistory_LLR_NumInvalidPorts->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumBadSerialPortCalls
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumBadSerialPortCalls(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumBadSerialPortCalls e = { timeTag, val };
    this->tlmHistory_LLR_NumBadSerialPortCalls->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumOuputBufferOverflows
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumOuputBufferOverflows(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumOuputBufferOverflows e = { timeTag, val };
    this->tlmHistory_LLR_NumOuputBufferOverflows->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_NumZeroPktSize
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_NumZeroPktSize(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_NumZeroPktSize e = { timeTag, val };
    this->tlmHistory_LLR_NumZeroPktSize->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_MaxCycleTime
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_MaxCycleTime(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_MaxCycleTime e = { timeTag, val };
    this->tlmHistory_LLR_MaxCycleTime->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_CycleTime
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_CycleTime(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_CycleTime e = { timeTag, val };
    this->tlmHistory_LLR_CycleTime->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: LLR_PatchProgress
  // ---------------------------------------------------------------------- 

  void LLRouterTesterBase ::
    tlmInput_LLR_PatchProgress(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_LLR_PatchProgress e = { timeTag, val };
    this->tlmHistory_LLR_PatchProgress->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
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

      case LLRouterComponentBase::EVENTID_LLR_DATARECEIVEERROR: 
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
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(FwEnumStoreType),_argSize,sizeof(FwEnumStoreType));
        }
#endif      
        FwEnumStoreType errorInt;
        _status = args.deserialize(errorInt);
        LLRouterComponentBase::LLR_ReceiveError error = static_cast<LLRouterComponentBase::LLR_ReceiveError>(errorInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_LLR_DataReceiveError(error);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_DATACRCERROR: 
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
        FW_ASSERT(_numArgs == 2,_numArgs,2);
        
#endif    
        U16 expected;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(U16),_argSize,sizeof(U16));
        }
#endif      
        _status = args.deserialize(expected);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        U16 actual;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(U16),_argSize,sizeof(U16));
        }
#endif      
        _status = args.deserialize(actual);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_LLR_DataCrcError(expected, actual);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_INVALIDPORTNUM: 
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
        U8 portNum;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(U8),_argSize,sizeof(U8));
        }
#endif      
        _status = args.deserialize(portNum);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_LLR_InvalidPortNum(portNum);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_BADSERIALPORTCALL: 
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
        FW_ASSERT(_numArgs == 2,_numArgs,2);
        
#endif    
        I32 status;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(I32),_argSize,sizeof(I32));
        }
#endif      
        _status = args.deserialize(status);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        U8 portNum;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(U8),_argSize,sizeof(U8));
        }
#endif      
        _status = args.deserialize(portNum);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_LLR_BadSerialPortCall(status, portNum);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_PATCHFILEOPEN: 
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
        FW_ASSERT(_numArgs == 2,_numArgs,2);
        
#endif    
        Fw::LogStringArg filename;
        _status = args.deserialize(filename);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        U32 filesize;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(U32),_argSize,sizeof(U32));
        }
#endif      
        _status = args.deserialize(filesize);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_HI_LLR_PatchFileOpen(filename, filesize);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_PATCHMODECHANGE: 
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
        Fw::LogStringArg newmode;
        _status = args.deserialize(newmode);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_HI_LLR_PatchModeChange(newmode);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_FIRSTACK: 
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
        this->logIn_ACTIVITY_HI_LLR_FirstACK();

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_PATCHEVENT: 
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
        U32 progress;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(U32),_argSize,sizeof(U32));
        }
#endif      
        _status = args.deserialize(progress);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_HI_LLR_PatchEvent(progress);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_PATCHCOMPLETE: 
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
        this->logIn_ACTIVITY_HI_LLR_PatchComplete();

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_PATCHERROR: 
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
        FW_ASSERT(_numArgs == 3,_numArgs,3);
        
#endif    
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(FwEnumStoreType),_argSize,sizeof(FwEnumStoreType));
        }
#endif      
        FwEnumStoreType errorInt;
        _status = args.deserialize(errorInt);
        LLRouterComponentBase::LLR_PatchError error = static_cast<LLRouterComponentBase::LLR_PatchError>(errorInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        I32 err_num;
#if FW_AMPCS_COMPATIBLE
        {
          // Deserialize the argument size
          U8 _argSize;
          _status = args.deserialize(_argSize);
          FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
          );
          FW_ASSERT(_argSize == sizeof(I32),_argSize,sizeof(I32));
        }
#endif      
        _status = args.deserialize(err_num);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        Fw::LogStringArg err_string;
        _status = args.deserialize(err_string);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_LLR_PatchError(error, err_num, err_string);

        break;

      }

      case LLRouterComponentBase::EVENTID_LLR_POLYERROR: 
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
        this->logIn_WARNING_HI_LLR_PolyError();

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void LLRouterTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_LLR_DataReceiveError->clear();
    this->eventHistory_LLR_DataCrcError->clear();
    this->eventHistory_LLR_InvalidPortNum->clear();
    this->eventHistory_LLR_BadSerialPortCall->clear();
    this->eventHistory_LLR_PatchFileOpen->clear();
    this->eventHistory_LLR_PatchModeChange->clear();
    this->eventsSize_LLR_FirstACK = 0;
    this->eventHistory_LLR_PatchEvent->clear();
    this->eventsSize_LLR_PatchComplete = 0;
    this->eventHistory_LLR_PatchError->clear();
    this->eventsSize_LLR_PolyError = 0;
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
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

  void LLRouterTesterBase ::
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

  void LLRouterTesterBase ::
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
  // Event: LLR_DataReceiveError 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_WARNING_HI_LLR_DataReceiveError(
        LLRouterComponentBase::LLR_ReceiveError error
    )
  {
    EventEntry_LLR_DataReceiveError e = {
      error
    };
    eventHistory_LLR_DataReceiveError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_DataCrcError 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_WARNING_HI_LLR_DataCrcError(
        U16 expected,
        U16 actual
    )
  {
    EventEntry_LLR_DataCrcError e = {
      expected, actual
    };
    eventHistory_LLR_DataCrcError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_InvalidPortNum 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_WARNING_HI_LLR_InvalidPortNum(
        U8 portNum
    )
  {
    EventEntry_LLR_InvalidPortNum e = {
      portNum
    };
    eventHistory_LLR_InvalidPortNum->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_BadSerialPortCall 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_WARNING_HI_LLR_BadSerialPortCall(
        I32 status,
        U8 portNum
    )
  {
    EventEntry_LLR_BadSerialPortCall e = {
      status, portNum
    };
    eventHistory_LLR_BadSerialPortCall->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchFileOpen 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_ACTIVITY_HI_LLR_PatchFileOpen(
        Fw::LogStringArg& filename,
        U32 filesize
    )
  {
    EventEntry_LLR_PatchFileOpen e = {
      filename, filesize
    };
    eventHistory_LLR_PatchFileOpen->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchModeChange 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_ACTIVITY_HI_LLR_PatchModeChange(
        Fw::LogStringArg& newmode
    )
  {
    EventEntry_LLR_PatchModeChange e = {
      newmode
    };
    eventHistory_LLR_PatchModeChange->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_FirstACK 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_ACTIVITY_HI_LLR_FirstACK(
        void
    )
  {
    ++this->eventsSize_LLR_FirstACK;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchEvent 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_ACTIVITY_HI_LLR_PatchEvent(
        U32 progress
    )
  {
    EventEntry_LLR_PatchEvent e = {
      progress
    };
    eventHistory_LLR_PatchEvent->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchComplete 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_ACTIVITY_HI_LLR_PatchComplete(
        void
    )
  {
    ++this->eventsSize_LLR_PatchComplete;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PatchError 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_WARNING_HI_LLR_PatchError(
        LLRouterComponentBase::LLR_PatchError error,
        I32 err_num,
        Fw::LogStringArg& err_string
    )
  {
    EventEntry_LLR_PatchError e = {
      error, err_num, err_string
    };
    eventHistory_LLR_PatchError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: LLR_PolyError 
  // ----------------------------------------------------------------------

  void LLRouterTesterBase ::
    logIn_WARNING_HI_LLR_PolyError(
        void
    )
  {
    ++this->eventsSize_LLR_PolyError;
    ++this->eventsSize;
  }

} // end namespace HLProc
