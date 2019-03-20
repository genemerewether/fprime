// ======================================================================
// \title  ImgComp/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for ImgComp component test harness base class
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

  ImgCompTesterBase ::
    ImgCompTesterBase(
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
    this->tlmHistory_IMGCOMP_BuffersHandled = 
      new History<TlmEntry_IMGCOMP_BuffersHandled>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_IMGCOMP_SoftCompError =
      new History<EventEntry_IMGCOMP_SoftCompError>(maxHistorySize);
    this->eventHistory_IMGCOMP_BadSetting =
      new History<EventEntry_IMGCOMP_BadSetting>(maxHistorySize);
    this->eventHistory_IMGCOMP_NoBuffer =
      new History<EventEntry_IMGCOMP_NoBuffer>(maxHistorySize);
    this->eventHistory_IMGCOMP_BufferOffset =
      new History<EventEntry_IMGCOMP_BufferOffset>(maxHistorySize);
    this->eventHistory_IMGCOMP_NoRestartMarkers =
      new History<EventEntry_IMGCOMP_NoRestartMarkers>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_compressedOutStorage =
      new History<FromPortEntry_compressedOutStorage>(maxHistorySize);
    this->fromPortHistory_compressedOutXmit =
      new History<FromPortEntry_compressedOutXmit>(maxHistorySize);
    this->fromPortHistory_compressedGetStorage =
      new History<FromPortEntry_compressedGetStorage>(maxHistorySize);
    this->fromPortHistory_compressedGetXmit =
      new History<FromPortEntry_compressedGetXmit>(maxHistorySize);
    this->fromPortHistory_uncompressedReturn =
      new History<FromPortEntry_uncompressedReturn>(maxHistorySize);
    this->fromPortHistory_pingOut =
      new History<FromPortEntry_pingOut>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  ImgCompTesterBase ::
    ~ImgCompTesterBase(void) 
  {
    // Destroy command history
    delete this->cmdResponseHistory;
    // Destroy telemetry histories
    delete this->tlmHistory_IMGCOMP_BuffersHandled;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_IMGCOMP_SoftCompError;
    delete this->eventHistory_IMGCOMP_BadSetting;
    delete this->eventHistory_IMGCOMP_NoBuffer;
    delete this->eventHistory_IMGCOMP_BufferOffset;
    delete this->eventHistory_IMGCOMP_NoRestartMarkers;
  }

  void ImgCompTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port compressedOutStorage

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_compressedOutStorage();
        ++_port
    ) {

      this->m_from_compressedOutStorage[_port].init();
      this->m_from_compressedOutStorage[_port].addCallComp(
          this,
          from_compressedOutStorage_static
      );
      this->m_from_compressedOutStorage[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_compressedOutStorage[%d]",
          this->m_objName,
          _port
      );
      this->m_from_compressedOutStorage[_port].setObjName(_portName);
#endif

    }

    // Attach input port compressedOutXmit

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_compressedOutXmit();
        ++_port
    ) {

      this->m_from_compressedOutXmit[_port].init();
      this->m_from_compressedOutXmit[_port].addCallComp(
          this,
          from_compressedOutXmit_static
      );
      this->m_from_compressedOutXmit[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_compressedOutXmit[%d]",
          this->m_objName,
          _port
      );
      this->m_from_compressedOutXmit[_port].setObjName(_portName);
#endif

    }

    // Attach input port compressedGetStorage

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_compressedGetStorage();
        ++_port
    ) {

      this->m_from_compressedGetStorage[_port].init();
      this->m_from_compressedGetStorage[_port].addCallComp(
          this,
          from_compressedGetStorage_static
      );
      this->m_from_compressedGetStorage[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_compressedGetStorage[%d]",
          this->m_objName,
          _port
      );
      this->m_from_compressedGetStorage[_port].setObjName(_portName);
#endif

    }

    // Attach input port compressedGetXmit

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_compressedGetXmit();
        ++_port
    ) {

      this->m_from_compressedGetXmit[_port].init();
      this->m_from_compressedGetXmit[_port].addCallComp(
          this,
          from_compressedGetXmit_static
      );
      this->m_from_compressedGetXmit[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_compressedGetXmit[%d]",
          this->m_objName,
          _port
      );
      this->m_from_compressedGetXmit[_port].setObjName(_portName);
#endif

    }

    // Attach input port uncompressedReturn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_uncompressedReturn();
        ++_port
    ) {

      this->m_from_uncompressedReturn[_port].init();
      this->m_from_uncompressedReturn[_port].addCallComp(
          this,
          from_uncompressedReturn_static
      );
      this->m_from_uncompressedReturn[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_uncompressedReturn[%d]",
          this->m_objName,
          _port
      );
      this->m_from_uncompressedReturn[_port].setObjName(_portName);
#endif

    }

    // Attach input port timeCaller

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_timeCaller();
        ++_port
    ) {

      this->m_from_timeCaller[_port].init();
      this->m_from_timeCaller[_port].addCallComp(
          this,
          from_timeCaller_static
      );
      this->m_from_timeCaller[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_timeCaller[%d]",
          this->m_objName,
          _port
      );
      this->m_from_timeCaller[_port].setObjName(_portName);
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

    // Initialize output port uncompressedIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_uncompressedIn();
        ++_port
    ) {
      this->m_to_uncompressedIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_uncompressedIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_uncompressedIn[_port].setObjName(_portName);
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

    // Initialize output port schedIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_schedIn();
        ++_port
    ) {
      this->m_to_schedIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_schedIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_schedIn[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_compressedOutStorage(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_compressedOutStorage);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_compressedOutXmit(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_compressedOutXmit);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_compressedGetStorage(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_compressedGetStorage);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_compressedGetXmit(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_compressedGetXmit);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_to_uncompressedIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_uncompressedIn);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_uncompressedReturn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_uncompressedReturn);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_to_pingIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_pingIn);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_timeCaller(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_timeCaller);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_pingOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_pingOut);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_to_schedIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_schedIn);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_to_CmdDisp(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_CmdDisp);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_CmdStatus(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdStatus);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_CmdReg(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdReg);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE ImgCompTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    connect_to_uncompressedIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputBufferSendPort *const uncompressedIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_uncompressedIn(),static_cast<AssertArg>(portNum));
    this->m_to_uncompressedIn[portNum].addCallPort(uncompressedIn);
  }

  void ImgCompTesterBase ::
    connect_to_pingIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputPingPort *const pingIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(),static_cast<AssertArg>(portNum));
    this->m_to_pingIn[portNum].addCallPort(pingIn);
  }

  void ImgCompTesterBase ::
    connect_to_schedIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const schedIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_schedIn(),static_cast<AssertArg>(portNum));
    this->m_to_schedIn[portNum].addCallPort(schedIn);
  }

  void ImgCompTesterBase ::
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

  void ImgCompTesterBase ::
    invoke_to_uncompressedIn(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_uncompressedIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_uncompressedIn(),static_cast<AssertArg>(portNum));
    this->m_to_uncompressedIn[portNum].invoke(
        fwBuffer
    );
  }

  void ImgCompTesterBase ::
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

  void ImgCompTesterBase ::
    invoke_to_schedIn(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_schedIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_schedIn(),static_cast<AssertArg>(portNum));
    this->m_to_schedIn[portNum].invoke(
        context
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool ImgCompTesterBase ::
    isConnected_to_uncompressedIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_uncompressedIn(), static_cast<AssertArg>(portNum));
    return this->m_to_uncompressedIn[portNum].isConnected();
  }

  bool ImgCompTesterBase ::
    isConnected_to_pingIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(), static_cast<AssertArg>(portNum));
    return this->m_to_pingIn[portNum].isConnected();
  }

  bool ImgCompTesterBase ::
    isConnected_to_schedIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_schedIn(), static_cast<AssertArg>(portNum));
    return this->m_to_schedIn[portNum].isConnected();
  }

  bool ImgCompTesterBase ::
    isConnected_to_CmdDisp(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_CmdDisp(), static_cast<AssertArg>(portNum));
    return this->m_to_CmdDisp[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputBufferSendPort *ImgCompTesterBase ::
    get_from_compressedOutStorage(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_compressedOutStorage(),static_cast<AssertArg>(portNum));
    return &this->m_from_compressedOutStorage[portNum];
  }

  Fw::InputBufferSendPort *ImgCompTesterBase ::
    get_from_compressedOutXmit(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_compressedOutXmit(),static_cast<AssertArg>(portNum));
    return &this->m_from_compressedOutXmit[portNum];
  }

  Fw::InputBufferGetPort *ImgCompTesterBase ::
    get_from_compressedGetStorage(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_compressedGetStorage(),static_cast<AssertArg>(portNum));
    return &this->m_from_compressedGetStorage[portNum];
  }

  Fw::InputBufferGetPort *ImgCompTesterBase ::
    get_from_compressedGetXmit(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_compressedGetXmit(),static_cast<AssertArg>(portNum));
    return &this->m_from_compressedGetXmit[portNum];
  }

  Fw::InputBufferSendPort *ImgCompTesterBase ::
    get_from_uncompressedReturn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_uncompressedReturn(),static_cast<AssertArg>(portNum));
    return &this->m_from_uncompressedReturn[portNum];
  }

  Fw::InputTimePort *ImgCompTesterBase ::
    get_from_timeCaller(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_timeCaller(),static_cast<AssertArg>(portNum));
    return &this->m_from_timeCaller[portNum];
  }

  Svc::InputPingPort *ImgCompTesterBase ::
    get_from_pingOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_pingOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_pingOut[portNum];
  }

  Fw::InputCmdResponsePort *ImgCompTesterBase ::
    get_from_CmdStatus(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdStatus(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdStatus[portNum];
  }

  Fw::InputCmdRegPort *ImgCompTesterBase ::
    get_from_CmdReg(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdReg(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdReg[portNum];
  }

  Fw::InputTlmPort *ImgCompTesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputLogPort *ImgCompTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *ImgCompTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    from_compressedOutStorage_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(callComp);
    ImgCompTesterBase* _testerBase = 
      static_cast<ImgCompTesterBase*>(callComp);
    _testerBase->from_compressedOutStorage_handlerBase(
        portNum,
        fwBuffer
    );
  }

  void ImgCompTesterBase ::
    from_compressedOutXmit_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(callComp);
    ImgCompTesterBase* _testerBase = 
      static_cast<ImgCompTesterBase*>(callComp);
    _testerBase->from_compressedOutXmit_handlerBase(
        portNum,
        fwBuffer
    );
  }

  Fw::Buffer ImgCompTesterBase ::
    from_compressedGetStorage_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    FW_ASSERT(callComp);
    ImgCompTesterBase* _testerBase = 
      static_cast<ImgCompTesterBase*>(callComp);
    return _testerBase->from_compressedGetStorage_handlerBase(
        portNum,
        size
    );
  }

  Fw::Buffer ImgCompTesterBase ::
    from_compressedGetXmit_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    FW_ASSERT(callComp);
    ImgCompTesterBase* _testerBase = 
      static_cast<ImgCompTesterBase*>(callComp);
    return _testerBase->from_compressedGetXmit_handlerBase(
        portNum,
        size
    );
  }

  void ImgCompTesterBase ::
    from_uncompressedReturn_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(callComp);
    ImgCompTesterBase* _testerBase = 
      static_cast<ImgCompTesterBase*>(callComp);
    _testerBase->from_uncompressedReturn_handlerBase(
        portNum,
        fwBuffer
    );
  }

  void ImgCompTesterBase ::
    from_pingOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    FW_ASSERT(callComp);
    ImgCompTesterBase* _testerBase = 
      static_cast<ImgCompTesterBase*>(callComp);
    _testerBase->from_pingOut_handlerBase(
        portNum,
        key
    );
  }

  void ImgCompTesterBase ::
    from_CmdStatus_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
  {
    ImgCompTesterBase* _testerBase =
      static_cast<ImgCompTesterBase*>(component);
    _testerBase->cmdResponseIn(opCode, cmdSeq, response);
  }

  void ImgCompTesterBase ::
    from_CmdReg_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode
    )
  {

  }

  void ImgCompTesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    ImgCompTesterBase* _testerBase =
      static_cast<ImgCompTesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void ImgCompTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    ImgCompTesterBase* _testerBase =
      static_cast<ImgCompTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void ImgCompTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    ImgCompTesterBase* _testerBase =
      static_cast<ImgCompTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void ImgCompTesterBase ::
    from_timeCaller_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    ImgCompTesterBase* _testerBase =
      static_cast<ImgCompTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_compressedOutStorage->clear();
    this->fromPortHistory_compressedOutXmit->clear();
    this->fromPortHistory_compressedGetStorage->clear();
    this->fromPortHistory_compressedGetXmit->clear();
    this->fromPortHistory_uncompressedReturn->clear();
    this->fromPortHistory_pingOut->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: compressedOutStorage
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
    pushFromPortEntry_compressedOutStorage(
        Fw::Buffer &fwBuffer
    )
  {
    FromPortEntry_compressedOutStorage _e = {
      fwBuffer
    };
    this->fromPortHistory_compressedOutStorage->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: compressedOutXmit
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
    pushFromPortEntry_compressedOutXmit(
        Fw::Buffer &fwBuffer
    )
  {
    FromPortEntry_compressedOutXmit _e = {
      fwBuffer
    };
    this->fromPortHistory_compressedOutXmit->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: compressedGetStorage
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
    pushFromPortEntry_compressedGetStorage(
        U32 size
    )
  {
    FromPortEntry_compressedGetStorage _e = {
      size
    };
    this->fromPortHistory_compressedGetStorage->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: compressedGetXmit
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
    pushFromPortEntry_compressedGetXmit(
        U32 size
    )
  {
    FromPortEntry_compressedGetXmit _e = {
      size
    };
    this->fromPortHistory_compressedGetXmit->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: uncompressedReturn
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
    pushFromPortEntry_uncompressedReturn(
        Fw::Buffer &fwBuffer
    )
  {
    FromPortEntry_uncompressedReturn _e = {
      fwBuffer
    };
    this->fromPortHistory_uncompressedReturn->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: pingOut
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
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
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    from_compressedOutStorage_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_compressedOutStorage(),static_cast<AssertArg>(portNum));
    this->from_compressedOutStorage_handler(
        portNum,
        fwBuffer
    );
  }

  void ImgCompTesterBase ::
    from_compressedOutXmit_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_compressedOutXmit(),static_cast<AssertArg>(portNum));
    this->from_compressedOutXmit_handler(
        portNum,
        fwBuffer
    );
  }

  Fw::Buffer ImgCompTesterBase ::
    from_compressedGetStorage_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    FW_ASSERT(portNum < this->getNum_from_compressedGetStorage(),static_cast<AssertArg>(portNum));
    return this->from_compressedGetStorage_handler(
        portNum,
        size
    );
  }

  Fw::Buffer ImgCompTesterBase ::
    from_compressedGetXmit_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    FW_ASSERT(portNum < this->getNum_from_compressedGetXmit(),static_cast<AssertArg>(portNum));
    return this->from_compressedGetXmit_handler(
        portNum,
        size
    );
  }

  void ImgCompTesterBase ::
    from_uncompressedReturn_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_uncompressedReturn(),static_cast<AssertArg>(portNum));
    this->from_uncompressedReturn_handler(
        portNum,
        fwBuffer
    );
  }

  void ImgCompTesterBase ::
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

  // ----------------------------------------------------------------------
  // Command response handling
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
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
  // Command: IMGCOMP_NoOp
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
    sendCmd_IMGCOMP_NoOp(
        const NATIVE_INT_TYPE instance,
        const U32 cmdSeq
    )
  {

    // Serialize arguments

    Fw::CmdArgBuffer buff;

    // Call output command port
    
    FwOpcodeType _opcode;
    const U32 idBase = this->getIdBase();
    _opcode = ImgCompComponentBase::OPCODE_IMGCOMP_NOOP + idBase;

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

  
  void ImgCompTesterBase ::
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

  void ImgCompTesterBase ::
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

  void ImgCompTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
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

      case ImgCompComponentBase::CHANNELID_IMGCOMP_BUFFERSHANDLED:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing IMGCOMP_BuffersHandled: %d\n", _status);
          return;
        }
        this->tlmInput_IMGCOMP_BuffersHandled(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void ImgCompTesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_IMGCOMP_BuffersHandled->clear();
  }

  // ---------------------------------------------------------------------- 
  // Channel: IMGCOMP_BuffersHandled
  // ---------------------------------------------------------------------- 

  void ImgCompTesterBase ::
    tlmInput_IMGCOMP_BuffersHandled(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_IMGCOMP_BuffersHandled e = { timeTag, val };
    this->tlmHistory_IMGCOMP_BuffersHandled->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
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

      case ImgCompComponentBase::EVENTID_IMGCOMP_SOFTCOMPERROR: 
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
        ImgCompComponentBase::SoftCompErrorType error = static_cast<ImgCompComponentBase::SoftCompErrorType>(errorInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        Fw::LogStringArg msg;
        _status = args.deserialize(msg);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_IMGCOMP_SoftCompError(error, msg);

        break;

      }

      case ImgCompComponentBase::EVENTID_IMGCOMP_BADBUFFER: 
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
        this->logIn_WARNING_HI_IMGCOMP_BadBuffer();

        break;

      }

      case ImgCompComponentBase::EVENTID_IMGCOMP_BADSETTING: 
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
        U32 portNum;
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
        _status = args.deserialize(portNum);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_IMGCOMP_BadSetting(portNum);

        break;

      }

      case ImgCompComponentBase::EVENTID_IMGCOMP_NOBUFFER: 
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
        U32 size;
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
        _status = args.deserialize(size);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_IMGCOMP_NoBuffer(size);

        break;

      }

      case ImgCompComponentBase::EVENTID_IMGCOMP_BUFFEROFFSET: 
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
        FW_ASSERT(_numArgs == 4,_numArgs,4);
        
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
        FwEnumStoreType typeInt;
        _status = args.deserialize(typeInt);
        ImgCompComponentBase::BufferOffsetSkipType type = static_cast<ImgCompComponentBase::BufferOffsetSkipType>(typeInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

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
        FwEnumStoreType outputInt;
        _status = args.deserialize(outputInt);
        ImgCompComponentBase::BufferOffsetSkipOutput output = static_cast<ImgCompComponentBase::BufferOffsetSkipOutput>(outputInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        U32 inBuffer;
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
        _status = args.deserialize(inBuffer);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        U32 portNum;
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
        _status = args.deserialize(portNum);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_IMGCOMP_BufferOffset(type, output, inBuffer, portNum);

        break;

      }

      case ImgCompComponentBase::EVENTID_IMGCOMP_NORESTARTMARKERS: 
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
        I32 error;
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
        _status = args.deserialize(error);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_IMGCOMP_NoRestartMarkers(error);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void ImgCompTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_IMGCOMP_SoftCompError->clear();
    this->eventsSize_IMGCOMP_BadBuffer = 0;
    this->eventHistory_IMGCOMP_BadSetting->clear();
    this->eventHistory_IMGCOMP_NoBuffer->clear();
    this->eventHistory_IMGCOMP_BufferOffset->clear();
    this->eventHistory_IMGCOMP_NoRestartMarkers->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
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

  void ImgCompTesterBase ::
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

  void ImgCompTesterBase ::
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
  // Event: IMGCOMP_SoftCompError 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    logIn_WARNING_HI_IMGCOMP_SoftCompError(
        ImgCompComponentBase::SoftCompErrorType error,
        Fw::LogStringArg& msg
    )
  {
    EventEntry_IMGCOMP_SoftCompError e = {
      error, msg
    };
    eventHistory_IMGCOMP_SoftCompError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_BadBuffer 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    logIn_WARNING_HI_IMGCOMP_BadBuffer(
        void
    )
  {
    ++this->eventsSize_IMGCOMP_BadBuffer;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_BadSetting 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    logIn_WARNING_HI_IMGCOMP_BadSetting(
        U32 portNum
    )
  {
    EventEntry_IMGCOMP_BadSetting e = {
      portNum
    };
    eventHistory_IMGCOMP_BadSetting->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_NoBuffer 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    logIn_WARNING_HI_IMGCOMP_NoBuffer(
        U32 size
    )
  {
    EventEntry_IMGCOMP_NoBuffer e = {
      size
    };
    eventHistory_IMGCOMP_NoBuffer->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_BufferOffset 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    logIn_WARNING_HI_IMGCOMP_BufferOffset(
        ImgCompComponentBase::BufferOffsetSkipType type,
        ImgCompComponentBase::BufferOffsetSkipOutput output,
        U32 inBuffer,
        U32 portNum
    )
  {
    EventEntry_IMGCOMP_BufferOffset e = {
      type, output, inBuffer, portNum
    };
    eventHistory_IMGCOMP_BufferOffset->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: IMGCOMP_NoRestartMarkers 
  // ----------------------------------------------------------------------

  void ImgCompTesterBase ::
    logIn_WARNING_HI_IMGCOMP_NoRestartMarkers(
        I32 error
    )
  {
    EventEntry_IMGCOMP_NoRestartMarkers e = {
      error
    };
    eventHistory_IMGCOMP_NoRestartMarkers->push_back(e);
    ++this->eventsSize;
  }

} // end namespace Svc
