// ======================================================================
// \title  SnapdragonHealth/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for SnapdragonHealth component test harness base class
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

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  SnapdragonHealthTesterBase ::
    SnapdragonHealthTesterBase(
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
    this->tlmHistory_SnapTempZone0DspCpuStat = 
      new History<TlmEntry_SnapTempZone0DspCpuStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone0DspCpuTemp = 
      new History<TlmEntry_SnapTempZone0DspCpuTemp>(maxHistorySize);
    this->tlmHistory_SnapTempZone1DspModemStat = 
      new History<TlmEntry_SnapTempZone1DspModemStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone1DspModemTemp = 
      new History<TlmEntry_SnapTempZone1DspModemTemp>(maxHistorySize);
    this->tlmHistory_SnapTempZone2ModemStat = 
      new History<TlmEntry_SnapTempZone2ModemStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone2ModemTemp = 
      new History<TlmEntry_SnapTempZone2ModemTemp>(maxHistorySize);
    this->tlmHistory_SnapTempZone3VideoStat = 
      new History<TlmEntry_SnapTempZone3VideoStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone3VideoTemp = 
      new History<TlmEntry_SnapTempZone3VideoTemp>(maxHistorySize);
    this->tlmHistory_SnapTempZone4WlanCpuStat = 
      new History<TlmEntry_SnapTempZone4WlanCpuStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone4WlanCpuTemp = 
      new History<TlmEntry_SnapTempZone4WlanCpuTemp>(maxHistorySize);
    this->tlmHistory_SnapTempZone5Cpu0Stat = 
      new History<TlmEntry_SnapTempZone5Cpu0Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone5Cpu0Temp = 
      new History<TlmEntry_SnapTempZone5Cpu0Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone6Cpu1Stat = 
      new History<TlmEntry_SnapTempZone6Cpu1Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone6Cpu1Temp = 
      new History<TlmEntry_SnapTempZone6Cpu1Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone7Cpu2Stat = 
      new History<TlmEntry_SnapTempZone7Cpu2Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone7Cpu2Temp = 
      new History<TlmEntry_SnapTempZone7Cpu2Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone8Cpu3Stat = 
      new History<TlmEntry_SnapTempZone8Cpu3Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone8Cpu3Temp = 
      new History<TlmEntry_SnapTempZone8Cpu3Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone9GpuAStat = 
      new History<TlmEntry_SnapTempZone9GpuAStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone9GpuATemp = 
      new History<TlmEntry_SnapTempZone9GpuATemp>(maxHistorySize);
    this->tlmHistory_SnapTempZone10GpuBStat = 
      new History<TlmEntry_SnapTempZone10GpuBStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone10GpuBTemp = 
      new History<TlmEntry_SnapTempZone10GpuBTemp>(maxHistorySize);
    this->tlmHistory_SnapdragonTotalCpuUtil = 
      new History<TlmEntry_SnapdragonTotalCpuUtil>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu0Util = 
      new History<TlmEntry_SnapdragonCpu0Util>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu1Util = 
      new History<TlmEntry_SnapdragonCpu1Util>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu2Util = 
      new History<TlmEntry_SnapdragonCpu2Util>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu3Util = 
      new History<TlmEntry_SnapdragonCpu3Util>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu0Freq = 
      new History<TlmEntry_SnapdragonCpu0Freq>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu1Freq = 
      new History<TlmEntry_SnapdragonCpu1Freq>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu2Freq = 
      new History<TlmEntry_SnapdragonCpu2Freq>(maxHistorySize);
    this->tlmHistory_SnapdragonCpu3Freq = 
      new History<TlmEntry_SnapdragonCpu3Freq>(maxHistorySize);
    this->tlmHistory_SnapdragonPowerSaver = 
      new History<TlmEntry_SnapdragonPowerSaver>(maxHistorySize);
    this->tlmHistory_SnapdragonBootCount = 
      new History<TlmEntry_SnapdragonBootCount>(maxHistorySize);
    this->tlmHistory_SnapTempZone11PM8841Stat = 
      new History<TlmEntry_SnapTempZone11PM8841Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone11PM8841Temp = 
      new History<TlmEntry_SnapTempZone11PM8841Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone12PM8941Stat = 
      new History<TlmEntry_SnapTempZone12PM8941Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone12PM8941Temp = 
      new History<TlmEntry_SnapTempZone12PM8941Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone13PA0Stat = 
      new History<TlmEntry_SnapTempZone13PA0Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone13PA0Temp = 
      new History<TlmEntry_SnapTempZone13PA0Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone14PA1Stat = 
      new History<TlmEntry_SnapTempZone14PA1Stat>(maxHistorySize);
    this->tlmHistory_SnapTempZone14PA1Temp = 
      new History<TlmEntry_SnapTempZone14PA1Temp>(maxHistorySize);
    this->tlmHistory_SnapTempZone15EmmcStat = 
      new History<TlmEntry_SnapTempZone15EmmcStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone15EmmcTemp = 
      new History<TlmEntry_SnapTempZone15EmmcTemp>(maxHistorySize);
    this->tlmHistory_SnapTempZone16MsmStat = 
      new History<TlmEntry_SnapTempZone16MsmStat>(maxHistorySize);
    this->tlmHistory_SnapTempZone16MsmTemp = 
      new History<TlmEntry_SnapTempZone16MsmTemp>(maxHistorySize);
    this->tlmHistory_SH_FswVersion = 
      new History<TlmEntry_SH_FswVersion>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_SH_FileOpenError =
      new History<EventEntry_SH_FileOpenError>(maxHistorySize);
    this->eventHistory_SH_FileReadError =
      new History<EventEntry_SH_FileReadError>(maxHistorySize);
    this->eventHistory_SH_FileWriteError =
      new History<EventEntry_SH_FileWriteError>(maxHistorySize);
    this->eventHistory_SH_FswVersion =
      new History<EventEntry_SH_FswVersion>(maxHistorySize);
    this->eventHistory_SH_PowerSaver =
      new History<EventEntry_SH_PowerSaver>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_PowerReport =
      new History<FromPortEntry_PowerReport>(maxHistorySize);
    this->fromPortHistory_pingOut =
      new History<FromPortEntry_pingOut>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  SnapdragonHealthTesterBase ::
    ~SnapdragonHealthTesterBase(void) 
  {
    // Destroy command history
    delete this->cmdResponseHistory;
    // Destroy telemetry histories
    delete this->tlmHistory_SnapTempZone0DspCpuStat;
    delete this->tlmHistory_SnapTempZone0DspCpuTemp;
    delete this->tlmHistory_SnapTempZone1DspModemStat;
    delete this->tlmHistory_SnapTempZone1DspModemTemp;
    delete this->tlmHistory_SnapTempZone2ModemStat;
    delete this->tlmHistory_SnapTempZone2ModemTemp;
    delete this->tlmHistory_SnapTempZone3VideoStat;
    delete this->tlmHistory_SnapTempZone3VideoTemp;
    delete this->tlmHistory_SnapTempZone4WlanCpuStat;
    delete this->tlmHistory_SnapTempZone4WlanCpuTemp;
    delete this->tlmHistory_SnapTempZone5Cpu0Stat;
    delete this->tlmHistory_SnapTempZone5Cpu0Temp;
    delete this->tlmHistory_SnapTempZone6Cpu1Stat;
    delete this->tlmHistory_SnapTempZone6Cpu1Temp;
    delete this->tlmHistory_SnapTempZone7Cpu2Stat;
    delete this->tlmHistory_SnapTempZone7Cpu2Temp;
    delete this->tlmHistory_SnapTempZone8Cpu3Stat;
    delete this->tlmHistory_SnapTempZone8Cpu3Temp;
    delete this->tlmHistory_SnapTempZone9GpuAStat;
    delete this->tlmHistory_SnapTempZone9GpuATemp;
    delete this->tlmHistory_SnapTempZone10GpuBStat;
    delete this->tlmHistory_SnapTempZone10GpuBTemp;
    delete this->tlmHistory_SnapdragonTotalCpuUtil;
    delete this->tlmHistory_SnapdragonCpu0Util;
    delete this->tlmHistory_SnapdragonCpu1Util;
    delete this->tlmHistory_SnapdragonCpu2Util;
    delete this->tlmHistory_SnapdragonCpu3Util;
    delete this->tlmHistory_SnapdragonCpu0Freq;
    delete this->tlmHistory_SnapdragonCpu1Freq;
    delete this->tlmHistory_SnapdragonCpu2Freq;
    delete this->tlmHistory_SnapdragonCpu3Freq;
    delete this->tlmHistory_SnapdragonPowerSaver;
    delete this->tlmHistory_SnapdragonBootCount;
    delete this->tlmHistory_SnapTempZone11PM8841Stat;
    delete this->tlmHistory_SnapTempZone11PM8841Temp;
    delete this->tlmHistory_SnapTempZone12PM8941Stat;
    delete this->tlmHistory_SnapTempZone12PM8941Temp;
    delete this->tlmHistory_SnapTempZone13PA0Stat;
    delete this->tlmHistory_SnapTempZone13PA0Temp;
    delete this->tlmHistory_SnapTempZone14PA1Stat;
    delete this->tlmHistory_SnapTempZone14PA1Temp;
    delete this->tlmHistory_SnapTempZone15EmmcStat;
    delete this->tlmHistory_SnapTempZone15EmmcTemp;
    delete this->tlmHistory_SnapTempZone16MsmStat;
    delete this->tlmHistory_SnapTempZone16MsmTemp;
    delete this->tlmHistory_SH_FswVersion;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_SH_FileOpenError;
    delete this->eventHistory_SH_FileReadError;
    delete this->eventHistory_SH_FileWriteError;
    delete this->eventHistory_SH_FswVersion;
    delete this->eventHistory_SH_PowerSaver;
  }

  void SnapdragonHealthTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port PowerReport

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_PowerReport();
        ++_port
    ) {

      this->m_from_PowerReport[_port].init();
      this->m_from_PowerReport[_port].addCallComp(
          this,
          from_PowerReport_static
      );
      this->m_from_PowerReport[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_PowerReport[%d]",
          this->m_objName,
          _port
      );
      this->m_from_PowerReport[_port].setObjName(_portName);
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

    // Initialize output port SchedIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_SchedIn();
        ++_port
    ) {
      this->m_to_SchedIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_SchedIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_SchedIn[_port].setObjName(_portName);
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

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_to_SchedIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SchedIn);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_PowerReport(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_PowerReport);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_to_pingIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_pingIn);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_pingOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_pingOut);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_to_CmdDisp(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_CmdDisp);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_CmdStatus(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdStatus);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_CmdReg(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdReg);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE SnapdragonHealthTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    connect_to_SchedIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const SchedIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    this->m_to_SchedIn[portNum].addCallPort(SchedIn);
  }

  void SnapdragonHealthTesterBase ::
    connect_to_pingIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputPingPort *const pingIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(),static_cast<AssertArg>(portNum));
    this->m_to_pingIn[portNum].addCallPort(pingIn);
  }

  void SnapdragonHealthTesterBase ::
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

  void SnapdragonHealthTesterBase ::
    invoke_to_SchedIn(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    this->m_to_SchedIn[portNum].invoke(
        context
    );
  }

  void SnapdragonHealthTesterBase ::
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

  bool SnapdragonHealthTesterBase ::
    isConnected_to_SchedIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(), static_cast<AssertArg>(portNum));
    return this->m_to_SchedIn[portNum].isConnected();
  }

  bool SnapdragonHealthTesterBase ::
    isConnected_to_pingIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(), static_cast<AssertArg>(portNum));
    return this->m_to_pingIn[portNum].isConnected();
  }

  bool SnapdragonHealthTesterBase ::
    isConnected_to_CmdDisp(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_CmdDisp(), static_cast<AssertArg>(portNum));
    return this->m_to_CmdDisp[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Svc::InputPolyPort *SnapdragonHealthTesterBase ::
    get_from_PowerReport(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_PowerReport(),static_cast<AssertArg>(portNum));
    return &this->m_from_PowerReport[portNum];
  }

  Svc::InputPingPort *SnapdragonHealthTesterBase ::
    get_from_pingOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_pingOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_pingOut[portNum];
  }

  Fw::InputCmdResponsePort *SnapdragonHealthTesterBase ::
    get_from_CmdStatus(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdStatus(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdStatus[portNum];
  }

  Fw::InputCmdRegPort *SnapdragonHealthTesterBase ::
    get_from_CmdReg(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdReg(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdReg[portNum];
  }

  Fw::InputTlmPort *SnapdragonHealthTesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputTimePort *SnapdragonHealthTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *SnapdragonHealthTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *SnapdragonHealthTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    from_PowerReport_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 entry,
        Svc::MeasurementStatus &status,
        Fw::Time &time,
        Fw::PolyType &val
    )
  {
    FW_ASSERT(callComp);
    SnapdragonHealthTesterBase* _testerBase = 
      static_cast<SnapdragonHealthTesterBase*>(callComp);
    _testerBase->from_PowerReport_handlerBase(
        portNum,
        entry, status, time, val
    );
  }

  void SnapdragonHealthTesterBase ::
    from_pingOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    FW_ASSERT(callComp);
    SnapdragonHealthTesterBase* _testerBase = 
      static_cast<SnapdragonHealthTesterBase*>(callComp);
    _testerBase->from_pingOut_handlerBase(
        portNum,
        key
    );
  }

  void SnapdragonHealthTesterBase ::
    from_CmdStatus_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
  {
    SnapdragonHealthTesterBase* _testerBase =
      static_cast<SnapdragonHealthTesterBase*>(component);
    _testerBase->cmdResponseIn(opCode, cmdSeq, response);
  }

  void SnapdragonHealthTesterBase ::
    from_CmdReg_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode
    )
  {

  }

  void SnapdragonHealthTesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    SnapdragonHealthTesterBase* _testerBase =
      static_cast<SnapdragonHealthTesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void SnapdragonHealthTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    SnapdragonHealthTesterBase* _testerBase =
      static_cast<SnapdragonHealthTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void SnapdragonHealthTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    SnapdragonHealthTesterBase* _testerBase =
      static_cast<SnapdragonHealthTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void SnapdragonHealthTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    SnapdragonHealthTesterBase* _testerBase =
      static_cast<SnapdragonHealthTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_PowerReport->clear();
    this->fromPortHistory_pingOut->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: PowerReport
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    pushFromPortEntry_PowerReport(
        U32 entry,
        Svc::MeasurementStatus &status,
        Fw::Time &time,
        Fw::PolyType &val
    )
  {
    FromPortEntry_PowerReport _e = {
      entry, status, time, val
    };
    this->fromPortHistory_PowerReport->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: pingOut
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
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

  void SnapdragonHealthTesterBase ::
    from_PowerReport_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 entry,
        Svc::MeasurementStatus &status,
        Fw::Time &time,
        Fw::PolyType &val
    )
  {
    FW_ASSERT(portNum < this->getNum_from_PowerReport(),static_cast<AssertArg>(portNum));
    this->from_PowerReport_handler(
        portNum,
        entry, status, time, val
    );
  }

  void SnapdragonHealthTesterBase ::
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

  void SnapdragonHealthTesterBase ::
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
  // Command: SnapdragonHealth_SetPowerSaver
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    sendCmd_SnapdragonHealth_SetPowerSaver(
        const NATIVE_INT_TYPE instance,
        const U32 cmdSeq,
        SnapdragonHealthComponentBase::PowerSaverMode Mode
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
    _opcode = SnapdragonHealthComponentBase::OPCODE_SNAPDRAGONHEALTH_SETPOWERSAVER + idBase;

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

  
  void SnapdragonHealthTesterBase ::
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

  void SnapdragonHealthTesterBase ::
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

  void SnapdragonHealthTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
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

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE0DSPCPUSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone0DspCpuStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone0DspCpuStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE0DSPCPUTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone0DspCpuTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone0DspCpuTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE1DSPMODEMSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone1DspModemStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone1DspModemStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE1DSPMODEMTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone1DspModemTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone1DspModemTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE2MODEMSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone2ModemStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone2ModemStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE2MODEMTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone2ModemTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone2ModemTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE3VIDEOSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone3VideoStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone3VideoStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE3VIDEOTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone3VideoTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone3VideoTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE4WLANCPUSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone4WlanCpuStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone4WlanCpuStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE4WLANCPUTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone4WlanCpuTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone4WlanCpuTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE5CPU0STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone5Cpu0Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone5Cpu0Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE5CPU0TEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone5Cpu0Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone5Cpu0Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE6CPU1STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone6Cpu1Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone6Cpu1Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE6CPU1TEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone6Cpu1Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone6Cpu1Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE7CPU2STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone7Cpu2Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone7Cpu2Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE7CPU2TEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone7Cpu2Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone7Cpu2Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE8CPU3STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone8Cpu3Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone8Cpu3Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE8CPU3TEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone8Cpu3Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone8Cpu3Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE9GPUASTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone9GpuAStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone9GpuAStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE9GPUATEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone9GpuATemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone9GpuATemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE10GPUBSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone10GpuBStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone10GpuBStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE10GPUBTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone10GpuBTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone10GpuBTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONTOTALCPUUTIL:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonTotalCpuUtil: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonTotalCpuUtil(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU0UTIL:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu0Util: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu0Util(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU1UTIL:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu1Util: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu1Util(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU2UTIL:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu2Util: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu2Util(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU3UTIL:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu3Util: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu3Util(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU0FREQ:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu0Freq: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu0Freq(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU1FREQ:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu1Freq: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu1Freq(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU2FREQ:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu2Freq: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu2Freq(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONCPU3FREQ:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonCpu3Freq: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonCpu3Freq(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONPOWERSAVER:
      {
        FwEnumStoreType SnapdragonPowerSaverarg;
        const Fw::SerializeStatus _status = val.deserialize(SnapdragonPowerSaverarg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonPowerSaver: %d\n", _status);
          return;
        }
        SnapdragonHealthComponentBase::PowerSaverModeTlm arg = 
          static_cast<SnapdragonHealthComponentBase::PowerSaverModeTlm>(SnapdragonPowerSaverarg);
        this->tlmInput_SnapdragonPowerSaver(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPDRAGONBOOTCOUNT:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapdragonBootCount: %d\n", _status);
          return;
        }
        this->tlmInput_SnapdragonBootCount(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE11PM8841STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone11PM8841Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone11PM8841Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE11PM8841TEMP:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone11PM8841Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone11PM8841Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE12PM8941STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone12PM8941Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone12PM8941Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE12PM8941TEMP:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone12PM8941Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone12PM8941Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE13PA0STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone13PA0Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone13PA0Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE13PA0TEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone13PA0Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone13PA0Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE14PA1STAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone14PA1Stat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone14PA1Stat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE14PA1TEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone14PA1Temp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone14PA1Temp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE15EMMCSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone15EmmcStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone15EmmcStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE15EMMCTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone15EmmcTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone15EmmcTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE16MSMSTAT:
      {
        bool arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone16MsmStat: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone16MsmStat(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SNAPTEMPZONE16MSMTEMP:
      {
        I32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SnapTempZone16MsmTemp: %d\n", _status);
          return;
        }
        this->tlmInput_SnapTempZone16MsmTemp(timeTag, arg);
        break;
      }

      case SnapdragonHealthComponentBase::CHANNELID_SH_FSWVERSION:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing SH_FswVersion: %d\n", _status);
          return;
        }
        this->tlmInput_SH_FswVersion(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void SnapdragonHealthTesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_SnapTempZone0DspCpuStat->clear();
    this->tlmHistory_SnapTempZone0DspCpuTemp->clear();
    this->tlmHistory_SnapTempZone1DspModemStat->clear();
    this->tlmHistory_SnapTempZone1DspModemTemp->clear();
    this->tlmHistory_SnapTempZone2ModemStat->clear();
    this->tlmHistory_SnapTempZone2ModemTemp->clear();
    this->tlmHistory_SnapTempZone3VideoStat->clear();
    this->tlmHistory_SnapTempZone3VideoTemp->clear();
    this->tlmHistory_SnapTempZone4WlanCpuStat->clear();
    this->tlmHistory_SnapTempZone4WlanCpuTemp->clear();
    this->tlmHistory_SnapTempZone5Cpu0Stat->clear();
    this->tlmHistory_SnapTempZone5Cpu0Temp->clear();
    this->tlmHistory_SnapTempZone6Cpu1Stat->clear();
    this->tlmHistory_SnapTempZone6Cpu1Temp->clear();
    this->tlmHistory_SnapTempZone7Cpu2Stat->clear();
    this->tlmHistory_SnapTempZone7Cpu2Temp->clear();
    this->tlmHistory_SnapTempZone8Cpu3Stat->clear();
    this->tlmHistory_SnapTempZone8Cpu3Temp->clear();
    this->tlmHistory_SnapTempZone9GpuAStat->clear();
    this->tlmHistory_SnapTempZone9GpuATemp->clear();
    this->tlmHistory_SnapTempZone10GpuBStat->clear();
    this->tlmHistory_SnapTempZone10GpuBTemp->clear();
    this->tlmHistory_SnapdragonTotalCpuUtil->clear();
    this->tlmHistory_SnapdragonCpu0Util->clear();
    this->tlmHistory_SnapdragonCpu1Util->clear();
    this->tlmHistory_SnapdragonCpu2Util->clear();
    this->tlmHistory_SnapdragonCpu3Util->clear();
    this->tlmHistory_SnapdragonCpu0Freq->clear();
    this->tlmHistory_SnapdragonCpu1Freq->clear();
    this->tlmHistory_SnapdragonCpu2Freq->clear();
    this->tlmHistory_SnapdragonCpu3Freq->clear();
    this->tlmHistory_SnapdragonPowerSaver->clear();
    this->tlmHistory_SnapdragonBootCount->clear();
    this->tlmHistory_SnapTempZone11PM8841Stat->clear();
    this->tlmHistory_SnapTempZone11PM8841Temp->clear();
    this->tlmHistory_SnapTempZone12PM8941Stat->clear();
    this->tlmHistory_SnapTempZone12PM8941Temp->clear();
    this->tlmHistory_SnapTempZone13PA0Stat->clear();
    this->tlmHistory_SnapTempZone13PA0Temp->clear();
    this->tlmHistory_SnapTempZone14PA1Stat->clear();
    this->tlmHistory_SnapTempZone14PA1Temp->clear();
    this->tlmHistory_SnapTempZone15EmmcStat->clear();
    this->tlmHistory_SnapTempZone15EmmcTemp->clear();
    this->tlmHistory_SnapTempZone16MsmStat->clear();
    this->tlmHistory_SnapTempZone16MsmTemp->clear();
    this->tlmHistory_SH_FswVersion->clear();
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone0DspCpuStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone0DspCpuStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone0DspCpuStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone0DspCpuStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone0DspCpuTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone0DspCpuTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone0DspCpuTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone0DspCpuTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone1DspModemStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone1DspModemStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone1DspModemStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone1DspModemStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone1DspModemTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone1DspModemTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone1DspModemTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone1DspModemTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone2ModemStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone2ModemStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone2ModemStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone2ModemStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone2ModemTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone2ModemTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone2ModemTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone2ModemTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone3VideoStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone3VideoStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone3VideoStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone3VideoStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone3VideoTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone3VideoTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone3VideoTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone3VideoTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone4WlanCpuStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone4WlanCpuStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone4WlanCpuStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone4WlanCpuStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone4WlanCpuTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone4WlanCpuTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone4WlanCpuTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone4WlanCpuTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone5Cpu0Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone5Cpu0Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone5Cpu0Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone5Cpu0Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone5Cpu0Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone5Cpu0Temp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone5Cpu0Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone5Cpu0Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone6Cpu1Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone6Cpu1Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone6Cpu1Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone6Cpu1Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone6Cpu1Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone6Cpu1Temp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone6Cpu1Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone6Cpu1Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone7Cpu2Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone7Cpu2Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone7Cpu2Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone7Cpu2Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone7Cpu2Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone7Cpu2Temp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone7Cpu2Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone7Cpu2Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone8Cpu3Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone8Cpu3Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone8Cpu3Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone8Cpu3Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone8Cpu3Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone8Cpu3Temp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone8Cpu3Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone8Cpu3Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone9GpuAStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone9GpuAStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone9GpuAStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone9GpuAStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone9GpuATemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone9GpuATemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone9GpuATemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone9GpuATemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone10GpuBStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone10GpuBStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone10GpuBStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone10GpuBStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone10GpuBTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone10GpuBTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone10GpuBTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone10GpuBTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonTotalCpuUtil
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonTotalCpuUtil(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_SnapdragonTotalCpuUtil e = { timeTag, val };
    this->tlmHistory_SnapdragonTotalCpuUtil->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu0Util
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu0Util(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_SnapdragonCpu0Util e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu0Util->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu1Util
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu1Util(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_SnapdragonCpu1Util e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu1Util->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu2Util
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu2Util(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_SnapdragonCpu2Util e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu2Util->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu3Util
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu3Util(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_SnapdragonCpu3Util e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu3Util->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu0Freq
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu0Freq(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_SnapdragonCpu0Freq e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu0Freq->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu1Freq
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu1Freq(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_SnapdragonCpu1Freq e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu1Freq->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu2Freq
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu2Freq(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_SnapdragonCpu2Freq e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu2Freq->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonCpu3Freq
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonCpu3Freq(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_SnapdragonCpu3Freq e = { timeTag, val };
    this->tlmHistory_SnapdragonCpu3Freq->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonPowerSaver
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonPowerSaver(
        const Fw::Time& timeTag,
        const SnapdragonHealthComponentBase::PowerSaverModeTlm& val
    )
  {
    TlmEntry_SnapdragonPowerSaver e = { timeTag, val };
    this->tlmHistory_SnapdragonPowerSaver->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapdragonBootCount
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapdragonBootCount(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_SnapdragonBootCount e = { timeTag, val };
    this->tlmHistory_SnapdragonBootCount->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone11PM8841Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone11PM8841Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone11PM8841Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone11PM8841Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone11PM8841Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone11PM8841Temp(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_SnapTempZone11PM8841Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone11PM8841Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone12PM8941Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone12PM8941Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone12PM8941Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone12PM8941Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone12PM8941Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone12PM8941Temp(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_SnapTempZone12PM8941Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone12PM8941Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone13PA0Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone13PA0Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone13PA0Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone13PA0Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone13PA0Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone13PA0Temp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone13PA0Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone13PA0Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone14PA1Stat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone14PA1Stat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone14PA1Stat e = { timeTag, val };
    this->tlmHistory_SnapTempZone14PA1Stat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone14PA1Temp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone14PA1Temp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone14PA1Temp e = { timeTag, val };
    this->tlmHistory_SnapTempZone14PA1Temp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone15EmmcStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone15EmmcStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone15EmmcStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone15EmmcStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone15EmmcTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone15EmmcTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone15EmmcTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone15EmmcTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone16MsmStat
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone16MsmStat(
        const Fw::Time& timeTag,
        const bool& val
    )
  {
    TlmEntry_SnapTempZone16MsmStat e = { timeTag, val };
    this->tlmHistory_SnapTempZone16MsmStat->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SnapTempZone16MsmTemp
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SnapTempZone16MsmTemp(
        const Fw::Time& timeTag,
        const I32& val
    )
  {
    TlmEntry_SnapTempZone16MsmTemp e = { timeTag, val };
    this->tlmHistory_SnapTempZone16MsmTemp->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: SH_FswVersion
  // ---------------------------------------------------------------------- 

  void SnapdragonHealthTesterBase ::
    tlmInput_SH_FswVersion(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_SH_FswVersion e = { timeTag, val };
    this->tlmHistory_SH_FswVersion->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
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

      case SnapdragonHealthComponentBase::EVENTID_SH_FILEOPENERROR: 
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
        I32 type;
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
        _status = args.deserialize(type);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        I32 stat;
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
        _status = args.deserialize(stat);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_LO_SH_FileOpenError(type, stat);

        break;

      }

      case SnapdragonHealthComponentBase::EVENTID_SH_FILEREADERROR: 
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
        I32 type;
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
        _status = args.deserialize(type);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        I32 stat;
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
        _status = args.deserialize(stat);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_LO_SH_FileReadError(type, stat);

        break;

      }

      case SnapdragonHealthComponentBase::EVENTID_SH_FILEWRITEERROR: 
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
        I32 type;
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
        _status = args.deserialize(type);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        I32 stat;
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
        _status = args.deserialize(stat);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_SH_FileWriteError(type, stat);

        break;

      }

      case SnapdragonHealthComponentBase::EVENTID_SH_FSWVERSION: 
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
        U32 hash;
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
        _status = args.deserialize(hash);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        Fw::LogStringArg branch;
        _status = args.deserialize(branch);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_HI_SH_FswVersion(hash, branch);

        break;

      }

      case SnapdragonHealthComponentBase::EVENTID_SH_POWERSAVER: 
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
        FwEnumStoreType ModeInt;
        _status = args.deserialize(ModeInt);
        SnapdragonHealthComponentBase::PowerSaverModeEvr Mode = static_cast<SnapdragonHealthComponentBase::PowerSaverModeEvr>(ModeInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_HI_SH_PowerSaver(Mode);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void SnapdragonHealthTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_SH_FileOpenError->clear();
    this->eventHistory_SH_FileReadError->clear();
    this->eventHistory_SH_FileWriteError->clear();
    this->eventHistory_SH_FswVersion->clear();
    this->eventHistory_SH_PowerSaver->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
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

  void SnapdragonHealthTesterBase ::
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

  void SnapdragonHealthTesterBase ::
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
  // Event: SH_FileOpenError 
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    logIn_WARNING_LO_SH_FileOpenError(
        I32 type,
        I32 stat
    )
  {
    EventEntry_SH_FileOpenError e = {
      type, stat
    };
    eventHistory_SH_FileOpenError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: SH_FileReadError 
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    logIn_WARNING_LO_SH_FileReadError(
        I32 type,
        I32 stat
    )
  {
    EventEntry_SH_FileReadError e = {
      type, stat
    };
    eventHistory_SH_FileReadError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: SH_FileWriteError 
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    logIn_WARNING_HI_SH_FileWriteError(
        I32 type,
        I32 stat
    )
  {
    EventEntry_SH_FileWriteError e = {
      type, stat
    };
    eventHistory_SH_FileWriteError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: SH_FswVersion 
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    logIn_ACTIVITY_HI_SH_FswVersion(
        U32 hash,
        Fw::LogStringArg& branch
    )
  {
    EventEntry_SH_FswVersion e = {
      hash, branch
    };
    eventHistory_SH_FswVersion->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: SH_PowerSaver 
  // ----------------------------------------------------------------------

  void SnapdragonHealthTesterBase ::
    logIn_ACTIVITY_HI_SH_PowerSaver(
        SnapdragonHealthComponentBase::PowerSaverModeEvr Mode
    )
  {
    EventEntry_SH_PowerSaver e = {
      Mode
    };
    eventHistory_SH_PowerSaver->push_back(e);
    ++this->eventsSize;
  }

} // end namespace SnapdragonFlight
