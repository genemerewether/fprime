// ======================================================================
// \title  LIDARLiteV3/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for LIDARLiteV3 component test harness base class
//
// \copyright
// Copyright 2009-2016, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#include <stdlib.h>
#include <string.h>
#include "TesterBase.hpp"

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  LIDARLiteV3TesterBase ::
    LIDARLiteV3TesterBase(
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
    this->tlmHistory_LLV3_Distance =
      new History<TlmEntry_LLV3_Distance>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    // Initialize histories for typed user output ports
    this->fromPortHistory_I2CWriteRead =
      new History<FromPortEntry_I2CWriteRead>(maxHistorySize);
    this->fromPortHistory_I2CWriteReadStatus =
      new History<FromPortEntry_I2CWriteReadStatus>(maxHistorySize);
    this->fromPortHistory_I2CConfig =
      new History<FromPortEntry_I2CConfig>(maxHistorySize);
    this->fromPortHistory_AltimeterSend =
      new History<FromPortEntry_AltimeterSend>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  LIDARLiteV3TesterBase ::
    ~LIDARLiteV3TesterBase(void)
  {
    // Destroy telemetry histories
    delete this->tlmHistory_LLV3_Distance;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
  }

  void LIDARLiteV3TesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port I2CWriteRead

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_I2CWriteRead();
        ++_port
    ) {

      this->m_from_I2CWriteRead[_port].init();
      this->m_from_I2CWriteRead[_port].addCallComp(
          this,
          from_I2CWriteRead_static
      );
      this->m_from_I2CWriteRead[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_I2CWriteRead[%d]",
          this->m_objName,
          _port
      );
      this->m_from_I2CWriteRead[_port].setObjName(_portName);
#endif

    }

    // Attach input port I2CWriteReadStatus

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_I2CWriteReadStatus();
        ++_port
    ) {

      this->m_from_I2CWriteReadStatus[_port].init();
      this->m_from_I2CWriteReadStatus[_port].addCallComp(
          this,
          from_I2CWriteReadStatus_static
      );
      this->m_from_I2CWriteReadStatus[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_I2CWriteReadStatus[%d]",
          this->m_objName,
          _port
      );
      this->m_from_I2CWriteReadStatus[_port].setObjName(_portName);
#endif

    }

    // Attach input port I2CConfig

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_I2CConfig();
        ++_port
    ) {

      this->m_from_I2CConfig[_port].init();
      this->m_from_I2CConfig[_port].addCallComp(
          this,
          from_I2CConfig_static
      );
      this->m_from_I2CConfig[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_I2CConfig[%d]",
          this->m_objName,
          _port
      );
      this->m_from_I2CConfig[_port].setObjName(_portName);
#endif

    }

    // Attach input port AltimeterSend

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_AltimeterSend();
        ++_port
    ) {

      this->m_from_AltimeterSend[_port].init();
      this->m_from_AltimeterSend[_port].addCallComp(
          this,
          from_AltimeterSend_static
      );
      this->m_from_AltimeterSend[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_AltimeterSend[%d]",
          this->m_objName,
          _port
      );
      this->m_from_AltimeterSend[_port].setObjName(_portName);
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

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_to_SchedIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SchedIn);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_I2CWriteRead(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_I2CWriteRead);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_I2CWriteReadStatus(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_I2CWriteReadStatus);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_I2CConfig(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_I2CConfig);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_AltimeterSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_AltimeterSend);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    connect_to_SchedIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const SchedIn
    )
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    this->m_to_SchedIn[portNum].addCallPort(SchedIn);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
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

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool LIDARLiteV3TesterBase ::
    isConnected_to_SchedIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(), static_cast<AssertArg>(portNum));
    return this->m_to_SchedIn[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------

  Drv::InputI2CWriteReadPort *LIDARLiteV3TesterBase ::
    get_from_I2CWriteRead(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteRead(),static_cast<AssertArg>(portNum));
    return &this->m_from_I2CWriteRead[portNum];
  }

  Drv::InputI2CWriteReadStatusPort *LIDARLiteV3TesterBase ::
    get_from_I2CWriteReadStatus(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteReadStatus(),static_cast<AssertArg>(portNum));
    return &this->m_from_I2CWriteReadStatus[portNum];
  }

  Drv::InputI2CConfigPort *LIDARLiteV3TesterBase ::
    get_from_I2CConfig(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_I2CConfig(),static_cast<AssertArg>(portNum));
    return &this->m_from_I2CConfig[portNum];
  }

  ROS::sensor_msgs::InputRangePort *LIDARLiteV3TesterBase ::
    get_from_AltimeterSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_AltimeterSend(),static_cast<AssertArg>(portNum));
    return &this->m_from_AltimeterSend[portNum];
  }

  Fw::InputTlmPort *LIDARLiteV3TesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputTimePort *LIDARLiteV3TesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *LIDARLiteV3TesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *LIDARLiteV3TesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    from_I2CWriteRead_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    _testerBase->from_I2CWriteRead_handlerBase(
        portNum,
        writeBuffer, readBuffer
    );
  }

  Drv::I2CStatus LIDARLiteV3TesterBase ::
    from_I2CWriteReadStatus_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        bool shouldBlock
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    return _testerBase->from_I2CWriteReadStatus_handlerBase(
        portNum,
        shouldBlock
    );
  }

  void LIDARLiteV3TesterBase ::
    from_I2CConfig_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    _testerBase->from_I2CConfig_handlerBase(
        portNum,
        busSpeed, slaveAddr, timeout
    );
  }

  void LIDARLiteV3TesterBase ::
    from_AltimeterSend_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::Range &Range
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    _testerBase->from_AltimeterSend_handlerBase(
        portNum,
        Range
    );
  }

  void LIDARLiteV3TesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void LIDARLiteV3TesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void LIDARLiteV3TesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void LIDARLiteV3TesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    LIDARLiteV3TesterBase* _testerBase =
      static_cast<LIDARLiteV3TesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_I2CWriteRead->clear();
    this->fromPortHistory_I2CWriteReadStatus->clear();
    this->fromPortHistory_I2CConfig->clear();
    this->fromPortHistory_AltimeterSend->clear();
  }

  // ----------------------------------------------------------------------
  // From port: I2CWriteRead
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_I2CWriteRead(
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    FromPortEntry_I2CWriteRead _e = {
      writeBuffer, readBuffer
    };
    this->fromPortHistory_I2CWriteRead->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // From port: I2CWriteReadStatus
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_I2CWriteReadStatus(
        bool shouldBlock
    )
  {
    FromPortEntry_I2CWriteReadStatus _e = {
      shouldBlock
    };
    this->fromPortHistory_I2CWriteReadStatus->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // From port: I2CConfig
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_I2CConfig(
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
    FromPortEntry_I2CConfig _e = {
      busSpeed, slaveAddr, timeout
    };
    this->fromPortHistory_I2CConfig->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // From port: AltimeterSend
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_AltimeterSend(
        ROS::sensor_msgs::Range &Range
    )
  {
    FromPortEntry_AltimeterSend _e = {
      Range
    };
    this->fromPortHistory_AltimeterSend->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    from_I2CWriteRead_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteRead(),static_cast<AssertArg>(portNum));
    this->from_I2CWriteRead_handler(
        portNum,
        writeBuffer, readBuffer
    );
  }

  Drv::I2CStatus LIDARLiteV3TesterBase ::
    from_I2CWriteReadStatus_handlerBase(
        const NATIVE_INT_TYPE portNum,
        bool shouldBlock
    )
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteReadStatus(),static_cast<AssertArg>(portNum));
    return this->from_I2CWriteReadStatus_handler(
        portNum,
        shouldBlock
    );
  }

  void LIDARLiteV3TesterBase ::
    from_I2CConfig_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
    FW_ASSERT(portNum < this->getNum_from_I2CConfig(),static_cast<AssertArg>(portNum));
    this->from_I2CConfig_handler(
        portNum,
        busSpeed, slaveAddr, timeout
    );
  }

  void LIDARLiteV3TesterBase ::
    from_AltimeterSend_handlerBase(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::Range &Range
    )
  {
    FW_ASSERT(portNum < this->getNum_from_AltimeterSend(),static_cast<AssertArg>(portNum));
    this->from_AltimeterSend_handler(
        portNum,
        Range
    );
  }

  // ----------------------------------------------------------------------
  // History
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
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

  void LIDARLiteV3TesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
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

      case LIDARLiteV3ComponentBase::CHANNELID_LLV3_DISTANCE:
      {
        F32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLV3_Distance: %d\n", _status);
          return;
        }
        this->tlmInput_LLV3_Distance(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void LIDARLiteV3TesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_LLV3_Distance->clear();
  }

  // ----------------------------------------------------------------------
  // Channel: LLV3_Distance
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    tlmInput_LLV3_Distance(
        const Fw::Time& timeTag,
        const F32& val
    )
  {
    TlmEntry_LLV3_Distance e = { timeTag, val };
    this->tlmHistory_LLV3_Distance->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
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

      case LIDARLiteV3ComponentBase::EVENTID_LLV3_INITCOMPLETE:
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
        this->logIn_ACTIVITY_LO_LLV3_InitComplete();

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void LIDARLiteV3TesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventsSize_LLV3_InitComplete = 0;
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
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

  void LIDARLiteV3TesterBase ::
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

  void LIDARLiteV3TesterBase ::
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
  // Event: LLV3_InitComplete
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    logIn_ACTIVITY_LO_LLV3_InitComplete(
        void
    )
  {
    ++this->eventsSize_LLV3_InitComplete;
    ++this->eventsSize;
  }

} // end namespace Drv
