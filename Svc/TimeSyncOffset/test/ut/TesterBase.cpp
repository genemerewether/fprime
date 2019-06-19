// ======================================================================
// \title  TimeSyncOffset/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for TimeSyncOffset component test harness base class
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

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  TimeSyncOffsetTesterBase ::
    TimeSyncOffsetTesterBase(
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
    this->tlmHistory_LLTime =
      new History<TlmEntry_LLTime>(maxHistorySize);
    this->tlmHistory_HLTime =
      new History<TlmEntry_HLTime>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_SchedIn_Timeout =
      new History<EventEntry_SchedIn_Timeout>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_GPIOPulse =
      new History<FromPortEntry_GPIOPulse>(maxHistorySize);
    this->fromPortHistory_ClockTimes =
      new History<FromPortEntry_ClockTimes>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  TimeSyncOffsetTesterBase ::
    ~TimeSyncOffsetTesterBase(void)
  {
    // Destroy telemetry histories
    delete this->tlmHistory_LLTime;
    delete this->tlmHistory_HLTime;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_SchedIn_Timeout;
  }

  void TimeSyncOffsetTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port GPIOPulse

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_GPIOPulse();
        ++_port
    ) {

      this->m_from_GPIOPulse[_port].init();
      this->m_from_GPIOPulse[_port].addCallComp(
          this,
          from_GPIOPulse_static
      );
      this->m_from_GPIOPulse[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_GPIOPulse[%d]",
          this->m_objName,
          _port
      );
      this->m_from_GPIOPulse[_port].setObjName(_portName);
#endif

    }

    // Attach input port ClockTimes

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_ClockTimes();
        ++_port
    ) {

      this->m_from_ClockTimes[_port].init();
      this->m_from_ClockTimes[_port].addCallComp(
          this,
          from_ClockTimes_static
      );
      this->m_from_ClockTimes[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_ClockTimes[%d]",
          this->m_objName,
          _port
      );
      this->m_from_ClockTimes[_port].setObjName(_portName);
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

    // Initialize output port LLTime

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_LLTime();
        ++_port
    ) {
      this->m_to_LLTime[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_LLTime[%d]",
          this->m_objName,
          _port
      );
      this->m_to_LLTime[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_to_SchedIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SchedIn);
  }

  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_to_LLTime(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_LLTime);
  }

  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_from_GPIOPulse(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_GPIOPulse);
  }

  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_from_ClockTimes(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_ClockTimes);
  }

  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE TimeSyncOffsetTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    connect_to_SchedIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const SchedIn
    )
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    this->m_to_SchedIn[portNum].addCallPort(SchedIn);
  }

  void TimeSyncOffsetTesterBase ::
    connect_to_LLTime(
        const NATIVE_INT_TYPE portNum,
        Fw::InputTimePort *const LLTime
    )
  {
    FW_ASSERT(portNum < this->getNum_to_LLTime(),static_cast<AssertArg>(portNum));
    this->m_to_LLTime[portNum].addCallPort(LLTime);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
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

  void TimeSyncOffsetTesterBase ::
    invoke_to_LLTime(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(portNum < this->getNum_to_LLTime(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_LLTime(),static_cast<AssertArg>(portNum));
    this->m_to_LLTime[portNum].invoke(
        time
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool TimeSyncOffsetTesterBase ::
    isConnected_to_SchedIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(), static_cast<AssertArg>(portNum));
    return this->m_to_SchedIn[portNum].isConnected();
  }

  bool TimeSyncOffsetTesterBase ::
    isConnected_to_LLTime(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_LLTime(), static_cast<AssertArg>(portNum));
    return this->m_to_LLTime[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------

  Drv::InputGpioWritePort *TimeSyncOffsetTesterBase ::
    get_from_GPIOPulse(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_GPIOPulse(),static_cast<AssertArg>(portNum));
    return &this->m_from_GPIOPulse[portNum];
  }

  Fw::InputTimePairPort *TimeSyncOffsetTesterBase ::
    get_from_ClockTimes(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_ClockTimes(),static_cast<AssertArg>(portNum));
    return &this->m_from_ClockTimes[portNum];
  }

  Fw::InputTlmPort *TimeSyncOffsetTesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputTimePort *TimeSyncOffsetTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *TimeSyncOffsetTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *TimeSyncOffsetTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    from_GPIOPulse_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
    FW_ASSERT(callComp);
    TimeSyncOffsetTesterBase* _testerBase =
      static_cast<TimeSyncOffsetTesterBase*>(callComp);
    _testerBase->from_GPIOPulse_handlerBase(
        portNum,
        state
    );
  }

  void TimeSyncOffsetTesterBase ::
    from_ClockTimes_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Time time1,
        Fw::Time time2
    )
  {
    FW_ASSERT(callComp);
    TimeSyncOffsetTesterBase* _testerBase =
      static_cast<TimeSyncOffsetTesterBase*>(callComp);
    _testerBase->from_ClockTimes_handlerBase(
        portNum,
        time1, time2
    );
  }

  void TimeSyncOffsetTesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    TimeSyncOffsetTesterBase* _testerBase =
      static_cast<TimeSyncOffsetTesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void TimeSyncOffsetTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    TimeSyncOffsetTesterBase* _testerBase =
      static_cast<TimeSyncOffsetTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void TimeSyncOffsetTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    TimeSyncOffsetTesterBase* _testerBase =
      static_cast<TimeSyncOffsetTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void TimeSyncOffsetTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    TimeSyncOffsetTesterBase* _testerBase =
      static_cast<TimeSyncOffsetTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_GPIOPulse->clear();
    this->fromPortHistory_ClockTimes->clear();
  }

  // ----------------------------------------------------------------------
  // From port: GPIOPulse
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    pushFromPortEntry_GPIOPulse(
        bool state
    )
  {
    FromPortEntry_GPIOPulse _e = {
      state
    };
    this->fromPortHistory_GPIOPulse->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // From port: ClockTimes
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    pushFromPortEntry_ClockTimes(
        Fw::Time time1,
        Fw::Time time2
    )
  {
    FromPortEntry_ClockTimes _e = {
      time1, time2
    };
    this->fromPortHistory_ClockTimes->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    from_GPIOPulse_handlerBase(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
    FW_ASSERT(portNum < this->getNum_from_GPIOPulse(),static_cast<AssertArg>(portNum));
    this->from_GPIOPulse_handler(
        portNum,
        state
    );
  }

  void TimeSyncOffsetTesterBase ::
    from_ClockTimes_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Time time1,
        Fw::Time time2
    )
  {
    FW_ASSERT(portNum < this->getNum_from_ClockTimes(),static_cast<AssertArg>(portNum));
    this->from_ClockTimes_handler(
        portNum,
        time1, time2
    );
  }

  // ----------------------------------------------------------------------
  // History
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
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

  void TimeSyncOffsetTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
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

      case TimeSyncOffsetComponentBase::CHANNELID_LLTIME:
      {
        F64 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLTime: %d\n", _status);
          return;
        }
        this->tlmInput_LLTime(timeTag, arg);
        break;
      }

      case TimeSyncOffsetComponentBase::CHANNELID_HLTIME:
      {
        F64 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HLTime: %d\n", _status);
          return;
        }
        this->tlmInput_HLTime(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void TimeSyncOffsetTesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_LLTime->clear();
    this->tlmHistory_HLTime->clear();
  }

  // ----------------------------------------------------------------------
  // Channel: LLTime
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    tlmInput_LLTime(
        const Fw::Time& timeTag,
        const F64& val
    )
  {
    TlmEntry_LLTime e = { timeTag, val };
    this->tlmHistory_LLTime->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Channel: HLTime
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    tlmInput_HLTime(
        const Fw::Time& timeTag,
        const F64& val
    )
  {
    TlmEntry_HLTime e = { timeTag, val };
    this->tlmHistory_HLTime->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
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

      case TimeSyncOffsetComponentBase::EVENTID_SCHEDIN_TIMEOUT:
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
        U8 sched_timeout;
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
        _status = args.deserialize(sched_timeout);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_LO_SchedIn_Timeout(sched_timeout);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void TimeSyncOffsetTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_SchedIn_Timeout->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
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

  void TimeSyncOffsetTesterBase ::
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

  void TimeSyncOffsetTesterBase ::
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
  // Event: SchedIn_Timeout
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    logIn_WARNING_LO_SchedIn_Timeout(
        U8 sched_timeout
    )
  {
    EventEntry_SchedIn_Timeout e = {
      sched_timeout
    };
    eventHistory_SchedIn_Timeout->push_back(e);
    ++this->eventsSize;
  }

} // end namespace Svc
