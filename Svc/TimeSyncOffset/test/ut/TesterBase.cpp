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
    this->tlmHistory_LLOffset =
      new History<TlmEntry_LLOffset>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_GPIOPulse =
      new History<FromPortEntry_GPIOPulse>(maxHistorySize);
    this->fromPortHistory_Offset =
      new History<FromPortEntry_Offset>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  TimeSyncOffsetTesterBase ::
    ~TimeSyncOffsetTesterBase(void)
  {
    // Destroy telemetry histories
    delete this->tlmHistory_LLOffset;
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

    // Attach input port Offset

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_Offset();
        ++_port
    ) {

      this->m_from_Offset[_port].init();
      this->m_from_Offset[_port].addCallComp(
          this,
          from_Offset_static
      );
      this->m_from_Offset[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_Offset[%d]",
          this->m_objName,
          _port
      );
      this->m_from_Offset[_port].setObjName(_portName);
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
    getNum_from_Offset(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Offset);
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

  Fw::InputTimePort *TimeSyncOffsetTesterBase ::
    get_from_Offset(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Offset(),static_cast<AssertArg>(portNum));
    return &this->m_from_Offset[portNum];
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
    from_Offset_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(callComp);
    TimeSyncOffsetTesterBase* _testerBase =
      static_cast<TimeSyncOffsetTesterBase*>(callComp);
    _testerBase->from_Offset_handlerBase(
        portNum,
        time
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
    this->fromPortHistory_Offset->clear();
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
  // From port: Offset
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    pushFromPortEntry_Offset(
        Fw::Time &time
    )
  {
    FromPortEntry_Offset _e = {
      time
    };
    this->fromPortHistory_Offset->push_back(_e);
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
    from_Offset_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(portNum < this->getNum_from_Offset(),static_cast<AssertArg>(portNum));
    this->from_Offset_handler(
        portNum,
        time
    );
  }

  // ----------------------------------------------------------------------
  // History
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    clearHistory()
  {
    this->clearTlm();
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

      case TimeSyncOffsetComponentBase::CHANNELID_LLOFFSET:
      {
        F64 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing LLOffset: %d\n", _status);
          return;
        }
        this->tlmInput_LLOffset(timeTag, arg);
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
    this->tlmHistory_LLOffset->clear();
  }

  // ----------------------------------------------------------------------
  // Channel: LLOffset
  // ----------------------------------------------------------------------

  void TimeSyncOffsetTesterBase ::
    tlmInput_LLOffset(
        const Fw::Time& timeTag,
        const F64& val
    )
  {
    TlmEntry_LLOffset e = { timeTag, val };
    this->tlmHistory_LLOffset->push_back(e);
    ++this->tlmSize;
  }

} // end namespace Svc
