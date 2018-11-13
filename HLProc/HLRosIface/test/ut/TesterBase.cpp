// ======================================================================
// \title  HLRosIface/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for HLRosIface component test harness base class
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

  HLRosIfaceTesterBase ::
    HLRosIfaceTesterBase(
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
    this->fromPortHistory_Tlm =
      new History<FromPortEntry_Tlm>(maxHistorySize);
    this->fromPortHistory_Log =
      new History<FromPortEntry_Log>(maxHistorySize);
    this->fromPortHistory_LogText =
      new History<FromPortEntry_LogText>(maxHistorySize);
    this->fromPortHistory_pingOut =
      new History<FromPortEntry_pingOut>(maxHistorySize);
    this->fromPortHistory_Time =
      new History<FromPortEntry_Time>(maxHistorySize);
    this->fromPortHistory_ImuStateUpdate =
      new History<FromPortEntry_ImuStateUpdate>(maxHistorySize);
    this->fromPortHistory_ActuatorsData =
      new History<FromPortEntry_ActuatorsData>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  HLRosIfaceTesterBase ::
    ~HLRosIfaceTesterBase(void) 
  {
  }

  void HLRosIfaceTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

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

    // Attach input port ImuStateUpdate

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_ImuStateUpdate();
        ++_port
    ) {

      this->m_from_ImuStateUpdate[_port].init();
      this->m_from_ImuStateUpdate[_port].addCallComp(
          this,
          from_ImuStateUpdate_static
      );
      this->m_from_ImuStateUpdate[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_ImuStateUpdate[%d]",
          this->m_objName,
          _port
      );
      this->m_from_ImuStateUpdate[_port].setObjName(_portName);
#endif

    }

    // Attach input port ActuatorsData

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_ActuatorsData();
        ++_port
    ) {

      this->m_from_ActuatorsData[_port].init();
      this->m_from_ActuatorsData[_port].addCallComp(
          this,
          from_ActuatorsData_static
      );
      this->m_from_ActuatorsData[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_ActuatorsData[%d]",
          this->m_objName,
          _port
      );
      this->m_from_ActuatorsData[_port].setObjName(_portName);
#endif

    }

    // Initialize output port Imu

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_Imu();
        ++_port
    ) {
      this->m_to_Imu[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_Imu[%d]",
          this->m_objName,
          _port
      );
      this->m_to_Imu[_port].setObjName(_portName);
#endif

    }

    // Initialize output port sched

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_sched();
        ++_port
    ) {
      this->m_to_sched[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_sched[%d]",
          this->m_objName,
          _port
      );
      this->m_to_sched[_port].setObjName(_portName);
#endif

    }

    // Initialize output port Odometry

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_Odometry();
        ++_port
    ) {
      this->m_to_Odometry[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_Odometry[%d]",
          this->m_objName,
          _port
      );
      this->m_to_Odometry[_port].setObjName(_portName);
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

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_to_Imu(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Imu);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_to_sched(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_sched);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_from_pingOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_pingOut);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_to_Odometry(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Odometry);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_from_ImuStateUpdate(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_ImuStateUpdate);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_from_ActuatorsData(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_ActuatorsData);
  }

  NATIVE_INT_TYPE HLRosIfaceTesterBase ::
    getNum_to_pingIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_pingIn);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void HLRosIfaceTesterBase ::
    connect_to_Imu(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::InputImuNoCovPort *const Imu
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Imu(),static_cast<AssertArg>(portNum));
    this->m_to_Imu[portNum].addCallPort(Imu);
  }

  void HLRosIfaceTesterBase ::
    connect_to_sched(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const sched
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_sched(),static_cast<AssertArg>(portNum));
    this->m_to_sched[portNum].addCallPort(sched);
  }

  void HLRosIfaceTesterBase ::
    connect_to_Odometry(
        const NATIVE_INT_TYPE portNum,
        ROS::nav_msgs::InputOdometryNoCovPort *const Odometry
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Odometry(),static_cast<AssertArg>(portNum));
    this->m_to_Odometry[portNum].addCallPort(Odometry);
  }

  void HLRosIfaceTesterBase ::
    connect_to_pingIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputPingPort *const pingIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(),static_cast<AssertArg>(portNum));
    this->m_to_pingIn[portNum].addCallPort(pingIn);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void HLRosIfaceTesterBase ::
    invoke_to_Imu(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
    FW_ASSERT(portNum < this->getNum_to_Imu(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_Imu(),static_cast<AssertArg>(portNum));
    this->m_to_Imu[portNum].invoke(
        ImuNoCov
    );
  }

  void HLRosIfaceTesterBase ::
    invoke_to_sched(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_sched(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_sched(),static_cast<AssertArg>(portNum));
    this->m_to_sched[portNum].invoke(
        context
    );
  }

  void HLRosIfaceTesterBase ::
    invoke_to_Odometry(
        const NATIVE_INT_TYPE portNum,
        ROS::nav_msgs::OdometryNoCov &OdometryNoCov
    )
  {
    FW_ASSERT(portNum < this->getNum_to_Odometry(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_Odometry(),static_cast<AssertArg>(portNum));
    this->m_to_Odometry[portNum].invoke(
        OdometryNoCov
    );
  }

  void HLRosIfaceTesterBase ::
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

  bool HLRosIfaceTesterBase ::
    isConnected_to_Imu(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Imu(), static_cast<AssertArg>(portNum));
    return this->m_to_Imu[portNum].isConnected();
  }

  bool HLRosIfaceTesterBase ::
    isConnected_to_sched(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_sched(), static_cast<AssertArg>(portNum));
    return this->m_to_sched[portNum].isConnected();
  }

  bool HLRosIfaceTesterBase ::
    isConnected_to_Odometry(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Odometry(), static_cast<AssertArg>(portNum));
    return this->m_to_Odometry[portNum].isConnected();
  }

  bool HLRosIfaceTesterBase ::
    isConnected_to_pingIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_pingIn(), static_cast<AssertArg>(portNum));
    return this->m_to_pingIn[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputTlmPort *HLRosIfaceTesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputLogPort *HLRosIfaceTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

  Fw::InputLogTextPort *HLRosIfaceTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }

  Svc::InputPingPort *HLRosIfaceTesterBase ::
    get_from_pingOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_pingOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_pingOut[portNum];
  }

  Fw::InputTimePort *HLRosIfaceTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  ROS::mav_msgs::InputImuStateUpdatePort *HLRosIfaceTesterBase ::
    get_from_ImuStateUpdate(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_ImuStateUpdate(),static_cast<AssertArg>(portNum));
    return &this->m_from_ImuStateUpdate[portNum];
  }

  ROS::mav_msgs::InputActuatorsPort *HLRosIfaceTesterBase ::
    get_from_ActuatorsData(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_ActuatorsData(),static_cast<AssertArg>(portNum));
    return &this->m_from_ActuatorsData[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void HLRosIfaceTesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    FW_ASSERT(callComp);
    HLRosIfaceTesterBase* _testerBase = 
      static_cast<HLRosIfaceTesterBase*>(callComp);
    _testerBase->from_Tlm_handlerBase(
        portNum,
        id, timeTag, val
    );
  }

  void HLRosIfaceTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(callComp);
    HLRosIfaceTesterBase* _testerBase = 
      static_cast<HLRosIfaceTesterBase*>(callComp);
    _testerBase->from_Log_handlerBase(
        portNum,
        id, timeTag, severity, args
    );
  }

  void HLRosIfaceTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    FW_ASSERT(callComp);
    HLRosIfaceTesterBase* _testerBase = 
      static_cast<HLRosIfaceTesterBase*>(callComp);
    _testerBase->from_LogText_handlerBase(
        portNum,
        id, timeTag, severity, text
    );
  }

  void HLRosIfaceTesterBase ::
    from_pingOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    FW_ASSERT(callComp);
    HLRosIfaceTesterBase* _testerBase = 
      static_cast<HLRosIfaceTesterBase*>(callComp);
    _testerBase->from_pingOut_handlerBase(
        portNum,
        key
    );
  }

  void HLRosIfaceTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(callComp);
    HLRosIfaceTesterBase* _testerBase = 
      static_cast<HLRosIfaceTesterBase*>(callComp);
    _testerBase->from_Time_handlerBase(
        portNum,
        time
    );
  }

  void HLRosIfaceTesterBase ::
    from_ImuStateUpdate_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate
    )
  {
    FW_ASSERT(callComp);
    HLRosIfaceTesterBase* _testerBase = 
      static_cast<HLRosIfaceTesterBase*>(callComp);
    _testerBase->from_ImuStateUpdate_handlerBase(
        portNum,
        ImuStateUpdate
    );
  }

  void HLRosIfaceTesterBase ::
    from_ActuatorsData_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Actuators &Actuators
    )
  {
    FW_ASSERT(callComp);
    HLRosIfaceTesterBase* _testerBase = 
      static_cast<HLRosIfaceTesterBase*>(callComp);
    _testerBase->from_ActuatorsData_handlerBase(
        portNum,
        Actuators
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void HLRosIfaceTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_Tlm->clear();
    this->fromPortHistory_Log->clear();
    this->fromPortHistory_LogText->clear();
    this->fromPortHistory_pingOut->clear();
    this->fromPortHistory_Time->clear();
    this->fromPortHistory_ImuStateUpdate->clear();
    this->fromPortHistory_ActuatorsData->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: Tlm
  // ---------------------------------------------------------------------- 

  void HLRosIfaceTesterBase ::
    pushFromPortEntry_Tlm(
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    FromPortEntry_Tlm _e = {
      id, timeTag, val
    };
    this->fromPortHistory_Tlm->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: Log
  // ---------------------------------------------------------------------- 

  void HLRosIfaceTesterBase ::
    pushFromPortEntry_Log(
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    FromPortEntry_Log _e = {
      id, timeTag, severity, args
    };
    this->fromPortHistory_Log->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: LogText
  // ---------------------------------------------------------------------- 

  void HLRosIfaceTesterBase ::
    pushFromPortEntry_LogText(
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    FromPortEntry_LogText _e = {
      id, timeTag, severity, text
    };
    this->fromPortHistory_LogText->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: pingOut
  // ---------------------------------------------------------------------- 

  void HLRosIfaceTesterBase ::
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
  // From port: Time
  // ---------------------------------------------------------------------- 

  void HLRosIfaceTesterBase ::
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
  // From port: ImuStateUpdate
  // ---------------------------------------------------------------------- 

  void HLRosIfaceTesterBase ::
    pushFromPortEntry_ImuStateUpdate(
        ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate
    )
  {
    FromPortEntry_ImuStateUpdate _e = {
      ImuStateUpdate
    };
    this->fromPortHistory_ImuStateUpdate->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: ActuatorsData
  // ---------------------------------------------------------------------- 

  void HLRosIfaceTesterBase ::
    pushFromPortEntry_ActuatorsData(
        ROS::mav_msgs::Actuators &Actuators
    )
  {
    FromPortEntry_ActuatorsData _e = {
      Actuators
    };
    this->fromPortHistory_ActuatorsData->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void HLRosIfaceTesterBase ::
    from_Tlm_handlerBase(
        const NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    this->from_Tlm_handler(
        portNum,
        id, timeTag, val
    );
  }

  void HLRosIfaceTesterBase ::
    from_Log_handlerBase(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    this->from_Log_handler(
        portNum,
        id, timeTag, severity, args
    );
  }

  void HLRosIfaceTesterBase ::
    from_LogText_handlerBase(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    this->from_LogText_handler(
        portNum,
        id, timeTag, severity, text
    );
  }

  void HLRosIfaceTesterBase ::
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

  void HLRosIfaceTesterBase ::
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

  void HLRosIfaceTesterBase ::
    from_ImuStateUpdate_handlerBase(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate
    )
  {
    FW_ASSERT(portNum < this->getNum_from_ImuStateUpdate(),static_cast<AssertArg>(portNum));
    this->from_ImuStateUpdate_handler(
        portNum,
        ImuStateUpdate
    );
  }

  void HLRosIfaceTesterBase ::
    from_ActuatorsData_handlerBase(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Actuators &Actuators
    )
  {
    FW_ASSERT(portNum < this->getNum_from_ActuatorsData(),static_cast<AssertArg>(portNum));
    this->from_ActuatorsData_handler(
        portNum,
        Actuators
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void HLRosIfaceTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace HLProc
