// ======================================================================
// \title  STIM300/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for STIM300 component test harness base class
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

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  STIM300TesterBase ::
    STIM300TesterBase(
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
    this->tlmHistory_NumPackets = 
      new History<TlmEntry_NumPackets>(maxHistorySize);
    this->tlmHistory_ImuPacket = 
      new History<TlmEntry_ImuPacket>(maxHistorySize);
    this->tlmHistory_TimeSyncStatus = 
      new History<TlmEntry_TimeSyncStatus>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_InvalidCounter =
      new History<EventEntry_InvalidCounter>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_IMU =
      new History<FromPortEntry_IMU>(maxHistorySize);
    this->fromPortHistory_packetTime =
      new History<FromPortEntry_packetTime>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  STIM300TesterBase ::
    ~STIM300TesterBase(void) 
  {
    // Destroy telemetry histories
    delete this->tlmHistory_NumPackets;
    delete this->tlmHistory_ImuPacket;
    delete this->tlmHistory_TimeSyncStatus;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_InvalidCounter;
  }

  void STIM300TesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port IMU

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_IMU();
        ++_port
    ) {

      this->m_from_IMU[_port].init();
      this->m_from_IMU[_port].addCallComp(
          this,
          from_IMU_static
      );
      this->m_from_IMU[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_IMU[%d]",
          this->m_objName,
          _port
      );
      this->m_from_IMU[_port].setObjName(_portName);
#endif

    }

    // Attach input port packetTime

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_packetTime();
        ++_port
    ) {

      this->m_from_packetTime[_port].init();
      this->m_from_packetTime[_port].addCallComp(
          this,
          from_packetTime_static
      );
      this->m_from_packetTime[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_packetTime[%d]",
          this->m_objName,
          _port
      );
      this->m_from_packetTime[_port].setObjName(_portName);
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

    // Initialize output port serialRead

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_serialRead();
        ++_port
    ) {
      this->m_to_serialRead[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_serialRead[%d]",
          this->m_objName,
          _port
      );
      this->m_to_serialRead[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_from_IMU(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_IMU);
  }

  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_to_sched(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_sched);
  }

  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_from_packetTime(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_packetTime);
  }

  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_to_serialRead(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_serialRead);
  }

  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE STIM300TesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    connect_to_sched(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const sched
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_sched(),static_cast<AssertArg>(portNum));
    this->m_to_sched[portNum].addCallPort(sched);
  }

  void STIM300TesterBase ::
    connect_to_serialRead(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSerialReadPort *const serialRead
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_serialRead(),static_cast<AssertArg>(portNum));
    this->m_to_serialRead[portNum].addCallPort(serialRead);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
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

  void STIM300TesterBase ::
    invoke_to_serialRead(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        SerialReadStatus &status
    )
  {
    FW_ASSERT(portNum < this->getNum_to_serialRead(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_serialRead(),static_cast<AssertArg>(portNum));
    this->m_to_serialRead[portNum].invoke(
        serBuffer, status
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool STIM300TesterBase ::
    isConnected_to_sched(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_sched(), static_cast<AssertArg>(portNum));
    return this->m_to_sched[portNum].isConnected();
  }

  bool STIM300TesterBase ::
    isConnected_to_serialRead(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_serialRead(), static_cast<AssertArg>(portNum));
    return this->m_to_serialRead[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  ROS::sensor_msgs::InputImuNoCovPort *STIM300TesterBase ::
    get_from_IMU(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_IMU(),static_cast<AssertArg>(portNum));
    return &this->m_from_IMU[portNum];
  }

  Fw::InputTimePort *STIM300TesterBase ::
    get_from_packetTime(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_packetTime(),static_cast<AssertArg>(portNum));
    return &this->m_from_packetTime[portNum];
  }

  Fw::InputTlmPort *STIM300TesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputTimePort *STIM300TesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *STIM300TesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *STIM300TesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    from_IMU_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
    FW_ASSERT(callComp);
    STIM300TesterBase* _testerBase = 
      static_cast<STIM300TesterBase*>(callComp);
    _testerBase->from_IMU_handlerBase(
        portNum,
        ImuNoCov
    );
  }

  void STIM300TesterBase ::
    from_packetTime_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(callComp);
    STIM300TesterBase* _testerBase = 
      static_cast<STIM300TesterBase*>(callComp);
    _testerBase->from_packetTime_handlerBase(
        portNum,
        time
    );
  }

  void STIM300TesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    STIM300TesterBase* _testerBase =
      static_cast<STIM300TesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void STIM300TesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    STIM300TesterBase* _testerBase =
      static_cast<STIM300TesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void STIM300TesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    STIM300TesterBase* _testerBase =
      static_cast<STIM300TesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void STIM300TesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    STIM300TesterBase* _testerBase =
      static_cast<STIM300TesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_IMU->clear();
    this->fromPortHistory_packetTime->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: IMU
  // ---------------------------------------------------------------------- 

  void STIM300TesterBase ::
    pushFromPortEntry_IMU(
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
    FromPortEntry_IMU _e = {
      ImuNoCov
    };
    this->fromPortHistory_IMU->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: packetTime
  // ---------------------------------------------------------------------- 

  void STIM300TesterBase ::
    pushFromPortEntry_packetTime(
        Fw::Time &time
    )
  {
    FromPortEntry_packetTime _e = {
      time
    };
    this->fromPortHistory_packetTime->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    from_IMU_handlerBase(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
    FW_ASSERT(portNum < this->getNum_from_IMU(),static_cast<AssertArg>(portNum));
    this->from_IMU_handler(
        portNum,
        ImuNoCov
    );
  }

  void STIM300TesterBase ::
    from_packetTime_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    FW_ASSERT(portNum < this->getNum_from_packetTime(),static_cast<AssertArg>(portNum));
    this->from_packetTime_handler(
        portNum,
        time
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
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

  void STIM300TesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
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

      case STIM300ComponentBase::CHANNELID_NUMPACKETS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing NumPackets: %d\n", _status);
          return;
        }
        this->tlmInput_NumPackets(timeTag, arg);
        break;
      }

      case STIM300ComponentBase::CHANNELID_IMUPACKET:
      {
        ROS::sensor_msgs::ImuNoCov arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing ImuPacket: %d\n", _status);
          return;
        }
        this->tlmInput_ImuPacket(timeTag, arg);
        break;
      }

      case STIM300ComponentBase::CHANNELID_TIMESYNCSTATUS:
      {
        FwEnumStoreType TimeSyncStatusarg;
        const Fw::SerializeStatus _status = val.deserialize(TimeSyncStatusarg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing TimeSyncStatus: %d\n", _status);
          return;
        }
        STIM300ComponentBase::STIM300TimeSync arg = 
          static_cast<STIM300ComponentBase::STIM300TimeSync>(TimeSyncStatusarg);
        this->tlmInput_TimeSyncStatus(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void STIM300TesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_NumPackets->clear();
    this->tlmHistory_ImuPacket->clear();
    this->tlmHistory_TimeSyncStatus->clear();
  }

  // ---------------------------------------------------------------------- 
  // Channel: NumPackets
  // ---------------------------------------------------------------------- 

  void STIM300TesterBase ::
    tlmInput_NumPackets(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_NumPackets e = { timeTag, val };
    this->tlmHistory_NumPackets->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: ImuPacket
  // ---------------------------------------------------------------------- 

  void STIM300TesterBase ::
    tlmInput_ImuPacket(
        const Fw::Time& timeTag,
        const ROS::sensor_msgs::ImuNoCov& val
    )
  {
    TlmEntry_ImuPacket e = { timeTag, val };
    this->tlmHistory_ImuPacket->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: TimeSyncStatus
  // ---------------------------------------------------------------------- 

  void STIM300TesterBase ::
    tlmInput_TimeSyncStatus(
        const Fw::Time& timeTag,
        const STIM300ComponentBase::STIM300TimeSync& val
    )
  {
    TlmEntry_TimeSyncStatus e = { timeTag, val };
    this->tlmHistory_TimeSyncStatus->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
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

      case STIM300ComponentBase::EVENTID_BUFFERFULL: 
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
        this->logIn_WARNING_HI_BufferFull();

        break;

      }

      case STIM300ComponentBase::EVENTID_UARTERROR: 
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
        this->logIn_WARNING_HI_UartError();

        break;

      }

      case STIM300ComponentBase::EVENTID_NOEVENTS: 
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
        this->logIn_WARNING_HI_NoEvents();

        break;

      }

      case STIM300ComponentBase::EVENTID_INVALIDCOUNTER: 
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
        U32 actualCount;
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
        _status = args.deserialize(actualCount);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        U32 exptecCount;
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
        _status = args.deserialize(exptecCount);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_InvalidCounter(actualCount, exptecCount);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void STIM300TesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventsSize_BufferFull = 0;
    this->eventsSize_UartError = 0;
    this->eventsSize_NoEvents = 0;
    this->eventHistory_InvalidCounter->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
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

  void STIM300TesterBase ::
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

  void STIM300TesterBase ::
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
  // Event: BufferFull 
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    logIn_WARNING_HI_BufferFull(
        void
    )
  {
    ++this->eventsSize_BufferFull;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: UartError 
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    logIn_WARNING_HI_UartError(
        void
    )
  {
    ++this->eventsSize_UartError;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: NoEvents 
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    logIn_WARNING_HI_NoEvents(
        void
    )
  {
    ++this->eventsSize_NoEvents;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: InvalidCounter 
  // ----------------------------------------------------------------------

  void STIM300TesterBase ::
    logIn_WARNING_HI_InvalidCounter(
        U32 actualCount,
        U32 exptecCount
    )
  {
    EventEntry_InvalidCounter e = {
      actualCount, exptecCount
    };
    eventHistory_InvalidCounter->push_back(e);
    ++this->eventsSize;
  }

} // end namespace Drv
