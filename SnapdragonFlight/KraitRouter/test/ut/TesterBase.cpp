// ======================================================================
// \title  KraitRouter/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for KraitRouter component test harness base class
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

  KraitRouterTesterBase ::
    KraitRouterTesterBase(
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
    this->tlmHistory_KR_ResyncErrors = 
      new History<TlmEntry_KR_ResyncErrors>(maxHistorySize);
    this->tlmHistory_KR_NumPackets = 
      new History<TlmEntry_KR_NumPackets>(maxHistorySize);
    this->tlmHistory_KR_NumDecodeErrors = 
      new History<TlmEntry_KR_NumDecodeErrors>(maxHistorySize);
    this->tlmHistory_KR_NumInvalidPorts = 
      new History<TlmEntry_KR_NumInvalidPorts>(maxHistorySize);
    this->tlmHistory_KR_NumBadSerialPortCalls = 
      new History<TlmEntry_KR_NumBadSerialPortCalls>(maxHistorySize);
    this->tlmHistory_KR_NumOutputBufferOverflows = 
      new History<TlmEntry_KR_NumOutputBufferOverflows>(maxHistorySize);
    this->tlmHistory_KR_NumZeroPktSize = 
      new History<TlmEntry_KR_NumZeroPktSize>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_KR_DataReceiveError =
      new History<EventEntry_KR_DataReceiveError>(maxHistorySize);
    this->eventHistory_KR_InvalidPortNum =
      new History<EventEntry_KR_InvalidPortNum>(maxHistorySize);
    this->eventHistory_KR_BadSerialPortCall =
      new History<EventEntry_KR_BadSerialPortCall>(maxHistorySize);
    this->eventHistory_KR_MemoryError =
      new History<EventEntry_KR_MemoryError>(maxHistorySize);
    this->eventHistory_KR_WriteError =
      new History<EventEntry_KR_WriteError>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  KraitRouterTesterBase ::
    ~KraitRouterTesterBase(void) 
  {
    // Destroy telemetry histories
    delete this->tlmHistory_KR_ResyncErrors;
    delete this->tlmHistory_KR_NumPackets;
    delete this->tlmHistory_KR_NumDecodeErrors;
    delete this->tlmHistory_KR_NumInvalidPorts;
    delete this->tlmHistory_KR_NumBadSerialPortCalls;
    delete this->tlmHistory_KR_NumOutputBufferOverflows;
    delete this->tlmHistory_KR_NumZeroPktSize;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_KR_DataReceiveError;
    delete this->eventHistory_KR_InvalidPortNum;
    delete this->eventHistory_KR_BadSerialPortCall;
    delete this->eventHistory_KR_MemoryError;
    delete this->eventHistory_KR_WriteError;
  }

  void KraitRouterTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port KraitPortsOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_KraitPortsOut();
        ++_port
    ) {

      this->m_from_KraitPortsOut[_port].init();
      this->m_from_KraitPortsOut[_port].addCallComp(
          this,
          from_KraitPortsOut_static
      );
      this->m_from_KraitPortsOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_KraitPortsOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_KraitPortsOut[_port].setObjName(_portName);
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

    // Initialize output port HexPortsIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_HexPortsIn();
        ++_port
    ) {
      this->m_to_HexPortsIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_HexPortsIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_HexPortsIn[_port].setObjName(_portName);
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

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE KraitRouterTesterBase ::
    getNum_to_HexPortsIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_HexPortsIn);
  }

  NATIVE_INT_TYPE KraitRouterTesterBase ::
    getNum_from_KraitPortsOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_KraitPortsOut);
  }

  NATIVE_INT_TYPE KraitRouterTesterBase ::
    getNum_to_Sched(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Sched);
  }

  NATIVE_INT_TYPE KraitRouterTesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE KraitRouterTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE KraitRouterTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE KraitRouterTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    connect_to_HexPortsIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputSerializePort *const HexPortsIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_HexPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_HexPortsIn[portNum].registerSerialPort(HexPortsIn);
  }

  void KraitRouterTesterBase ::
    connect_to_Sched(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const Sched
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(),static_cast<AssertArg>(portNum));
    this->m_to_Sched[portNum].addCallPort(Sched);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    invoke_to_HexPortsIn(
      NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_HexPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_HexPortsIn[portNum].invokeSerial(Buffer);
  }

  void KraitRouterTesterBase ::
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

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool KraitRouterTesterBase ::
    isConnected_to_HexPortsIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_HexPortsIn(), static_cast<AssertArg>(portNum));
    return this->m_to_HexPortsIn[portNum].isConnected();
  }

  bool KraitRouterTesterBase ::
    isConnected_to_Sched(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(), static_cast<AssertArg>(portNum));
    return this->m_to_Sched[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputSerializePort *KraitRouterTesterBase ::
    get_from_KraitPortsOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_KraitPortsOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_KraitPortsOut[portNum];
  }

  Fw::InputTlmPort *KraitRouterTesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputTimePort *KraitRouterTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *KraitRouterTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *KraitRouterTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    from_KraitPortsOut_static(
      Fw::PassiveComponentBase *const callComp, //!< The component instance
      const NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer //!< serialized data buffer
    )
  {
    FW_ASSERT(callComp);
    KraitRouterTesterBase* _testerBase = 
      static_cast<KraitRouterTesterBase*>(callComp);

    _testerBase->from_KraitPortsOut_handlerBase(
        portNum,
        Buffer
    );
  }  

  void KraitRouterTesterBase ::
    from_KraitPortsOut_handlerBase(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    FW_ASSERT(portNum < this->getNum_from_KraitPortsOut(),static_cast<AssertArg>(portNum));
    this->from_KraitPortsOut_handler(
        portNum,
        Buffer
    );
  } 
   
  void KraitRouterTesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    KraitRouterTesterBase* _testerBase =
      static_cast<KraitRouterTesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void KraitRouterTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    KraitRouterTesterBase* _testerBase =
      static_cast<KraitRouterTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void KraitRouterTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    KraitRouterTesterBase* _testerBase =
      static_cast<KraitRouterTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void KraitRouterTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    KraitRouterTesterBase* _testerBase =
      static_cast<KraitRouterTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    clearHistory()
  {
    this->clearTlm();
    this->textLogHistory->clear();
    this->clearEvents();
  }

  // ----------------------------------------------------------------------
  // Time
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
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

      case KraitRouterComponentBase::CHANNELID_KR_RESYNCERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing KR_ResyncErrors: %d\n", _status);
          return;
        }
        this->tlmInput_KR_ResyncErrors(timeTag, arg);
        break;
      }

      case KraitRouterComponentBase::CHANNELID_KR_NUMPACKETS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing KR_NumPackets: %d\n", _status);
          return;
        }
        this->tlmInput_KR_NumPackets(timeTag, arg);
        break;
      }

      case KraitRouterComponentBase::CHANNELID_KR_NUMDECODEERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing KR_NumDecodeErrors: %d\n", _status);
          return;
        }
        this->tlmInput_KR_NumDecodeErrors(timeTag, arg);
        break;
      }

      case KraitRouterComponentBase::CHANNELID_KR_NUMINVALIDPORTS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing KR_NumInvalidPorts: %d\n", _status);
          return;
        }
        this->tlmInput_KR_NumInvalidPorts(timeTag, arg);
        break;
      }

      case KraitRouterComponentBase::CHANNELID_KR_NUMBADSERIALPORTCALLS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing KR_NumBadSerialPortCalls: %d\n", _status);
          return;
        }
        this->tlmInput_KR_NumBadSerialPortCalls(timeTag, arg);
        break;
      }

      case KraitRouterComponentBase::CHANNELID_KR_NUMOUTPUTBUFFEROVERFLOWS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing KR_NumOutputBufferOverflows: %d\n", _status);
          return;
        }
        this->tlmInput_KR_NumOutputBufferOverflows(timeTag, arg);
        break;
      }

      case KraitRouterComponentBase::CHANNELID_KR_NUMZEROPKTSIZE:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing KR_NumZeroPktSize: %d\n", _status);
          return;
        }
        this->tlmInput_KR_NumZeroPktSize(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void KraitRouterTesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_KR_ResyncErrors->clear();
    this->tlmHistory_KR_NumPackets->clear();
    this->tlmHistory_KR_NumDecodeErrors->clear();
    this->tlmHistory_KR_NumInvalidPorts->clear();
    this->tlmHistory_KR_NumBadSerialPortCalls->clear();
    this->tlmHistory_KR_NumOutputBufferOverflows->clear();
    this->tlmHistory_KR_NumZeroPktSize->clear();
  }

  // ---------------------------------------------------------------------- 
  // Channel: KR_ResyncErrors
  // ---------------------------------------------------------------------- 

  void KraitRouterTesterBase ::
    tlmInput_KR_ResyncErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_KR_ResyncErrors e = { timeTag, val };
    this->tlmHistory_KR_ResyncErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: KR_NumPackets
  // ---------------------------------------------------------------------- 

  void KraitRouterTesterBase ::
    tlmInput_KR_NumPackets(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_KR_NumPackets e = { timeTag, val };
    this->tlmHistory_KR_NumPackets->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: KR_NumDecodeErrors
  // ---------------------------------------------------------------------- 

  void KraitRouterTesterBase ::
    tlmInput_KR_NumDecodeErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_KR_NumDecodeErrors e = { timeTag, val };
    this->tlmHistory_KR_NumDecodeErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: KR_NumInvalidPorts
  // ---------------------------------------------------------------------- 

  void KraitRouterTesterBase ::
    tlmInput_KR_NumInvalidPorts(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_KR_NumInvalidPorts e = { timeTag, val };
    this->tlmHistory_KR_NumInvalidPorts->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: KR_NumBadSerialPortCalls
  // ---------------------------------------------------------------------- 

  void KraitRouterTesterBase ::
    tlmInput_KR_NumBadSerialPortCalls(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_KR_NumBadSerialPortCalls e = { timeTag, val };
    this->tlmHistory_KR_NumBadSerialPortCalls->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: KR_NumOutputBufferOverflows
  // ---------------------------------------------------------------------- 

  void KraitRouterTesterBase ::
    tlmInput_KR_NumOutputBufferOverflows(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_KR_NumOutputBufferOverflows e = { timeTag, val };
    this->tlmHistory_KR_NumOutputBufferOverflows->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: KR_NumZeroPktSize
  // ---------------------------------------------------------------------- 

  void KraitRouterTesterBase ::
    tlmInput_KR_NumZeroPktSize(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_KR_NumZeroPktSize e = { timeTag, val };
    this->tlmHistory_KR_NumZeroPktSize->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
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

      case KraitRouterComponentBase::EVENTID_KR_DATARECEIVEERROR: 
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
        KraitRouterComponentBase::KR_ReceiveError error = static_cast<KraitRouterComponentBase::KR_ReceiveError>(errorInt);
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

        this->logIn_WARNING_HI_KR_DataReceiveError(error, stat);

        break;

      }

      case KraitRouterComponentBase::EVENTID_KR_INVALIDPORTNUM: 
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
        FwEnumStoreType PortTypeInt;
        _status = args.deserialize(PortTypeInt);
        KraitRouterComponentBase::KR_PortType PortType = static_cast<KraitRouterComponentBase::KR_PortType>(PortTypeInt);
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

        this->logIn_WARNING_HI_KR_InvalidPortNum(PortType, portNum);

        break;

      }

      case KraitRouterComponentBase::EVENTID_KR_BADSERIALPORTCALL: 
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

        this->logIn_WARNING_HI_KR_BadSerialPortCall(status, portNum);

        break;

      }

      case KraitRouterComponentBase::EVENTID_KR_MEMORYERROR: 
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
        KraitRouterComponentBase::KR_MemoryError error = static_cast<KraitRouterComponentBase::KR_MemoryError>(errorInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_KR_MemoryError(error);

        break;

      }

      case KraitRouterComponentBase::EVENTID_KR_WRITEERROR: 
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

        this->logIn_WARNING_HI_KR_WriteError(error);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void KraitRouterTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_KR_DataReceiveError->clear();
    this->eventHistory_KR_InvalidPortNum->clear();
    this->eventHistory_KR_BadSerialPortCall->clear();
    this->eventHistory_KR_MemoryError->clear();
    this->eventHistory_KR_WriteError->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
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

  void KraitRouterTesterBase ::
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

  void KraitRouterTesterBase ::
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
  // Event: KR_DataReceiveError 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    logIn_WARNING_HI_KR_DataReceiveError(
        KraitRouterComponentBase::KR_ReceiveError error,
        I32 stat
    )
  {
    EventEntry_KR_DataReceiveError e = {
      error, stat
    };
    eventHistory_KR_DataReceiveError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: KR_InvalidPortNum 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    logIn_WARNING_HI_KR_InvalidPortNum(
        KraitRouterComponentBase::KR_PortType PortType,
        U8 portNum
    )
  {
    EventEntry_KR_InvalidPortNum e = {
      PortType, portNum
    };
    eventHistory_KR_InvalidPortNum->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: KR_BadSerialPortCall 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    logIn_WARNING_HI_KR_BadSerialPortCall(
        I32 status,
        U8 portNum
    )
  {
    EventEntry_KR_BadSerialPortCall e = {
      status, portNum
    };
    eventHistory_KR_BadSerialPortCall->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: KR_MemoryError 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    logIn_WARNING_HI_KR_MemoryError(
        KraitRouterComponentBase::KR_MemoryError error
    )
  {
    EventEntry_KR_MemoryError e = {
      error
    };
    eventHistory_KR_MemoryError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: KR_WriteError 
  // ----------------------------------------------------------------------

  void KraitRouterTesterBase ::
    logIn_WARNING_HI_KR_WriteError(
        I32 error
    )
  {
    EventEntry_KR_WriteError e = {
      error
    };
    eventHistory_KR_WriteError->push_back(e);
    ++this->eventsSize;
  }

} // end namespace SnapdragonFlight
