// ======================================================================
// \title  HexRouter/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for HexRouter component test harness base class
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

  HexRouterTesterBase ::
    HexRouterTesterBase(
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
    this->tlmHistory_HR_ResyncErrors = 
      new History<TlmEntry_HR_ResyncErrors>(maxHistorySize);
    this->tlmHistory_HR_NumPackets = 
      new History<TlmEntry_HR_NumPackets>(maxHistorySize);
    this->tlmHistory_HR_NumDecodeErrors = 
      new History<TlmEntry_HR_NumDecodeErrors>(maxHistorySize);
    this->tlmHistory_HR_NumInvalidPorts = 
      new History<TlmEntry_HR_NumInvalidPorts>(maxHistorySize);
    this->tlmHistory_HR_NumBadSerialPortCalls = 
      new History<TlmEntry_HR_NumBadSerialPortCalls>(maxHistorySize);
    this->tlmHistory_HR_NumOutputBufferOverflows = 
      new History<TlmEntry_HR_NumOutputBufferOverflows>(maxHistorySize);
    this->tlmHistory_HR_NumZeroPktSize = 
      new History<TlmEntry_HR_NumZeroPktSize>(maxHistorySize);
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_HR_DataReceiveError =
      new History<EventEntry_HR_DataReceiveError>(maxHistorySize);
    this->eventHistory_HR_InvalidPortNum =
      new History<EventEntry_HR_InvalidPortNum>(maxHistorySize);
    this->eventHistory_HR_BadSerialPortCall =
      new History<EventEntry_HR_BadSerialPortCall>(maxHistorySize);
    this->eventHistory_HR_MemoryError =
      new History<EventEntry_HR_MemoryError>(maxHistorySize);
    this->eventHistory_HR_WriteError =
      new History<EventEntry_HR_WriteError>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_readBufferSend =
      new History<FromPortEntry_readBufferSend>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  HexRouterTesterBase ::
    ~HexRouterTesterBase(void) 
  {
    // Destroy telemetry histories
    delete this->tlmHistory_HR_ResyncErrors;
    delete this->tlmHistory_HR_NumPackets;
    delete this->tlmHistory_HR_NumDecodeErrors;
    delete this->tlmHistory_HR_NumInvalidPorts;
    delete this->tlmHistory_HR_NumBadSerialPortCalls;
    delete this->tlmHistory_HR_NumOutputBufferOverflows;
    delete this->tlmHistory_HR_NumZeroPktSize;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_HR_DataReceiveError;
    delete this->eventHistory_HR_InvalidPortNum;
    delete this->eventHistory_HR_BadSerialPortCall;
    delete this->eventHistory_HR_MemoryError;
    delete this->eventHistory_HR_WriteError;
  }

  void HexRouterTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port HexPortsOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_HexPortsOut();
        ++_port
    ) {

      this->m_from_HexPortsOut[_port].init();
      this->m_from_HexPortsOut[_port].addCallComp(
          this,
          from_HexPortsOut_static
      );
      this->m_from_HexPortsOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_HexPortsOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_HexPortsOut[_port].setObjName(_portName);
#endif

    }

    // Attach input port readBufferSend

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_readBufferSend();
        ++_port
    ) {

      this->m_from_readBufferSend[_port].init();
      this->m_from_readBufferSend[_port].addCallComp(
          this,
          from_readBufferSend_static
      );
      this->m_from_readBufferSend[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_readBufferSend[%d]",
          this->m_objName,
          _port
      );
      this->m_from_readBufferSend[_port].setObjName(_portName);
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

    // Initialize output port KraitPortsIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_KraitPortsIn();
        ++_port
    ) {
      this->m_to_KraitPortsIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_KraitPortsIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_KraitPortsIn[_port].setObjName(_portName);
#endif

    }

    // Initialize output port readBufferRecv

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_readBufferRecv();
        ++_port
    ) {
      this->m_to_readBufferRecv[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_readBufferRecv[%d]",
          this->m_objName,
          _port
      );
      this->m_to_readBufferRecv[_port].setObjName(_portName);
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

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_to_KraitPortsIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_KraitPortsIn);
  }

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_from_HexPortsOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_HexPortsOut);
  }

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_from_readBufferSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_readBufferSend);
  }

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_to_readBufferRecv(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_readBufferRecv);
  }

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_to_Sched(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Sched);
  }

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_from_Tlm(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Tlm);
  }

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE HexRouterTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    connect_to_KraitPortsIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputSerializePort *const KraitPortsIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_KraitPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_KraitPortsIn[portNum].registerSerialPort(KraitPortsIn);
  }

  void HexRouterTesterBase ::
    connect_to_readBufferRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::InputBufferSendPort *const readBufferRecv
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_readBufferRecv(),static_cast<AssertArg>(portNum));
    this->m_to_readBufferRecv[portNum].addCallPort(readBufferRecv);
  }

  void HexRouterTesterBase ::
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

  void HexRouterTesterBase ::
    invoke_to_KraitPortsIn(
      NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_KraitPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_KraitPortsIn[portNum].invokeSerial(Buffer);
  }

  void HexRouterTesterBase ::
    invoke_to_readBufferRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_readBufferRecv(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_readBufferRecv(),static_cast<AssertArg>(portNum));
    this->m_to_readBufferRecv[portNum].invoke(
        fwBuffer
    );
  }

  void HexRouterTesterBase ::
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

  bool HexRouterTesterBase ::
    isConnected_to_KraitPortsIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_KraitPortsIn(), static_cast<AssertArg>(portNum));
    return this->m_to_KraitPortsIn[portNum].isConnected();
  }

  bool HexRouterTesterBase ::
    isConnected_to_readBufferRecv(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_readBufferRecv(), static_cast<AssertArg>(portNum));
    return this->m_to_readBufferRecv[portNum].isConnected();
  }

  bool HexRouterTesterBase ::
    isConnected_to_Sched(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(), static_cast<AssertArg>(portNum));
    return this->m_to_Sched[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputSerializePort *HexRouterTesterBase ::
    get_from_HexPortsOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_HexPortsOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_HexPortsOut[portNum];
  }

  Fw::InputBufferSendPort *HexRouterTesterBase ::
    get_from_readBufferSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_readBufferSend(),static_cast<AssertArg>(portNum));
    return &this->m_from_readBufferSend[portNum];
  }

  Fw::InputTlmPort *HexRouterTesterBase ::
    get_from_Tlm(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Tlm(),static_cast<AssertArg>(portNum));
    return &this->m_from_Tlm[portNum];
  }

  Fw::InputTimePort *HexRouterTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *HexRouterTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *HexRouterTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    from_readBufferSend_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(callComp);
    HexRouterTesterBase* _testerBase = 
      static_cast<HexRouterTesterBase*>(callComp);
    _testerBase->from_readBufferSend_handlerBase(
        portNum,
        fwBuffer
    );
  }

  void HexRouterTesterBase ::
    from_HexPortsOut_static(
      Fw::PassiveComponentBase *const callComp, //!< The component instance
      const NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer //!< serialized data buffer
    )
  {
    FW_ASSERT(callComp);
    HexRouterTesterBase* _testerBase = 
      static_cast<HexRouterTesterBase*>(callComp);

    _testerBase->from_HexPortsOut_handlerBase(
        portNum,
        Buffer
    );
  }  

  void HexRouterTesterBase ::
    from_HexPortsOut_handlerBase(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    FW_ASSERT(portNum < this->getNum_from_HexPortsOut(),static_cast<AssertArg>(portNum));
    this->from_HexPortsOut_handler(
        portNum,
        Buffer
    );
  } 
   
  void HexRouterTesterBase ::
    from_Tlm_static(
        Fw::PassiveComponentBase *const component,
        NATIVE_INT_TYPE portNum,
        FwChanIdType id,
        Fw::Time &timeTag,
        Fw::TlmBuffer &val
    )
  {
    HexRouterTesterBase* _testerBase =
      static_cast<HexRouterTesterBase*>(component);
    _testerBase->dispatchTlm(id, timeTag, val);
  }

  void HexRouterTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    HexRouterTesterBase* _testerBase =
      static_cast<HexRouterTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void HexRouterTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    HexRouterTesterBase* _testerBase =
      static_cast<HexRouterTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void HexRouterTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    HexRouterTesterBase* _testerBase =
      static_cast<HexRouterTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_readBufferSend->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: readBufferSend
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    pushFromPortEntry_readBufferSend(
        Fw::Buffer fwBuffer
    )
  {
    FromPortEntry_readBufferSend _e = {
      fwBuffer
    };
    this->fromPortHistory_readBufferSend->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    from_readBufferSend_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_readBufferSend(),static_cast<AssertArg>(portNum));
    this->from_readBufferSend_handler(
        portNum,
        fwBuffer
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
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

  void HexRouterTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Telemetry dispatch
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
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

      case HexRouterComponentBase::CHANNELID_HR_RESYNCERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HR_ResyncErrors: %d\n", _status);
          return;
        }
        this->tlmInput_HR_ResyncErrors(timeTag, arg);
        break;
      }

      case HexRouterComponentBase::CHANNELID_HR_NUMPACKETS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HR_NumPackets: %d\n", _status);
          return;
        }
        this->tlmInput_HR_NumPackets(timeTag, arg);
        break;
      }

      case HexRouterComponentBase::CHANNELID_HR_NUMDECODEERRORS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HR_NumDecodeErrors: %d\n", _status);
          return;
        }
        this->tlmInput_HR_NumDecodeErrors(timeTag, arg);
        break;
      }

      case HexRouterComponentBase::CHANNELID_HR_NUMINVALIDPORTS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HR_NumInvalidPorts: %d\n", _status);
          return;
        }
        this->tlmInput_HR_NumInvalidPorts(timeTag, arg);
        break;
      }

      case HexRouterComponentBase::CHANNELID_HR_NUMBADSERIALPORTCALLS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HR_NumBadSerialPortCalls: %d\n", _status);
          return;
        }
        this->tlmInput_HR_NumBadSerialPortCalls(timeTag, arg);
        break;
      }

      case HexRouterComponentBase::CHANNELID_HR_NUMOUTPUTBUFFEROVERFLOWS:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HR_NumOutputBufferOverflows: %d\n", _status);
          return;
        }
        this->tlmInput_HR_NumOutputBufferOverflows(timeTag, arg);
        break;
      }

      case HexRouterComponentBase::CHANNELID_HR_NUMZEROPKTSIZE:
      {
        U32 arg;
        const Fw::SerializeStatus _status = val.deserialize(arg);
        if (_status != Fw::FW_SERIALIZE_OK) {
          printf("Error deserializing HR_NumZeroPktSize: %d\n", _status);
          return;
        }
        this->tlmInput_HR_NumZeroPktSize(timeTag, arg);
        break;
      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void HexRouterTesterBase ::
    clearTlm(void)
  {
    this->tlmSize = 0;
    this->tlmHistory_HR_ResyncErrors->clear();
    this->tlmHistory_HR_NumPackets->clear();
    this->tlmHistory_HR_NumDecodeErrors->clear();
    this->tlmHistory_HR_NumInvalidPorts->clear();
    this->tlmHistory_HR_NumBadSerialPortCalls->clear();
    this->tlmHistory_HR_NumOutputBufferOverflows->clear();
    this->tlmHistory_HR_NumZeroPktSize->clear();
  }

  // ---------------------------------------------------------------------- 
  // Channel: HR_ResyncErrors
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    tlmInput_HR_ResyncErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_HR_ResyncErrors e = { timeTag, val };
    this->tlmHistory_HR_ResyncErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: HR_NumPackets
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    tlmInput_HR_NumPackets(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_HR_NumPackets e = { timeTag, val };
    this->tlmHistory_HR_NumPackets->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: HR_NumDecodeErrors
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    tlmInput_HR_NumDecodeErrors(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_HR_NumDecodeErrors e = { timeTag, val };
    this->tlmHistory_HR_NumDecodeErrors->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: HR_NumInvalidPorts
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    tlmInput_HR_NumInvalidPorts(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_HR_NumInvalidPorts e = { timeTag, val };
    this->tlmHistory_HR_NumInvalidPorts->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: HR_NumBadSerialPortCalls
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    tlmInput_HR_NumBadSerialPortCalls(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_HR_NumBadSerialPortCalls e = { timeTag, val };
    this->tlmHistory_HR_NumBadSerialPortCalls->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: HR_NumOutputBufferOverflows
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    tlmInput_HR_NumOutputBufferOverflows(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_HR_NumOutputBufferOverflows e = { timeTag, val };
    this->tlmHistory_HR_NumOutputBufferOverflows->push_back(e);
    ++this->tlmSize;
  }

  // ---------------------------------------------------------------------- 
  // Channel: HR_NumZeroPktSize
  // ---------------------------------------------------------------------- 

  void HexRouterTesterBase ::
    tlmInput_HR_NumZeroPktSize(
        const Fw::Time& timeTag,
        const U32& val
    )
  {
    TlmEntry_HR_NumZeroPktSize e = { timeTag, val };
    this->tlmHistory_HR_NumZeroPktSize->push_back(e);
    ++this->tlmSize;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
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

      case HexRouterComponentBase::EVENTID_HR_DATARECEIVEERROR: 
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
        HexRouterComponentBase::HR_ReceiveError error = static_cast<HexRouterComponentBase::HR_ReceiveError>(errorInt);
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

        this->logIn_WARNING_HI_HR_DataReceiveError(error, stat);

        break;

      }

      case HexRouterComponentBase::EVENTID_HR_INVALIDPORTNUM: 
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
        HexRouterComponentBase::HR_PortType PortType = static_cast<HexRouterComponentBase::HR_PortType>(PortTypeInt);
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

        this->logIn_WARNING_HI_HR_InvalidPortNum(PortType, portNum);

        break;

      }

      case HexRouterComponentBase::EVENTID_HR_BADSERIALPORTCALL: 
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

        this->logIn_WARNING_HI_HR_BadSerialPortCall(status, portNum);

        break;

      }

      case HexRouterComponentBase::EVENTID_HR_MEMORYERROR: 
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
        HexRouterComponentBase::HR_MemoryError error = static_cast<HexRouterComponentBase::HR_MemoryError>(errorInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_WARNING_HI_HR_MemoryError(error);

        break;

      }

      case HexRouterComponentBase::EVENTID_HR_WRITEERROR: 
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

        this->logIn_WARNING_HI_HR_WriteError(error);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void HexRouterTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_HR_DataReceiveError->clear();
    this->eventHistory_HR_InvalidPortNum->clear();
    this->eventHistory_HR_BadSerialPortCall->clear();
    this->eventHistory_HR_MemoryError->clear();
    this->eventHistory_HR_WriteError->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
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

  void HexRouterTesterBase ::
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

  void HexRouterTesterBase ::
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
  // Event: HR_DataReceiveError 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    logIn_WARNING_HI_HR_DataReceiveError(
        HexRouterComponentBase::HR_ReceiveError error,
        I32 stat
    )
  {
    EventEntry_HR_DataReceiveError e = {
      error, stat
    };
    eventHistory_HR_DataReceiveError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: HR_InvalidPortNum 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    logIn_WARNING_HI_HR_InvalidPortNum(
        HexRouterComponentBase::HR_PortType PortType,
        U8 portNum
    )
  {
    EventEntry_HR_InvalidPortNum e = {
      PortType, portNum
    };
    eventHistory_HR_InvalidPortNum->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: HR_BadSerialPortCall 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    logIn_WARNING_HI_HR_BadSerialPortCall(
        I32 status,
        U8 portNum
    )
  {
    EventEntry_HR_BadSerialPortCall e = {
      status, portNum
    };
    eventHistory_HR_BadSerialPortCall->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: HR_MemoryError 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    logIn_WARNING_HI_HR_MemoryError(
        HexRouterComponentBase::HR_MemoryError error
    )
  {
    EventEntry_HR_MemoryError e = {
      error
    };
    eventHistory_HR_MemoryError->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: HR_WriteError 
  // ----------------------------------------------------------------------

  void HexRouterTesterBase ::
    logIn_WARNING_HI_HR_WriteError(
        I32 error
    )
  {
    EventEntry_HR_WriteError e = {
      error
    };
    eventHistory_HR_WriteError->push_back(e);
    ++this->eventsSize;
  }

} // end namespace SnapdragonFlight
