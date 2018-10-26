// ======================================================================
// \title  SerialTextConverter/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for SerialTextConverter component test harness base class
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

  SerialTextConverterTesterBase ::
    SerialTextConverterTesterBase(
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
    // Initialize event histories
#if FW_ENABLE_TEXT_LOGGING
    this->textLogHistory = new History<TextLogEntry>(maxHistorySize);
#endif
    this->eventHistory_STC_SerialText =
      new History<EventEntry_STC_SerialText>(maxHistorySize);
    this->eventHistory_STC_SerialReadBadStatus =
      new History<EventEntry_STC_SerialReadBadStatus>(maxHistorySize);
    this->eventHistory_STC_SetMode_Cmd_Sent =
      new History<EventEntry_STC_SetMode_Cmd_Sent>(maxHistorySize);
    this->eventHistory_STC_File =
      new History<EventEntry_STC_File>(maxHistorySize);
    // Initialize histories for typed user output ports
    this->fromPortHistory_SerialBufferSend =
      new History<FromPortEntry_SerialBufferSend>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  SerialTextConverterTesterBase ::
    ~SerialTextConverterTesterBase(void) 
  {
    // Destroy command history
    delete this->cmdResponseHistory;
    // Destroy event histories
#if FW_ENABLE_TEXT_LOGGING
    delete this->textLogHistory;
#endif
    delete this->eventHistory_STC_SerialText;
    delete this->eventHistory_STC_SerialReadBadStatus;
    delete this->eventHistory_STC_SetMode_Cmd_Sent;
    delete this->eventHistory_STC_File;
  }

  void SerialTextConverterTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

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

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_to_SerReadPort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SerReadPort);
  }

  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_from_SerialBufferSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SerialBufferSend);
  }

  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_to_CmdDisp(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_CmdDisp);
  }

  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_from_CmdStatus(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdStatus);
  }

  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_from_CmdReg(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdReg);
  }

  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_from_Time(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Time);
  }

  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_from_Log(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_Log);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  NATIVE_INT_TYPE SerialTextConverterTesterBase ::
    getNum_from_LogText(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogText);
  }
#endif

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    connect_to_SerReadPort(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSerialReadPort *const SerReadPort
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(),static_cast<AssertArg>(portNum));
    this->m_to_SerReadPort[portNum].addCallPort(SerReadPort);
  }

  void SerialTextConverterTesterBase ::
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

  void SerialTextConverterTesterBase ::
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

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool SerialTextConverterTesterBase ::
    isConnected_to_SerReadPort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(), static_cast<AssertArg>(portNum));
    return this->m_to_SerReadPort[portNum].isConnected();
  }

  bool SerialTextConverterTesterBase ::
    isConnected_to_CmdDisp(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_CmdDisp(), static_cast<AssertArg>(portNum));
    return this->m_to_CmdDisp[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputBufferSendPort *SerialTextConverterTesterBase ::
    get_from_SerialBufferSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SerialBufferSend(),static_cast<AssertArg>(portNum));
    return &this->m_from_SerialBufferSend[portNum];
  }

  Fw::InputCmdResponsePort *SerialTextConverterTesterBase ::
    get_from_CmdStatus(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdStatus(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdStatus[portNum];
  }

  Fw::InputCmdRegPort *SerialTextConverterTesterBase ::
    get_from_CmdReg(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdReg(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdReg[portNum];
  }

  Fw::InputTimePort *SerialTextConverterTesterBase ::
    get_from_Time(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Time(),static_cast<AssertArg>(portNum));
    return &this->m_from_Time[portNum];
  }

  Fw::InputLogPort *SerialTextConverterTesterBase ::
    get_from_Log(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_Log(),static_cast<AssertArg>(portNum));
    return &this->m_from_Log[portNum];
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  Fw::InputLogTextPort *SerialTextConverterTesterBase ::
    get_from_LogText(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogText(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogText[portNum];
  }
#endif

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    from_SerialBufferSend_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    FW_ASSERT(callComp);
    SerialTextConverterTesterBase* _testerBase = 
      static_cast<SerialTextConverterTesterBase*>(callComp);
    _testerBase->from_SerialBufferSend_handlerBase(
        portNum,
        fwBuffer
    );
  }

  void SerialTextConverterTesterBase ::
    from_CmdStatus_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
  {
    SerialTextConverterTesterBase* _testerBase =
      static_cast<SerialTextConverterTesterBase*>(component);
    _testerBase->cmdResponseIn(opCode, cmdSeq, response);
  }

  void SerialTextConverterTesterBase ::
    from_CmdReg_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode
    )
  {

  }

  void SerialTextConverterTesterBase ::
    from_Log_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::LogSeverity severity,
        Fw::LogBuffer &args
    )
  {
    SerialTextConverterTesterBase* _testerBase =
      static_cast<SerialTextConverterTesterBase*>(component);
    _testerBase->dispatchEvents(id, timeTag, severity, args);
  }

#if FW_ENABLE_TEXT_LOGGING == 1
  void SerialTextConverterTesterBase ::
    from_LogText_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        Fw::Time &timeTag,
        Fw::TextLogSeverity severity,
        Fw::TextLogString &text
    )
  {
    SerialTextConverterTesterBase* _testerBase =
      static_cast<SerialTextConverterTesterBase*>(component);
    _testerBase->textLogIn(id,timeTag,severity,text);
  }
#endif

  void SerialTextConverterTesterBase ::
    from_Time_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        Fw::Time& time
    )
  {
    SerialTextConverterTesterBase* _testerBase =
      static_cast<SerialTextConverterTesterBase*>(component);
    time = _testerBase->m_testTime;
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_SerialBufferSend->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: SerialBufferSend
  // ---------------------------------------------------------------------- 

  void SerialTextConverterTesterBase ::
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
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
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

  // ----------------------------------------------------------------------
  // Command response handling
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
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
  // Command: STC_SET_MODE
  // ---------------------------------------------------------------------- 

  void SerialTextConverterTesterBase ::
    sendCmd_STC_SET_MODE(
        const NATIVE_INT_TYPE instance,
        const U32 cmdSeq,
        SerialTextConverterComponentBase::StcMode mode
    )
  {

    // Serialize arguments

    Fw::CmdArgBuffer buff;
    Fw::SerializeStatus _status;
    _status = buff.serialize((FwEnumStoreType) mode);
    FW_ASSERT(_status == Fw::FW_SERIALIZE_OK,static_cast<AssertArg>(_status));

    // Call output command port
    
    FwOpcodeType _opcode;
    const U32 idBase = this->getIdBase();
    _opcode = SerialTextConverterComponentBase::OPCODE_STC_SET_MODE + idBase;

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

  
  void SerialTextConverterTesterBase ::
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

  void SerialTextConverterTesterBase ::
    clearHistory()
  {
    this->cmdResponseHistory->clear();
    this->textLogHistory->clear();
    this->clearEvents();
    this->clearFromPortHistory();
  }

  // ----------------------------------------------------------------------
  // Time
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    setTestTime(const Fw::Time& time)
  {
    this->m_testTime = time;
  }

  // ----------------------------------------------------------------------
  // Event dispatch
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
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

      case SerialTextConverterComponentBase::EVENTID_STC_SERIALTEXT: 
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
        Fw::LogStringArg data;
        _status = args.deserialize(data);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_LO_STC_SerialText(data);

        break;

      }

      case SerialTextConverterComponentBase::EVENTID_STC_SERIALREADBADSTATUS: 
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

        this->logIn_WARNING_LO_STC_SerialReadBadStatus(status);

        break;

      }

      case SerialTextConverterComponentBase::EVENTID_STC_SETMODE_CMD_SENT: 
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
        FwEnumStoreType modeInt;
        _status = args.deserialize(modeInt);
        SerialTextConverterComponentBase::StcModeEv mode = static_cast<SerialTextConverterComponentBase::StcModeEv>(modeInt);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_LO_STC_SetMode_Cmd_Sent(mode);

        break;

      }

      case SerialTextConverterComponentBase::EVENTID_STC_SETMODE_CMD_INVALID: 
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
        this->logIn_WARNING_LO_STC_SetMode_Cmd_Invalid();

        break;

      }

      case SerialTextConverterComponentBase::EVENTID_STC_FILE: 
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
        Fw::LogStringArg log_file;
        _status = args.deserialize(log_file);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );

        this->logIn_ACTIVITY_LO_STC_File(log_file);

        break;

      }

      default: {
        FW_ASSERT(0, id);
        break;
      }

    }

  }

  void SerialTextConverterTesterBase ::
    clearEvents(void)
  {
    this->eventsSize = 0;
    this->eventHistory_STC_SerialText->clear();
    this->eventHistory_STC_SerialReadBadStatus->clear();
    this->eventHistory_STC_SetMode_Cmd_Sent->clear();
    this->eventsSize_STC_SetMode_Cmd_Invalid = 0;
    this->eventHistory_STC_File->clear();
  }

#if FW_ENABLE_TEXT_LOGGING

  // ----------------------------------------------------------------------
  // Text events 
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
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

  void SerialTextConverterTesterBase ::
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

  void SerialTextConverterTesterBase ::
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
  // Event: STC_SerialText 
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    logIn_ACTIVITY_LO_STC_SerialText(
        Fw::LogStringArg& data
    )
  {
    EventEntry_STC_SerialText e = {
      data
    };
    eventHistory_STC_SerialText->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: STC_SerialReadBadStatus 
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    logIn_WARNING_LO_STC_SerialReadBadStatus(
        I32 status
    )
  {
    EventEntry_STC_SerialReadBadStatus e = {
      status
    };
    eventHistory_STC_SerialReadBadStatus->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: STC_SetMode_Cmd_Sent 
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    logIn_ACTIVITY_LO_STC_SetMode_Cmd_Sent(
        SerialTextConverterComponentBase::StcModeEv mode
    )
  {
    EventEntry_STC_SetMode_Cmd_Sent e = {
      mode
    };
    eventHistory_STC_SetMode_Cmd_Sent->push_back(e);
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: STC_SetMode_Cmd_Invalid 
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    logIn_WARNING_LO_STC_SetMode_Cmd_Invalid(
        void
    )
  {
    ++this->eventsSize_STC_SetMode_Cmd_Invalid;
    ++this->eventsSize;
  }

  // ----------------------------------------------------------------------
  // Event: STC_File 
  // ----------------------------------------------------------------------

  void SerialTextConverterTesterBase ::
    logIn_ACTIVITY_LO_STC_File(
        Fw::LogStringArg& log_file
    )
  {
    EventEntry_STC_File e = {
      log_file
    };
    eventHistory_STC_File->push_back(e);
    ++this->eventsSize;
  }

} // end namespace Svc
