// ======================================================================
// \title  FlatOutputAdapter/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for FlatOutputAdapter component test harness base class
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

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  FlatOutputAdapterTesterBase ::
    FlatOutputAdapterTesterBase(
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
    // Initialize histories for typed user output ports
    this->fromPortHistory_flatOutput =
      new History<FromPortEntry_flatOutput>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  FlatOutputAdapterTesterBase ::
    ~FlatOutputAdapterTesterBase(void)
  {
    // Destroy command history
    delete this->cmdResponseHistory;
  }

  void FlatOutputAdapterTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    this->m_param_w_x_valid = Fw::PARAM_UNINIT;
    this->m_param_w_y_valid = Fw::PARAM_UNINIT;
    this->m_param_w_z_valid = Fw::PARAM_UNINIT;

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port flatOutput

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_flatOutput();
        ++_port
    ) {

      this->m_from_flatOutput[_port].init();
      this->m_from_flatOutput[_port].addCallComp(
          this,
          from_flatOutput_static
      );
      this->m_from_flatOutput[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_flatOutput[%d]",
          this->m_objName,
          _port
      );
      this->m_from_flatOutput[_port].setObjName(_portName);
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

    // Attach input port ParamGet

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_ParamGet();
        ++_port
    ) {

      this->m_from_ParamGet[_port].init();
      this->m_from_ParamGet[_port].addCallComp(
          this,
          from_ParamGet_static
      );
      this->m_from_ParamGet[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_ParamGet[%d]",
          this->m_objName,
          _port
      );
      this->m_from_ParamGet[_port].setObjName(_portName);
#endif

    }

    // Attach input port ParamSet

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_ParamSet();
        ++_port
    ) {

      this->m_from_ParamSet[_port].init();
      this->m_from_ParamSet[_port].addCallComp(
          this,
          from_ParamSet_static
      );
      this->m_from_ParamSet[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_ParamSet[%d]",
          this->m_objName,
          _port
      );
      this->m_from_ParamSet[_port].setObjName(_portName);
#endif

    }

    // Initialize output port se3Cmd

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_se3Cmd();
        ++_port
    ) {
      this->m_to_se3Cmd[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_se3Cmd[%d]",
          this->m_objName,
          _port
      );
      this->m_to_se3Cmd[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE FlatOutputAdapterTesterBase ::
    getNum_to_se3Cmd(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_se3Cmd);
  }

  NATIVE_INT_TYPE FlatOutputAdapterTesterBase ::
    getNum_from_flatOutput(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_flatOutput);
  }

  NATIVE_INT_TYPE FlatOutputAdapterTesterBase ::
    getNum_to_CmdDisp(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_CmdDisp);
  }

  NATIVE_INT_TYPE FlatOutputAdapterTesterBase ::
    getNum_from_CmdStatus(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdStatus);
  }

  NATIVE_INT_TYPE FlatOutputAdapterTesterBase ::
    getNum_from_CmdReg(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_CmdReg);
  }

  NATIVE_INT_TYPE FlatOutputAdapterTesterBase ::
    getNum_from_ParamGet(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_ParamGet);
  }

  NATIVE_INT_TYPE FlatOutputAdapterTesterBase ::
    getNum_from_ParamSet(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_ParamSet);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    connect_to_se3Cmd(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::InputSe3FeedForwardPort *const se3Cmd
    )
  {
    FW_ASSERT(portNum < this->getNum_to_se3Cmd(),static_cast<AssertArg>(portNum));
    this->m_to_se3Cmd[portNum].addCallPort(se3Cmd);
  }

  void FlatOutputAdapterTesterBase ::
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

  void FlatOutputAdapterTesterBase ::
    invoke_to_se3Cmd(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Se3FeedForward &Se3FeedForward
    )
  {
    FW_ASSERT(portNum < this->getNum_to_se3Cmd(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_se3Cmd(),static_cast<AssertArg>(portNum));
    this->m_to_se3Cmd[portNum].invoke(
        Se3FeedForward
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool FlatOutputAdapterTesterBase ::
    isConnected_to_se3Cmd(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_se3Cmd(), static_cast<AssertArg>(portNum));
    return this->m_to_se3Cmd[portNum].isConnected();
  }

  bool FlatOutputAdapterTesterBase ::
    isConnected_to_CmdDisp(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_CmdDisp(), static_cast<AssertArg>(portNum));
    return this->m_to_CmdDisp[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------

  ROS::mav_msgs::InputFlatOutputPort *FlatOutputAdapterTesterBase ::
    get_from_flatOutput(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_flatOutput(),static_cast<AssertArg>(portNum));
    return &this->m_from_flatOutput[portNum];
  }

  Fw::InputCmdResponsePort *FlatOutputAdapterTesterBase ::
    get_from_CmdStatus(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdStatus(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdStatus[portNum];
  }

  Fw::InputCmdRegPort *FlatOutputAdapterTesterBase ::
    get_from_CmdReg(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_CmdReg(),static_cast<AssertArg>(portNum));
    return &this->m_from_CmdReg[portNum];
  }

  Fw::InputPrmGetPort *FlatOutputAdapterTesterBase ::
    get_from_ParamGet(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_ParamGet(),static_cast<AssertArg>(portNum));
    return &this->m_from_ParamGet[portNum];
  }

  Fw::InputPrmSetPort *FlatOutputAdapterTesterBase ::
    get_from_ParamSet(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_ParamSet(),static_cast<AssertArg>(portNum));
    return &this->m_from_ParamSet[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    from_flatOutput_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    FW_ASSERT(callComp);
    FlatOutputAdapterTesterBase* _testerBase =
      static_cast<FlatOutputAdapterTesterBase*>(callComp);
    _testerBase->from_flatOutput_handlerBase(
        portNum,
        FlatOutput
    );
  }

  void FlatOutputAdapterTesterBase ::
    from_CmdStatus_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        const Fw::CommandResponse response
    )
  {
    FlatOutputAdapterTesterBase* _testerBase =
      static_cast<FlatOutputAdapterTesterBase*>(component);
    _testerBase->cmdResponseIn(opCode, cmdSeq, response);
  }

  void FlatOutputAdapterTesterBase ::
    from_CmdReg_static(
        Fw::PassiveComponentBase *const component,
        const NATIVE_INT_TYPE portNum,
        const FwOpcodeType opCode
    )
  {

  }


  Fw::ParamValid FlatOutputAdapterTesterBase ::
    from_ParamGet_static(
        Fw::PassiveComponentBase* component,
        NATIVE_INT_TYPE portNum,
        FwPrmIdType id,
        Fw::ParamBuffer &val
    )
  {
    FlatOutputAdapterTesterBase* _testerBase =
      static_cast<FlatOutputAdapterTesterBase*>(component);

    Fw::SerializeStatus _status;
    Fw::ParamValid _ret = Fw::PARAM_VALID;
    val.resetSer();

    const U32 idBase = _testerBase->getIdBase();
    FW_ASSERT(id >= idBase, id, idBase);

    switch (id - idBase) {
      case 0:
      {
        _status = val.serialize(_testerBase->m_param_w_x);
        _ret = _testerBase->m_param_w_x_valid;
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );
      }
        break;
      case 1:
      {
        _status = val.serialize(_testerBase->m_param_w_y);
        _ret = _testerBase->m_param_w_y_valid;
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );
      }
        break;
      case 2:
      {
        _status = val.serialize(_testerBase->m_param_w_z);
        _ret = _testerBase->m_param_w_z_valid;
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );
      }
        break;
      default:
        FW_ASSERT(id);
        break;
    }

    return _ret;
  }

  void FlatOutputAdapterTesterBase ::
    from_ParamSet_static(
        Fw::PassiveComponentBase* component,
        NATIVE_INT_TYPE portNum,
        FwPrmIdType id,
        Fw::ParamBuffer &val
    )
  {
    FlatOutputAdapterTesterBase* _testerBase =
      static_cast<FlatOutputAdapterTesterBase*>(component);

    Fw::SerializeStatus _status;
    val.resetDeser();

    // This is exercised completely in autocode,
    // so just verify that it works. If it doesn't
    // it probably is an autocoder error.

    const U32 idBase = _testerBase->getIdBase();
    FW_ASSERT(id >= idBase, id, idBase);

    switch (id - idBase) {
      case 0:
      {
        F64 w_xVal;
        _status = val.deserialize(w_xVal);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );
        FW_ASSERT(
            w_xVal ==
            _testerBase->m_param_w_x
        );
        break;
      }

      case 1:
      {
        F64 w_yVal;
        _status = val.deserialize(w_yVal);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );
        FW_ASSERT(
            w_yVal ==
            _testerBase->m_param_w_y
        );
        break;
      }

      case 2:
      {
        F64 w_zVal;
        _status = val.deserialize(w_zVal);
        FW_ASSERT(
            _status == Fw::FW_SERIALIZE_OK,
            static_cast<AssertArg>(_status)
        );
        FW_ASSERT(
            w_zVal ==
            _testerBase->m_param_w_z
        );
        break;
      }

      default: {
        FW_ASSERT(id);
        break;
      }

    }
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_flatOutput->clear();
  }

  // ----------------------------------------------------------------------
  // From port: flatOutput
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    pushFromPortEntry_flatOutput(
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    FromPortEntry_flatOutput _e = {
      FlatOutput
    };
    this->fromPortHistory_flatOutput->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    from_flatOutput_handlerBase(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    FW_ASSERT(portNum < this->getNum_from_flatOutput(),static_cast<AssertArg>(portNum));
    this->from_flatOutput_handler(
        portNum,
        FlatOutput
    );
  }

  // ----------------------------------------------------------------------
  // Command response handling
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    cmdResponseIn(
        const FwOpcodeType opCode,
        const U32 seq,
        const Fw::CommandResponse response
    )
  {
    CmdResponse e = { opCode, seq, response };
    this->cmdResponseHistory->push_back(e);
  }


  void FlatOutputAdapterTesterBase ::
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

  void FlatOutputAdapterTesterBase ::
    clearHistory()
  {
    this->cmdResponseHistory->clear();
    this->clearFromPortHistory();
  }

  // ----------------------------------------------------------------------
  // Parameter w_x
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    paramSet_w_x(
        const F64& val,
        Fw::ParamValid valid
    )
  {
    this->m_param_w_x = val;
    this->m_param_w_x_valid = valid;
  }

  void FlatOutputAdapterTesterBase ::
    paramSend_w_x(
        NATIVE_INT_TYPE instance,
        U32 cmdSeq
    )
  {

    // Build command for parameter set

    Fw::CmdArgBuffer args;
    FW_ASSERT(
        args.serialize(
            this->m_param_w_x
        ) == Fw::FW_SERIALIZE_OK
    );
    const U32 idBase = this->getIdBase();
    FwOpcodeType _prmOpcode;
    _prmOpcode =  FlatOutputAdapterComponentBase::OPCODE_W_X_SET + idBase;
    if (not this->m_to_CmdDisp[0].isConnected()) {
      printf("Test Command Output port not connected!\n");
    }
    else {
      this->m_to_CmdDisp[0].invoke(
          _prmOpcode,
          cmdSeq,
          args
      );
    }

  }

  void FlatOutputAdapterTesterBase ::
    paramSave_w_x (
        NATIVE_INT_TYPE instance,
        U32 cmdSeq
    )

  {
    Fw::CmdArgBuffer args;
    FwOpcodeType _prmOpcode;
    const U32 idBase = this->getIdBase();
    _prmOpcode = FlatOutputAdapterComponentBase::OPCODE_W_X_SAVE + idBase;
    if (not this->m_to_CmdDisp[0].isConnected()) {
      printf("Test Command Output port not connected!\n");
    }
    else {
      this->m_to_CmdDisp[0].invoke(
          _prmOpcode,
          cmdSeq,
          args
      );
    }
  }

  // ----------------------------------------------------------------------
  // Parameter w_y
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    paramSet_w_y(
        const F64& val,
        Fw::ParamValid valid
    )
  {
    this->m_param_w_y = val;
    this->m_param_w_y_valid = valid;
  }

  void FlatOutputAdapterTesterBase ::
    paramSend_w_y(
        NATIVE_INT_TYPE instance,
        U32 cmdSeq
    )
  {

    // Build command for parameter set

    Fw::CmdArgBuffer args;
    FW_ASSERT(
        args.serialize(
            this->m_param_w_y
        ) == Fw::FW_SERIALIZE_OK
    );
    const U32 idBase = this->getIdBase();
    FwOpcodeType _prmOpcode;
    _prmOpcode =  FlatOutputAdapterComponentBase::OPCODE_W_Y_SET + idBase;
    if (not this->m_to_CmdDisp[0].isConnected()) {
      printf("Test Command Output port not connected!\n");
    }
    else {
      this->m_to_CmdDisp[0].invoke(
          _prmOpcode,
          cmdSeq,
          args
      );
    }

  }

  void FlatOutputAdapterTesterBase ::
    paramSave_w_y (
        NATIVE_INT_TYPE instance,
        U32 cmdSeq
    )

  {
    Fw::CmdArgBuffer args;
    FwOpcodeType _prmOpcode;
    const U32 idBase = this->getIdBase();
    _prmOpcode = FlatOutputAdapterComponentBase::OPCODE_W_Y_SAVE + idBase;
    if (not this->m_to_CmdDisp[0].isConnected()) {
      printf("Test Command Output port not connected!\n");
    }
    else {
      this->m_to_CmdDisp[0].invoke(
          _prmOpcode,
          cmdSeq,
          args
      );
    }
  }

  // ----------------------------------------------------------------------
  // Parameter w_z
  // ----------------------------------------------------------------------

  void FlatOutputAdapterTesterBase ::
    paramSet_w_z(
        const F64& val,
        Fw::ParamValid valid
    )
  {
    this->m_param_w_z = val;
    this->m_param_w_z_valid = valid;
  }

  void FlatOutputAdapterTesterBase ::
    paramSend_w_z(
        NATIVE_INT_TYPE instance,
        U32 cmdSeq
    )
  {

    // Build command for parameter set

    Fw::CmdArgBuffer args;
    FW_ASSERT(
        args.serialize(
            this->m_param_w_z
        ) == Fw::FW_SERIALIZE_OK
    );
    const U32 idBase = this->getIdBase();
    FwOpcodeType _prmOpcode;
    _prmOpcode =  FlatOutputAdapterComponentBase::OPCODE_W_Z_SET + idBase;
    if (not this->m_to_CmdDisp[0].isConnected()) {
      printf("Test Command Output port not connected!\n");
    }
    else {
      this->m_to_CmdDisp[0].invoke(
          _prmOpcode,
          cmdSeq,
          args
      );
    }

  }

  void FlatOutputAdapterTesterBase ::
    paramSave_w_z (
        NATIVE_INT_TYPE instance,
        U32 cmdSeq
    )

  {
    Fw::CmdArgBuffer args;
    FwOpcodeType _prmOpcode;
    const U32 idBase = this->getIdBase();
    _prmOpcode = FlatOutputAdapterComponentBase::OPCODE_W_Z_SAVE + idBase;
    if (not this->m_to_CmdDisp[0].isConnected()) {
      printf("Test Command Output port not connected!\n");
    }
    else {
      this->m_to_CmdDisp[0].invoke(
          _prmOpcode,
          cmdSeq,
          args
      );
    }
  }

} // end namespace Gnc
