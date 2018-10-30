// ======================================================================
// \title  LLDebug/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for LLDebug component test harness base class
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

namespace LLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  LLDebugTesterBase ::
    LLDebugTesterBase(
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
    this->fromPortHistory_SerWritePort =
      new History<FromPortEntry_SerWritePort>(maxHistorySize);
    this->fromPortHistory_SwFault =
      new History<FromPortEntry_SwFault>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  LLDebugTesterBase ::
    ~LLDebugTesterBase(void) 
  {
  }

  void LLDebugTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port SerWritePort

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_SerWritePort();
        ++_port
    ) {

      this->m_from_SerWritePort[_port].init();
      this->m_from_SerWritePort[_port].addCallComp(
          this,
          from_SerWritePort_static
      );
      this->m_from_SerWritePort[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_SerWritePort[%d]",
          this->m_objName,
          _port
      );
      this->m_from_SerWritePort[_port].setObjName(_portName);
#endif

    }

    // Attach input port SwFault

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_SwFault();
        ++_port
    ) {

      this->m_from_SwFault[_port].init();
      this->m_from_SwFault[_port].addCallComp(
          this,
          from_SwFault_static
      );
      this->m_from_SwFault[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_SwFault[%d]",
          this->m_objName,
          _port
      );
      this->m_from_SwFault[_port].setObjName(_portName);
#endif

    }

    // Initialize output port DebugStringIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_DebugStringIn();
        ++_port
    ) {
      this->m_to_DebugStringIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_DebugStringIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_DebugStringIn[_port].setObjName(_portName);
#endif

    }

    // Initialize output port AssertEnable

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_AssertEnable();
        ++_port
    ) {
      this->m_to_AssertEnable[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_AssertEnable[%d]",
          this->m_objName,
          _port
      );
      this->m_to_AssertEnable[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE LLDebugTesterBase ::
    getNum_from_SerWritePort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SerWritePort);
  }

  NATIVE_INT_TYPE LLDebugTesterBase ::
    getNum_from_SwFault(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SwFault);
  }

  NATIVE_INT_TYPE LLDebugTesterBase ::
    getNum_to_DebugStringIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_DebugStringIn);
  }

  NATIVE_INT_TYPE LLDebugTesterBase ::
    getNum_to_AssertEnable(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_AssertEnable);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void LLDebugTesterBase ::
    connect_to_DebugStringIn(
        const NATIVE_INT_TYPE portNum,
        LLProc::InputDebugStringPort *const DebugStringIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_DebugStringIn(),static_cast<AssertArg>(portNum));
    this->m_to_DebugStringIn[portNum].addCallPort(DebugStringIn);
  }

  void LLDebugTesterBase ::
    connect_to_AssertEnable(
        const NATIVE_INT_TYPE portNum,
        Common::InputEnablePort *const AssertEnable
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_AssertEnable(),static_cast<AssertArg>(portNum));
    this->m_to_AssertEnable[portNum].addCallPort(AssertEnable);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void LLDebugTesterBase ::
    invoke_to_DebugStringIn(
        const NATIVE_INT_TYPE portNum,
        U8 *string,
        U32 size
    )
  {
    FW_ASSERT(portNum < this->getNum_to_DebugStringIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_DebugStringIn(),static_cast<AssertArg>(portNum));
    this->m_to_DebugStringIn[portNum].invoke(
        string, size
    );
  }

  void LLDebugTesterBase ::
    invoke_to_AssertEnable(
        const NATIVE_INT_TYPE portNum,
        Common::EnableState enabled
    )
  {
    FW_ASSERT(portNum < this->getNum_to_AssertEnable(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_AssertEnable(),static_cast<AssertArg>(portNum));
    this->m_to_AssertEnable[portNum].invoke(
        enabled
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool LLDebugTesterBase ::
    isConnected_to_DebugStringIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_DebugStringIn(), static_cast<AssertArg>(portNum));
    return this->m_to_DebugStringIn[portNum].isConnected();
  }

  bool LLDebugTesterBase ::
    isConnected_to_AssertEnable(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_AssertEnable(), static_cast<AssertArg>(portNum));
    return this->m_to_AssertEnable[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Drv::InputSerialWritePort *LLDebugTesterBase ::
    get_from_SerWritePort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SerWritePort(),static_cast<AssertArg>(portNum));
    return &this->m_from_SerWritePort[portNum];
  }

  R5::InputGpioSetPort *LLDebugTesterBase ::
    get_from_SwFault(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SwFault(),static_cast<AssertArg>(portNum));
    return &this->m_from_SwFault[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void LLDebugTesterBase ::
    from_SerWritePort_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    FW_ASSERT(callComp);
    LLDebugTesterBase* _testerBase = 
      static_cast<LLDebugTesterBase*>(callComp);
    _testerBase->from_SerWritePort_handlerBase(
        portNum,
        serBuffer
    );
  }

  void LLDebugTesterBase ::
    from_SwFault_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        R5::GpioSetBank bank,
        U32 bit,
        R5::GpioSetVal val
    )
  {
    FW_ASSERT(callComp);
    LLDebugTesterBase* _testerBase = 
      static_cast<LLDebugTesterBase*>(callComp);
    _testerBase->from_SwFault_handlerBase(
        portNum,
        bank, bit, val
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void LLDebugTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_SerWritePort->clear();
    this->fromPortHistory_SwFault->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: SerWritePort
  // ---------------------------------------------------------------------- 

  void LLDebugTesterBase ::
    pushFromPortEntry_SerWritePort(
        Fw::Buffer &serBuffer
    )
  {
    FromPortEntry_SerWritePort _e = {
      serBuffer
    };
    this->fromPortHistory_SerWritePort->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: SwFault
  // ---------------------------------------------------------------------- 

  void LLDebugTesterBase ::
    pushFromPortEntry_SwFault(
        R5::GpioSetBank bank,
        U32 bit,
        R5::GpioSetVal val
    )
  {
    FromPortEntry_SwFault _e = {
      bank, bit, val
    };
    this->fromPortHistory_SwFault->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void LLDebugTesterBase ::
    from_SerWritePort_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_SerWritePort(),static_cast<AssertArg>(portNum));
    this->from_SerWritePort_handler(
        portNum,
        serBuffer
    );
  }

  void LLDebugTesterBase ::
    from_SwFault_handlerBase(
        const NATIVE_INT_TYPE portNum,
        R5::GpioSetBank bank,
        U32 bit,
        R5::GpioSetVal val
    )
  {
    FW_ASSERT(portNum < this->getNum_from_SwFault(),static_cast<AssertArg>(portNum));
    this->from_SwFault_handler(
        portNum,
        bank, bit, val
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void LLDebugTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace LLProc
