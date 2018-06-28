// ======================================================================
// \title  SerLogger/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for SerLogger component test harness base class
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

  SerLoggerTesterBase ::
    SerLoggerTesterBase(
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
    this->fromPortHistory_LogOut =
      new History<FromPortEntry_LogOut>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  SerLoggerTesterBase ::
    ~SerLoggerTesterBase(void) 
  {
  }

  void SerLoggerTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port LogOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_LogOut();
        ++_port
    ) {

      this->m_from_LogOut[_port].init();
      this->m_from_LogOut[_port].addCallComp(
          this,
          from_LogOut_static
      );
      this->m_from_LogOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_LogOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_LogOut[_port].setObjName(_portName);
#endif

    }

    // Initialize output port SerPortIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_SerPortIn();
        ++_port
    ) {
      this->m_to_SerPortIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_SerPortIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_SerPortIn[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE SerLoggerTesterBase ::
    getNum_to_SerPortIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SerPortIn);
  }

  NATIVE_INT_TYPE SerLoggerTesterBase ::
    getNum_from_LogOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_LogOut);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void SerLoggerTesterBase ::
    connect_to_SerPortIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputSerializePort *const SerPortIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_SerPortIn(),static_cast<AssertArg>(portNum));
    this->m_to_SerPortIn[portNum].registerSerialPort(SerPortIn);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void SerLoggerTesterBase ::
    invoke_to_SerPortIn(
      NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_SerPortIn(),static_cast<AssertArg>(portNum));
    this->m_to_SerPortIn[portNum].invokeSerial(Buffer);
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool SerLoggerTesterBase ::
    isConnected_to_SerPortIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SerPortIn(), static_cast<AssertArg>(portNum));
    return this->m_to_SerPortIn[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Svc::InputActiveFileLogPortPort *SerLoggerTesterBase ::
    get_from_LogOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_LogOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_LogOut[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void SerLoggerTesterBase ::
    from_LogOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Svc::ActiveFileLoggerPacket &data
    )
  {
    FW_ASSERT(callComp);
    SerLoggerTesterBase* _testerBase = 
      static_cast<SerLoggerTesterBase*>(callComp);
    _testerBase->from_LogOut_handlerBase(
        portNum,
        data
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void SerLoggerTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_LogOut->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: LogOut
  // ---------------------------------------------------------------------- 

  void SerLoggerTesterBase ::
    pushFromPortEntry_LogOut(
        Svc::ActiveFileLoggerPacket &data
    )
  {
    FromPortEntry_LogOut _e = {
      data
    };
    this->fromPortHistory_LogOut->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void SerLoggerTesterBase ::
    from_LogOut_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Svc::ActiveFileLoggerPacket &data
    )
  {
    FW_ASSERT(portNum < this->getNum_from_LogOut(),static_cast<AssertArg>(portNum));
    this->from_LogOut_handler(
        portNum,
        data
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void SerLoggerTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace Svc
