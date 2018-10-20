// ======================================================================
// \title  R5UartDriver/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for R5UartDriver component test harness base class
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

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  R5UartDriverTesterBase ::
    R5UartDriverTesterBase(
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
  }

  R5UartDriverTesterBase ::
    ~R5UartDriverTesterBase(void) 
  {
  }

  void R5UartDriverTesterBase ::
    init(NATIVE_INT_TYPE instance)
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Initialize output port serialRecv

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_serialRecv();
        ++_port
    ) {
      this->m_to_serialRecv[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_serialRecv[%d]",
          this->m_objName,
          _port
      );
      this->m_to_serialRecv[_port].setObjName(_portName);
#endif

    }

    // Initialize output port serialSend

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_serialSend();
        ++_port
    ) {
      this->m_to_serialSend[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_serialSend[%d]",
          this->m_objName,
          _port
      );
      this->m_to_serialSend[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE R5UartDriverTesterBase ::
    getNum_to_serialRecv(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_serialRecv);
  }

  NATIVE_INT_TYPE R5UartDriverTesterBase ::
    getNum_to_serialSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_serialSend);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void R5UartDriverTesterBase ::
    connect_to_serialRecv(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSerialReadPort *const serialRecv
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_serialRecv(),static_cast<AssertArg>(portNum));
    this->m_to_serialRecv[portNum].addCallPort(serialRecv);
  }

  void R5UartDriverTesterBase ::
    connect_to_serialSend(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSerialWritePort *const serialSend
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_serialSend(),static_cast<AssertArg>(portNum));
    this->m_to_serialSend[portNum].addCallPort(serialSend);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void R5UartDriverTesterBase ::
    invoke_to_serialRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        Drv::SerialReadStatus &status
    )
  {
    FW_ASSERT(portNum < this->getNum_to_serialRecv(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_serialRecv(),static_cast<AssertArg>(portNum));
    this->m_to_serialRecv[portNum].invoke(
        serBuffer, status
    );
  }

  void R5UartDriverTesterBase ::
    invoke_to_serialSend(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_serialSend(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_serialSend(),static_cast<AssertArg>(portNum));
    this->m_to_serialSend[portNum].invoke(
        serBuffer
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool R5UartDriverTesterBase ::
    isConnected_to_serialRecv(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_serialRecv(), static_cast<AssertArg>(portNum));
    return this->m_to_serialRecv[portNum].isConnected();
  }

  bool R5UartDriverTesterBase ::
    isConnected_to_serialSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_serialSend(), static_cast<AssertArg>(portNum));
    return this->m_to_serialSend[portNum].isConnected();
  }

} // end namespace R5
