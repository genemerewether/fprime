// ======================================================================
// \title  R5Prm/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for R5Prm component test harness base class
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

  R5PrmTesterBase ::
    R5PrmTesterBase(
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

  R5PrmTesterBase ::
    ~R5PrmTesterBase(void) 
  {
  }

  void R5PrmTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Initialize output port set

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_set();
        ++_port
    ) {
      this->m_to_set[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_set[%d]",
          this->m_objName,
          _port
      );
      this->m_to_set[_port].setObjName(_portName);
#endif

    }

    // Initialize output port get

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_get();
        ++_port
    ) {
      this->m_to_get[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_get[%d]",
          this->m_objName,
          _port
      );
      this->m_to_get[_port].setObjName(_portName);
#endif

    }

    // Initialize output port write

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_write();
        ++_port
    ) {
      this->m_to_write[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_write[%d]",
          this->m_objName,
          _port
      );
      this->m_to_write[_port].setObjName(_portName);
#endif

    }

    // Initialize output port check

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_check();
        ++_port
    ) {
      this->m_to_check[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_check[%d]",
          this->m_objName,
          _port
      );
      this->m_to_check[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE R5PrmTesterBase ::
    getNum_to_set(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_set);
  }

  NATIVE_INT_TYPE R5PrmTesterBase ::
    getNum_to_get(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_get);
  }

  NATIVE_INT_TYPE R5PrmTesterBase ::
    getNum_to_write(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_write);
  }

  NATIVE_INT_TYPE R5PrmTesterBase ::
    getNum_to_check(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_check);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void R5PrmTesterBase ::
    connect_to_set(
        const NATIVE_INT_TYPE portNum,
        R5::InputPrmSetPort *const set
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_set(),static_cast<AssertArg>(portNum));
    this->m_to_set[portNum].addCallPort(set);
  }

  void R5PrmTesterBase ::
    connect_to_get(
        const NATIVE_INT_TYPE portNum,
        R5::InputPrmGetPort *const get
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    this->m_to_get[portNum].addCallPort(get);
  }

  void R5PrmTesterBase ::
    connect_to_write(
        const NATIVE_INT_TYPE portNum,
        R5::InputPrmWritePort *const write
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_write(),static_cast<AssertArg>(portNum));
    this->m_to_write[portNum].addCallPort(write);
  }

  void R5PrmTesterBase ::
    connect_to_check(
        const NATIVE_INT_TYPE portNum,
        R5::InputPrmCheckPort *const check
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_check(),static_cast<AssertArg>(portNum));
    this->m_to_check[portNum].addCallPort(check);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void R5PrmTesterBase ::
    invoke_to_set(
        const NATIVE_INT_TYPE portNum,
        R5::PrmEntry &prm,
        bool &valid
    )
  {
    FW_ASSERT(portNum < this->getNum_to_set(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_set(),static_cast<AssertArg>(portNum));
    this->m_to_set[portNum].invoke(
        prm, valid
    );
  }

  void R5PrmTesterBase ::
    invoke_to_get(
        const NATIVE_INT_TYPE portNum,
        R5::PrmEntry &prm,
        bool &valid
    )
  {
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    this->m_to_get[portNum].invoke(
        prm, valid
    );
  }

  void R5PrmTesterBase ::
    invoke_to_write(
        const NATIVE_INT_TYPE portNum,
        bool force
    )
  {
    FW_ASSERT(portNum < this->getNum_to_write(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_write(),static_cast<AssertArg>(portNum));
    this->m_to_write[portNum].invoke(
        force
    );
  }

  void R5PrmTesterBase ::
    invoke_to_check(
        const NATIVE_INT_TYPE portNum,
        PrmWriteStatus &stat
    )
  {
    FW_ASSERT(portNum < this->getNum_to_check(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_check(),static_cast<AssertArg>(portNum));
    this->m_to_check[portNum].invoke(
        stat
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool R5PrmTesterBase ::
    isConnected_to_set(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_set(), static_cast<AssertArg>(portNum));
    return this->m_to_set[portNum].isConnected();
  }

  bool R5PrmTesterBase ::
    isConnected_to_get(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_get(), static_cast<AssertArg>(portNum));
    return this->m_to_get[portNum].isConnected();
  }

  bool R5PrmTesterBase ::
    isConnected_to_write(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_write(), static_cast<AssertArg>(portNum));
    return this->m_to_write[portNum].isConnected();
  }

  bool R5PrmTesterBase ::
    isConnected_to_check(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_check(), static_cast<AssertArg>(portNum));
    return this->m_to_check[portNum].isConnected();
  }

} // end namespace R5
