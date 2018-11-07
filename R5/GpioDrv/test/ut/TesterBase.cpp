// ======================================================================
// \title  R5GpioDriver/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for R5GpioDriver component test harness base class
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

  R5GpioDriverTesterBase ::
    R5GpioDriverTesterBase(
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

  R5GpioDriverTesterBase ::
    ~R5GpioDriverTesterBase(void)
  {
  }

  void R5GpioDriverTesterBase ::
    init(NATIVE_INT_TYPE instance)
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

    // Initialize output port wait

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_wait();
        ++_port
    ) {
      this->m_to_wait[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_wait[%d]",
          this->m_objName,
          _port
      );
      this->m_to_wait[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE R5GpioDriverTesterBase ::
    getNum_to_set(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_set);
  }

  NATIVE_INT_TYPE R5GpioDriverTesterBase ::
    getNum_to_get(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_get);
  }

  NATIVE_INT_TYPE R5GpioDriverTesterBase ::
    getNum_to_wait(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_wait);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports
  // ----------------------------------------------------------------------

  void R5GpioDriverTesterBase ::
    connect_to_set(
        const NATIVE_INT_TYPE portNum,
        R5::InputGpioSetPort *const set
    )
  {
    FW_ASSERT(portNum < this->getNum_to_set(),static_cast<AssertArg>(portNum));
    this->m_to_set[portNum].addCallPort(set);
  }

  void R5GpioDriverTesterBase ::
    connect_to_get(
        const NATIVE_INT_TYPE portNum,
        R5::InputGpioGetPort *const get
    )
  {
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    this->m_to_get[portNum].addCallPort(get);
  }

  void R5GpioDriverTesterBase ::
    connect_to_wait(
        const NATIVE_INT_TYPE portNum,
        R5::InputGpioWaitPort *const wait
    )
  {
    FW_ASSERT(portNum < this->getNum_to_wait(),static_cast<AssertArg>(portNum));
    this->m_to_wait[portNum].addCallPort(wait);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void R5GpioDriverTesterBase ::
    invoke_to_set(
        const NATIVE_INT_TYPE portNum,
        GpioSetBank bank,
        U32 bit,
        GpioSetVal val
    )
  {
    FW_ASSERT(portNum < this->getNum_to_set(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_set(),static_cast<AssertArg>(portNum));
    this->m_to_set[portNum].invoke(
        bank, bit, val
    );
  }

  void R5GpioDriverTesterBase ::
    invoke_to_get(
        const NATIVE_INT_TYPE portNum,
        GpioGetBank bank,
        U32 bit,
        GpioGetVal &val
    )
  {
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    this->m_to_get[portNum].invoke(
        bank, bit, val
    );
  }

  void R5GpioDriverTesterBase ::
    invoke_to_wait(
        const NATIVE_INT_TYPE portNum,
        GpioWaitBank bank,
        U32 bit,
        bool &alreadySet
    )
  {
    FW_ASSERT(portNum < this->getNum_to_wait(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_wait(),static_cast<AssertArg>(portNum));
    this->m_to_wait[portNum].invoke(
        bank, bit, alreadySet
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool R5GpioDriverTesterBase ::
    isConnected_to_set(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_set(), static_cast<AssertArg>(portNum));
    return this->m_to_set[portNum].isConnected();
  }

  bool R5GpioDriverTesterBase ::
    isConnected_to_get(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_get(), static_cast<AssertArg>(portNum));
    return this->m_to_get[portNum].isConnected();
  }

  bool R5GpioDriverTesterBase ::
    isConnected_to_wait(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_wait(), static_cast<AssertArg>(portNum));
    return this->m_to_wait[portNum].isConnected();
  }

} // end namespace R5
