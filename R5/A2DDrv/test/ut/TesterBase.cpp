// ======================================================================
// \title  R5A2DDriver/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for R5A2DDriver component test harness base class
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

  R5A2DDriverTesterBase ::
    R5A2DDriverTesterBase(
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

  R5A2DDriverTesterBase ::
    ~R5A2DDriverTesterBase(void) 
  {
  }

  void R5A2DDriverTesterBase ::
    init(NATIVE_INT_TYPE instance)
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

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

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE R5A2DDriverTesterBase ::
    getNum_to_get(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_get);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void R5A2DDriverTesterBase ::
    connect_to_get(
        const NATIVE_INT_TYPE portNum,
        R5::InputA2DGetPort *const get
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    this->m_to_get[portNum].addCallPort(get);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void R5A2DDriverTesterBase ::
    invoke_to_get(
        const NATIVE_INT_TYPE portNum,
        D2AGetBank bank,
        U32 channel,
        F32 &val
    )
  {
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_get(),static_cast<AssertArg>(portNum));
    this->m_to_get[portNum].invoke(
        bank, channel, val
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool R5A2DDriverTesterBase ::
    isConnected_to_get(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_get(), static_cast<AssertArg>(portNum));
    return this->m_to_get[portNum].isConnected();
  }

} // end namespace R5
