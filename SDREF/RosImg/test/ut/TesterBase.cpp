// ======================================================================
// \title  RosImg/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for RosImg component test harness base class
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

namespace Navoutdoor {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  RosImgTesterBase ::
    RosImgTesterBase(
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

  RosImgTesterBase ::
    ~RosImgTesterBase(void) 
  {
  }

  void RosImgTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Initialize output port ImageRecv

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_ImageRecv();
        ++_port
    ) {
      this->m_to_ImageRecv[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_ImageRecv[%d]",
          this->m_objName,
          _port
      );
      this->m_to_ImageRecv[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE RosImgTesterBase ::
    getNum_to_ImageRecv(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_ImageRecv);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void RosImgTesterBase ::
    connect_to_ImageRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::InputBufferSendPort *const ImageRecv
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_ImageRecv(),static_cast<AssertArg>(portNum));
    this->m_to_ImageRecv[portNum].addCallPort(ImageRecv);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void RosImgTesterBase ::
    invoke_to_ImageRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_ImageRecv(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_ImageRecv(),static_cast<AssertArg>(portNum));
    this->m_to_ImageRecv[portNum].invoke(
        fwBuffer
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool RosImgTesterBase ::
    isConnected_to_ImageRecv(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_ImageRecv(), static_cast<AssertArg>(portNum));
    return this->m_to_ImageRecv[portNum].isConnected();
  }

} // end namespace Navoutdoor
