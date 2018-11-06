// ======================================================================
// \title  R5SpiMasterDriver/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for R5SpiMasterDriver component test harness base class
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

  R5SpiMasterDriverTesterBase ::
    R5SpiMasterDriverTesterBase(
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

  R5SpiMasterDriverTesterBase ::
    ~R5SpiMasterDriverTesterBase(void) 
  {
  }

  void R5SpiMasterDriverTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Initialize output port spiSend

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_spiSend();
        ++_port
    ) {
      this->m_to_spiSend[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_spiSend[%d]",
          this->m_objName,
          _port
      );
      this->m_to_spiSend[_port].setObjName(_portName);
#endif

    }

    // Initialize output port spiRecv

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_spiRecv();
        ++_port
    ) {
      this->m_to_spiRecv[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_spiRecv[%d]",
          this->m_objName,
          _port
      );
      this->m_to_spiRecv[_port].setObjName(_portName);
#endif

    }

    // Initialize output port spiSendRecv

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_spiSendRecv();
        ++_port
    ) {
      this->m_to_spiSendRecv[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_spiSendRecv[%d]",
          this->m_objName,
          _port
      );
      this->m_to_spiSendRecv[_port].setObjName(_portName);
#endif

    }

    // Initialize output port spiConfig

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_spiConfig();
        ++_port
    ) {
      this->m_to_spiConfig[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_spiConfig[%d]",
          this->m_objName,
          _port
      );
      this->m_to_spiConfig[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE R5SpiMasterDriverTesterBase ::
    getNum_to_spiSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_spiSend);
  }

  NATIVE_INT_TYPE R5SpiMasterDriverTesterBase ::
    getNum_to_spiRecv(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_spiRecv);
  }

  NATIVE_INT_TYPE R5SpiMasterDriverTesterBase ::
    getNum_to_spiSendRecv(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_spiSendRecv);
  }

  NATIVE_INT_TYPE R5SpiMasterDriverTesterBase ::
    getNum_to_spiConfig(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_spiConfig);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void R5SpiMasterDriverTesterBase ::
    connect_to_spiSend(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSpiWritePort *const spiSend
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_spiSend(),static_cast<AssertArg>(portNum));
    this->m_to_spiSend[portNum].addCallPort(spiSend);
  }

  void R5SpiMasterDriverTesterBase ::
    connect_to_spiRecv(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSpiReadPort *const spiRecv
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_spiRecv(),static_cast<AssertArg>(portNum));
    this->m_to_spiRecv[portNum].addCallPort(spiRecv);
  }

  void R5SpiMasterDriverTesterBase ::
    connect_to_spiSendRecv(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSpiReadWritePort *const spiSendRecv
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_spiSendRecv(),static_cast<AssertArg>(portNum));
    this->m_to_spiSendRecv[portNum].addCallPort(spiSendRecv);
  }

  void R5SpiMasterDriverTesterBase ::
    connect_to_spiConfig(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSpiConfigPort *const spiConfig
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_spiConfig(),static_cast<AssertArg>(portNum));
    this->m_to_spiConfig[portNum].addCallPort(spiConfig);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void R5SpiMasterDriverTesterBase ::
    invoke_to_spiSend(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &buff
    )
  {
    FW_ASSERT(portNum < this->getNum_to_spiSend(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_spiSend(),static_cast<AssertArg>(portNum));
    this->m_to_spiSend[portNum].invoke(
        buff
    );
  }

  void R5SpiMasterDriverTesterBase ::
    invoke_to_spiRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &buff
    )
  {
    FW_ASSERT(portNum < this->getNum_to_spiRecv(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_spiRecv(),static_cast<AssertArg>(portNum));
    this->m_to_spiRecv[portNum].invoke(
        buff
    );
  }

  void R5SpiMasterDriverTesterBase ::
    invoke_to_spiSendRecv(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_spiSendRecv(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_spiSendRecv(),static_cast<AssertArg>(portNum));
    this->m_to_spiSendRecv[portNum].invoke(
        writeBuffer, readBuffer
    );
  }

  void R5SpiMasterDriverTesterBase ::
    invoke_to_spiConfig(
        const NATIVE_INT_TYPE portNum,
        U32 busSpeed
    )
  {
    FW_ASSERT(portNum < this->getNum_to_spiConfig(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_spiConfig(),static_cast<AssertArg>(portNum));
    this->m_to_spiConfig[portNum].invoke(
        busSpeed
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool R5SpiMasterDriverTesterBase ::
    isConnected_to_spiSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_spiSend(), static_cast<AssertArg>(portNum));
    return this->m_to_spiSend[portNum].isConnected();
  }

  bool R5SpiMasterDriverTesterBase ::
    isConnected_to_spiRecv(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_spiRecv(), static_cast<AssertArg>(portNum));
    return this->m_to_spiRecv[portNum].isConnected();
  }

  bool R5SpiMasterDriverTesterBase ::
    isConnected_to_spiSendRecv(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_spiSendRecv(), static_cast<AssertArg>(portNum));
    return this->m_to_spiSendRecv[portNum].isConnected();
  }

  bool R5SpiMasterDriverTesterBase ::
    isConnected_to_spiConfig(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_spiConfig(), static_cast<AssertArg>(portNum));
    return this->m_to_spiConfig[portNum].isConnected();
  }

} // end namespace R5
