// ======================================================================
// \title  GPSPosAdapter/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for GPSPosAdapter component test harness base class
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

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  GPSPosAdapterTesterBase ::
    GPSPosAdapterTesterBase(
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
    // Clear history
    this->clearHistory();
  }

  GPSPosAdapterTesterBase ::
    ~GPSPosAdapterTesterBase(void) 
  {
  }

  void GPSPosAdapterTesterBase ::
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

    // Initialize output port Guid

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_Guid();
        ++_port
    ) {
      this->m_to_Guid[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_Guid[%d]",
          this->m_objName,
          _port
      );
      this->m_to_Guid[_port].setObjName(_portName);
#endif

    }

    // Initialize output port Nav

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_Nav();
        ++_port
    ) {
      this->m_to_Nav[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_Nav[%d]",
          this->m_objName,
          _port
      );
      this->m_to_Nav[_port].setObjName(_portName);
#endif

    }

    // Initialize output port sched

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_sched();
        ++_port
    ) {
      this->m_to_sched[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_sched[%d]",
          this->m_objName,
          _port
      );
      this->m_to_sched[_port].setObjName(_portName);
#endif

    }

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

  NATIVE_INT_TYPE GPSPosAdapterTesterBase ::
    getNum_to_Guid(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Guid);
  }

  NATIVE_INT_TYPE GPSPosAdapterTesterBase ::
    getNum_to_Nav(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Nav);
  }

  NATIVE_INT_TYPE GPSPosAdapterTesterBase ::
    getNum_to_sched(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_sched);
  }

  NATIVE_INT_TYPE GPSPosAdapterTesterBase ::
    getNum_to_SerReadPort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SerReadPort);
  }

  NATIVE_INT_TYPE GPSPosAdapterTesterBase ::
    getNum_from_SerWritePort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SerWritePort);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void GPSPosAdapterTesterBase ::
    connect_to_Guid(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::InputFlatOutputPort *const Guid
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Guid(),static_cast<AssertArg>(portNum));
    this->m_to_Guid[portNum].addCallPort(Guid);
  }

  void GPSPosAdapterTesterBase ::
    connect_to_Nav(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::InputFlatOutputPort *const Nav
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Nav(),static_cast<AssertArg>(portNum));
    this->m_to_Nav[portNum].addCallPort(Nav);
  }

  void GPSPosAdapterTesterBase ::
    connect_to_sched(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const sched
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_sched(),static_cast<AssertArg>(portNum));
    this->m_to_sched[portNum].addCallPort(sched);
  }

  void GPSPosAdapterTesterBase ::
    connect_to_SerReadPort(
        const NATIVE_INT_TYPE portNum,
        Drv::InputSerialReadPort *const SerReadPort
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(),static_cast<AssertArg>(portNum));
    this->m_to_SerReadPort[portNum].addCallPort(SerReadPort);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void GPSPosAdapterTesterBase ::
    invoke_to_Guid(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    FW_ASSERT(portNum < this->getNum_to_Guid(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_Guid(),static_cast<AssertArg>(portNum));
    this->m_to_Guid[portNum].invoke(
        FlatOutput
    );
  }

  void GPSPosAdapterTesterBase ::
    invoke_to_Nav(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    FW_ASSERT(portNum < this->getNum_to_Nav(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_Nav(),static_cast<AssertArg>(portNum));
    this->m_to_Nav[portNum].invoke(
        FlatOutput
    );
  }

  void GPSPosAdapterTesterBase ::
    invoke_to_sched(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_sched(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_sched(),static_cast<AssertArg>(portNum));
    this->m_to_sched[portNum].invoke(
        context
    );
  }

  void GPSPosAdapterTesterBase ::
    invoke_to_SerReadPort(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        SerialReadStatus &status
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

  bool GPSPosAdapterTesterBase ::
    isConnected_to_Guid(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Guid(), static_cast<AssertArg>(portNum));
    return this->m_to_Guid[portNum].isConnected();
  }

  bool GPSPosAdapterTesterBase ::
    isConnected_to_Nav(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Nav(), static_cast<AssertArg>(portNum));
    return this->m_to_Nav[portNum].isConnected();
  }

  bool GPSPosAdapterTesterBase ::
    isConnected_to_sched(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_sched(), static_cast<AssertArg>(portNum));
    return this->m_to_sched[portNum].isConnected();
  }

  bool GPSPosAdapterTesterBase ::
    isConnected_to_SerReadPort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SerReadPort(), static_cast<AssertArg>(portNum));
    return this->m_to_SerReadPort[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Drv::InputSerialWritePort *GPSPosAdapterTesterBase ::
    get_from_SerWritePort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SerWritePort(),static_cast<AssertArg>(portNum));
    return &this->m_from_SerWritePort[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void GPSPosAdapterTesterBase ::
    from_SerWritePort_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    FW_ASSERT(callComp);
    GPSPosAdapterTesterBase* _testerBase = 
      static_cast<GPSPosAdapterTesterBase*>(callComp);
    _testerBase->from_SerWritePort_handlerBase(
        portNum,
        serBuffer
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void GPSPosAdapterTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_SerWritePort->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: SerWritePort
  // ---------------------------------------------------------------------- 

  void GPSPosAdapterTesterBase ::
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
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void GPSPosAdapterTesterBase ::
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

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void GPSPosAdapterTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace Drv
