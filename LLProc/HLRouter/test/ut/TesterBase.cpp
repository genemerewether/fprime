// ======================================================================
// \title  HLRouter/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for HLRouter component test harness base class
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

  HLRouterTesterBase ::
    HLRouterTesterBase(
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
    this->fromPortHistory_SerReadPort =
      new History<FromPortEntry_SerReadPort>(maxHistorySize);
    this->fromPortHistory_SerWritePort =
      new History<FromPortEntry_SerWritePort>(maxHistorySize);
    this->fromPortHistory_DebugOut =
      new History<FromPortEntry_DebugOut>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  HLRouterTesterBase ::
    ~HLRouterTesterBase(void) 
  {
  }

  void HLRouterTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port HLPortsOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_HLPortsOut();
        ++_port
    ) {

      this->m_from_HLPortsOut[_port].init();
      this->m_from_HLPortsOut[_port].addCallComp(
          this,
          from_HLPortsOut_static
      );
      this->m_from_HLPortsOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_HLPortsOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_HLPortsOut[_port].setObjName(_portName);
#endif

    }

    // Attach input port SerReadPort

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_SerReadPort();
        ++_port
    ) {

      this->m_from_SerReadPort[_port].init();
      this->m_from_SerReadPort[_port].addCallComp(
          this,
          from_SerReadPort_static
      );
      this->m_from_SerReadPort[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_SerReadPort[%d]",
          this->m_objName,
          _port
      );
      this->m_from_SerReadPort[_port].setObjName(_portName);
#endif

    }

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

    // Attach input port DebugOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_DebugOut();
        ++_port
    ) {

      this->m_from_DebugOut[_port].init();
      this->m_from_DebugOut[_port].addCallComp(
          this,
          from_DebugOut_static
      );
      this->m_from_DebugOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_DebugOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_DebugOut[_port].setObjName(_portName);
#endif

    }

    // Initialize output port LLPortsIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_LLPortsIn();
        ++_port
    ) {
      this->m_to_LLPortsIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_LLPortsIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_LLPortsIn[_port].setObjName(_portName);
#endif

    }

    // Initialize output port Sched

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_Sched();
        ++_port
    ) {
      this->m_to_Sched[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_Sched[%d]",
          this->m_objName,
          _port
      );
      this->m_to_Sched[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE HLRouterTesterBase ::
    getNum_to_LLPortsIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_LLPortsIn);
  }

  NATIVE_INT_TYPE HLRouterTesterBase ::
    getNum_from_HLPortsOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_HLPortsOut);
  }

  NATIVE_INT_TYPE HLRouterTesterBase ::
    getNum_from_SerReadPort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SerReadPort);
  }

  NATIVE_INT_TYPE HLRouterTesterBase ::
    getNum_from_SerWritePort(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_SerWritePort);
  }

  NATIVE_INT_TYPE HLRouterTesterBase ::
    getNum_to_Sched(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_Sched);
  }

  NATIVE_INT_TYPE HLRouterTesterBase ::
    getNum_from_DebugOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_DebugOut);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void HLRouterTesterBase ::
    connect_to_LLPortsIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputSerializePort *const LLPortsIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_LLPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_LLPortsIn[portNum].registerSerialPort(LLPortsIn);
  }

  void HLRouterTesterBase ::
    connect_to_Sched(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const Sched
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(),static_cast<AssertArg>(portNum));
    this->m_to_Sched[portNum].addCallPort(Sched);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void HLRouterTesterBase ::
    invoke_to_LLPortsIn(
      NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_LLPortsIn(),static_cast<AssertArg>(portNum));
    this->m_to_LLPortsIn[portNum].invokeSerial(Buffer);
  }

  void HLRouterTesterBase ::
    invoke_to_Sched(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_Sched(),static_cast<AssertArg>(portNum));
    this->m_to_Sched[portNum].invoke(
        context
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool HLRouterTesterBase ::
    isConnected_to_LLPortsIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_LLPortsIn(), static_cast<AssertArg>(portNum));
    return this->m_to_LLPortsIn[portNum].isConnected();
  }

  bool HLRouterTesterBase ::
    isConnected_to_Sched(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_Sched(), static_cast<AssertArg>(portNum));
    return this->m_to_Sched[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputSerializePort *HLRouterTesterBase ::
    get_from_HLPortsOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_HLPortsOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_HLPortsOut[portNum];
  }

  Drv::InputSerialReadPort *HLRouterTesterBase ::
    get_from_SerReadPort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SerReadPort(),static_cast<AssertArg>(portNum));
    return &this->m_from_SerReadPort[portNum];
  }

  Drv::InputSerialWritePort *HLRouterTesterBase ::
    get_from_SerWritePort(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_SerWritePort(),static_cast<AssertArg>(portNum));
    return &this->m_from_SerWritePort[portNum];
  }

  LLProc::InputDebugStringPort *HLRouterTesterBase ::
    get_from_DebugOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_DebugOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_DebugOut[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void HLRouterTesterBase ::
    from_SerReadPort_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        Drv::SerialReadStatus &status
    )
  {
    FW_ASSERT(callComp);
    HLRouterTesterBase* _testerBase = 
      static_cast<HLRouterTesterBase*>(callComp);
    _testerBase->from_SerReadPort_handlerBase(
        portNum,
        serBuffer, status
    );
  }

  void HLRouterTesterBase ::
    from_SerWritePort_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    FW_ASSERT(callComp);
    HLRouterTesterBase* _testerBase = 
      static_cast<HLRouterTesterBase*>(callComp);
    _testerBase->from_SerWritePort_handlerBase(
        portNum,
        serBuffer
    );
  }

  void HLRouterTesterBase ::
    from_DebugOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U8 *string,
        U32 size
    )
  {
    FW_ASSERT(callComp);
    HLRouterTesterBase* _testerBase = 
      static_cast<HLRouterTesterBase*>(callComp);
    _testerBase->from_DebugOut_handlerBase(
        portNum,
        string, size
    );
  }

  void HLRouterTesterBase ::
    from_HLPortsOut_static(
      Fw::PassiveComponentBase *const callComp, //!< The component instance
      const NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer //!< serialized data buffer
    )
  {
    FW_ASSERT(callComp);
    HLRouterTesterBase* _testerBase = 
      static_cast<HLRouterTesterBase*>(callComp);

    _testerBase->from_HLPortsOut_handlerBase(
        portNum,
        Buffer
    );
  }  

  void HLRouterTesterBase ::
    from_HLPortsOut_handlerBase(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    FW_ASSERT(portNum < this->getNum_from_HLPortsOut(),static_cast<AssertArg>(portNum));
    this->from_HLPortsOut_handler(
        portNum,
        Buffer
    );
  } 
   
  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void HLRouterTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_SerReadPort->clear();
    this->fromPortHistory_SerWritePort->clear();
    this->fromPortHistory_DebugOut->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: SerReadPort
  // ---------------------------------------------------------------------- 

  void HLRouterTesterBase ::
    pushFromPortEntry_SerReadPort(
        Fw::Buffer &serBuffer,
        Drv::SerialReadStatus &status
    )
  {
    FromPortEntry_SerReadPort _e = {
      serBuffer, status
    };
    this->fromPortHistory_SerReadPort->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: SerWritePort
  // ---------------------------------------------------------------------- 

  void HLRouterTesterBase ::
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
  // From port: DebugOut
  // ---------------------------------------------------------------------- 

  void HLRouterTesterBase ::
    pushFromPortEntry_DebugOut(
        U8 *string,
        U32 size
    )
  {
    FromPortEntry_DebugOut _e = {
      string, size
    };
    this->fromPortHistory_DebugOut->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void HLRouterTesterBase ::
    from_SerReadPort_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        Drv::SerialReadStatus &status
    )
  {
    FW_ASSERT(portNum < this->getNum_from_SerReadPort(),static_cast<AssertArg>(portNum));
    this->from_SerReadPort_handler(
        portNum,
        serBuffer, status
    );
  }

  void HLRouterTesterBase ::
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

  void HLRouterTesterBase ::
    from_DebugOut_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U8 *string,
        U32 size
    )
  {
    FW_ASSERT(portNum < this->getNum_from_DebugOut(),static_cast<AssertArg>(portNum));
    this->from_DebugOut_handler(
        portNum,
        string, size
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void HLRouterTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace LLProc
