// ======================================================================
// \title  Tee/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for Tee component test harness base class
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

  TeeTesterBase ::
    TeeTesterBase(
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
    // Clear history
    this->clearHistory();
  }

  TeeTesterBase ::
    ~TeeTesterBase(void) 
  {
  }

  void TeeTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port DataOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_DataOut();
        ++_port
    ) {

      this->m_from_DataOut[_port].init();
      this->m_from_DataOut[_port].addCallComp(
          this,
          from_DataOut_static
      );
      this->m_from_DataOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_DataOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_DataOut[_port].setObjName(_portName);
#endif

    }

    // Initialize output port DataIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_DataIn();
        ++_port
    ) {
      this->m_to_DataIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_DataIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_DataIn[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE TeeTesterBase ::
    getNum_to_DataIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_DataIn);
  }

  NATIVE_INT_TYPE TeeTesterBase ::
    getNum_from_DataOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_DataOut);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void TeeTesterBase ::
    connect_to_DataIn(
        const NATIVE_INT_TYPE portNum,
        Fw::InputSerializePort *const DataIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_DataIn(),static_cast<AssertArg>(portNum));
    this->m_to_DataIn[portNum].registerSerialPort(DataIn);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void TeeTesterBase ::
    invoke_to_DataIn(
      NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer
    )
  {
    FW_ASSERT(portNum < this->getNum_to_DataIn(),static_cast<AssertArg>(portNum));
    this->m_to_DataIn[portNum].invokeSerial(Buffer);
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool TeeTesterBase ::
    isConnected_to_DataIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_DataIn(), static_cast<AssertArg>(portNum));
    return this->m_to_DataIn[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Fw::InputSerializePort *TeeTesterBase ::
    get_from_DataOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_DataOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_DataOut[portNum];
  }

  // Added manually
  void TeeTesterBase ::
    from_DataOut_static(
      Fw::PassiveComponentBase *const callComp, //!< The component instance
      const NATIVE_INT_TYPE portNum, //!< The port number
      Fw::SerializeBufferBase& Buffer //!< serialized data buffer
    )
  {
    FW_ASSERT(callComp);
    TeeTesterBase* _testerBase =
      static_cast<TeeTesterBase*>(callComp);

    _testerBase->from_DataOut_handlerBase(
        portNum,
        Buffer
    );
  }

  void TeeTesterBase ::
    from_DataOut_handlerBase (
      NATIVE_INT_TYPE portNum, /*!< The port number*/
      Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    FW_ASSERT(portNum < this->getNum_from_DataOut(),static_cast<AssertArg>(portNum));
    this->from_DataOut_handler(
        portNum,
        Buffer
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void TeeTesterBase ::
    clearHistory()
  {
  }

} // end namespace Svc
