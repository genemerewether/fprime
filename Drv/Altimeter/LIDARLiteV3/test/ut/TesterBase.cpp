// ======================================================================
// \title  LIDARLiteV3/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for LIDARLiteV3 component test harness base class
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

  LIDARLiteV3TesterBase ::
    LIDARLiteV3TesterBase(
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
    this->fromPortHistory_AltimeterSend =
      new History<FromPortEntry_AltimeterSend>(maxHistorySize);
    this->fromPortHistory_I2CConfig =
      new History<FromPortEntry_I2CConfig>(maxHistorySize);
    this->fromPortHistory_I2CWriteRead =
      new History<FromPortEntry_I2CWriteRead>(maxHistorySize);
    this->fromPortHistory_I2CWriteReadStatus =
      new History<FromPortEntry_I2CWriteReadStatus>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  LIDARLiteV3TesterBase ::
    ~LIDARLiteV3TesterBase(void) 
  {
  }

  void LIDARLiteV3TesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port AltimeterSend

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_AltimeterSend();
        ++_port
    ) {

      this->m_from_AltimeterSend[_port].init();
      this->m_from_AltimeterSend[_port].addCallComp(
          this,
          from_AltimeterSend_static
      );
      this->m_from_AltimeterSend[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_AltimeterSend[%d]",
          this->m_objName,
          _port
      );
      this->m_from_AltimeterSend[_port].setObjName(_portName);
#endif

    }

    // Attach input port I2CConfig

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_I2CConfig();
        ++_port
    ) {

      this->m_from_I2CConfig[_port].init();
      this->m_from_I2CConfig[_port].addCallComp(
          this,
          from_I2CConfig_static
      );
      this->m_from_I2CConfig[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_I2CConfig[%d]",
          this->m_objName,
          _port
      );
      this->m_from_I2CConfig[_port].setObjName(_portName);
#endif

    }

    // Attach input port I2CWriteRead

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_I2CWriteRead();
        ++_port
    ) {

      this->m_from_I2CWriteRead[_port].init();
      this->m_from_I2CWriteRead[_port].addCallComp(
          this,
          from_I2CWriteRead_static
      );
      this->m_from_I2CWriteRead[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_I2CWriteRead[%d]",
          this->m_objName,
          _port
      );
      this->m_from_I2CWriteRead[_port].setObjName(_portName);
#endif

    }

    // Attach input port I2CWriteReadStatus

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_I2CWriteReadStatus();
        ++_port
    ) {

      this->m_from_I2CWriteReadStatus[_port].init();
      this->m_from_I2CWriteReadStatus[_port].addCallComp(
          this,
          from_I2CWriteReadStatus_static
      );
      this->m_from_I2CWriteReadStatus[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_I2CWriteReadStatus[%d]",
          this->m_objName,
          _port
      );
      this->m_from_I2CWriteReadStatus[_port].setObjName(_portName);
#endif

    }

    // Initialize output port SchedIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_SchedIn();
        ++_port
    ) {
      this->m_to_SchedIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_SchedIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_SchedIn[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_to_SchedIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_SchedIn);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_AltimeterSend(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_AltimeterSend);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_I2CConfig(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_I2CConfig);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_I2CWriteRead(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_I2CWriteRead);
  }

  NATIVE_INT_TYPE LIDARLiteV3TesterBase ::
    getNum_from_I2CWriteReadStatus(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_I2CWriteReadStatus);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    connect_to_SchedIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputSchedPort *const SchedIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    this->m_to_SchedIn[portNum].addCallPort(SchedIn);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    invoke_to_SchedIn(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_SchedIn(),static_cast<AssertArg>(portNum));
    this->m_to_SchedIn[portNum].invoke(
        context
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool LIDARLiteV3TesterBase ::
    isConnected_to_SchedIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_SchedIn(), static_cast<AssertArg>(portNum));
    return this->m_to_SchedIn[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Drv::InputAltimeterPort *LIDARLiteV3TesterBase ::
    get_from_AltimeterSend(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_AltimeterSend(),static_cast<AssertArg>(portNum));
    return &this->m_from_AltimeterSend[portNum];
  }

  Drv::InputI2CConfigPort *LIDARLiteV3TesterBase ::
    get_from_I2CConfig(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_I2CConfig(),static_cast<AssertArg>(portNum));
    return &this->m_from_I2CConfig[portNum];
  }

  Drv::InputI2CWriteReadPort *LIDARLiteV3TesterBase ::
    get_from_I2CWriteRead(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteRead(),static_cast<AssertArg>(portNum));
    return &this->m_from_I2CWriteRead[portNum];
  }

  Drv::InputI2CWriteReadStatusPort *LIDARLiteV3TesterBase ::
    get_from_I2CWriteReadStatus(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteReadStatus(),static_cast<AssertArg>(portNum));
    return &this->m_from_I2CWriteReadStatus[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    from_AltimeterSend_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Drv::Altimeter altimeterEuData
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase = 
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    _testerBase->from_AltimeterSend_handlerBase(
        portNum,
        altimeterEuData
    );
  }

  void LIDARLiteV3TesterBase ::
    from_I2CConfig_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase = 
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    _testerBase->from_I2CConfig_handlerBase(
        portNum,
        busSpeed, slaveAddr, timeout
    );
  }

  void LIDARLiteV3TesterBase ::
    from_I2CWriteRead_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase = 
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    _testerBase->from_I2CWriteRead_handlerBase(
        portNum,
        writeBuffer, readBuffer
    );
  }

  Drv::I2CStatus LIDARLiteV3TesterBase ::
    from_I2CWriteReadStatus_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        bool shouldBlock
    )
  {
    FW_ASSERT(callComp);
    LIDARLiteV3TesterBase* _testerBase = 
      static_cast<LIDARLiteV3TesterBase*>(callComp);
    return _testerBase->from_I2CWriteReadStatus_handlerBase(
        portNum,
        shouldBlock
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_AltimeterSend->clear();
    this->fromPortHistory_I2CConfig->clear();
    this->fromPortHistory_I2CWriteRead->clear();
    this->fromPortHistory_I2CWriteReadStatus->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: AltimeterSend
  // ---------------------------------------------------------------------- 

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_AltimeterSend(
        Drv::Altimeter altimeterEuData
    )
  {
    FromPortEntry_AltimeterSend _e = {
      altimeterEuData
    };
    this->fromPortHistory_AltimeterSend->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: I2CConfig
  // ---------------------------------------------------------------------- 

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_I2CConfig(
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
    FromPortEntry_I2CConfig _e = {
      busSpeed, slaveAddr, timeout
    };
    this->fromPortHistory_I2CConfig->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: I2CWriteRead
  // ---------------------------------------------------------------------- 

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_I2CWriteRead(
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    FromPortEntry_I2CWriteRead _e = {
      writeBuffer, readBuffer
    };
    this->fromPortHistory_I2CWriteRead->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ---------------------------------------------------------------------- 
  // From port: I2CWriteReadStatus
  // ---------------------------------------------------------------------- 

  void LIDARLiteV3TesterBase ::
    pushFromPortEntry_I2CWriteReadStatus(
        bool shouldBlock
    )
  {
    FromPortEntry_I2CWriteReadStatus _e = {
      shouldBlock
    };
    this->fromPortHistory_I2CWriteReadStatus->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    from_AltimeterSend_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Drv::Altimeter altimeterEuData
    )
  {
    FW_ASSERT(portNum < this->getNum_from_AltimeterSend(),static_cast<AssertArg>(portNum));
    this->from_AltimeterSend_handler(
        portNum,
        altimeterEuData
    );
  }

  void LIDARLiteV3TesterBase ::
    from_I2CConfig_handlerBase(
        const NATIVE_INT_TYPE portNum,
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
    FW_ASSERT(portNum < this->getNum_from_I2CConfig(),static_cast<AssertArg>(portNum));
    this->from_I2CConfig_handler(
        portNum,
        busSpeed, slaveAddr, timeout
    );
  }

  void LIDARLiteV3TesterBase ::
    from_I2CWriteRead_handlerBase(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteRead(),static_cast<AssertArg>(portNum));
    this->from_I2CWriteRead_handler(
        portNum,
        writeBuffer, readBuffer
    );
  }

  Drv::I2CStatus LIDARLiteV3TesterBase ::
    from_I2CWriteReadStatus_handlerBase(
        const NATIVE_INT_TYPE portNum,
        bool shouldBlock
    )
  {
    FW_ASSERT(portNum < this->getNum_from_I2CWriteReadStatus(),static_cast<AssertArg>(portNum));
    return this->from_I2CWriteReadStatus_handler(
        portNum,
        shouldBlock
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void LIDARLiteV3TesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace Drv
