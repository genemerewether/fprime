// ======================================================================
// \title  PassiveRateGroup/test/ut/TesterBase.cpp
// \author Auto-generated
// \brief  cpp file for PassiveRateGroup component test harness base class
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

  PassiveRateGroupTesterBase ::
    PassiveRateGroupTesterBase(
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
    this->fromPortHistory_RateGroupMemberOut =
      new History<FromPortEntry_RateGroupMemberOut>(maxHistorySize);
    // Clear history
    this->clearHistory();
  }

  PassiveRateGroupTesterBase ::
    ~PassiveRateGroupTesterBase(void) 
  {
  }

  void PassiveRateGroupTesterBase ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {

    // Initialize base class

		Fw::PassiveComponentBase::init(instance);

    // Attach input port RateGroupMemberOut

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_from_RateGroupMemberOut();
        ++_port
    ) {

      this->m_from_RateGroupMemberOut[_port].init();
      this->m_from_RateGroupMemberOut[_port].addCallComp(
          this,
          from_RateGroupMemberOut_static
      );
      this->m_from_RateGroupMemberOut[_port].setPortNum(_port);

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      (void) snprintf(
          _portName,
          sizeof(_portName),
          "%s_from_RateGroupMemberOut[%d]",
          this->m_objName,
          _port
      );
      this->m_from_RateGroupMemberOut[_port].setObjName(_portName);
#endif

    }

    // Initialize output port CycleIn

    for (
        NATIVE_INT_TYPE _port = 0;
        _port < this->getNum_to_CycleIn();
        ++_port
    ) {
      this->m_to_CycleIn[_port].init();

#if FW_OBJECT_NAMES == 1
      char _portName[80];
      snprintf(
          _portName,
          sizeof(_portName),
          "%s_to_CycleIn[%d]",
          this->m_objName,
          _port
      );
      this->m_to_CycleIn[_port].setObjName(_portName);
#endif

    }

  }

  // ----------------------------------------------------------------------
  // Getters for port counts
  // ----------------------------------------------------------------------

  NATIVE_INT_TYPE PassiveRateGroupTesterBase ::
    getNum_to_CycleIn(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_to_CycleIn);
  }

  NATIVE_INT_TYPE PassiveRateGroupTesterBase ::
    getNum_from_RateGroupMemberOut(void) const
  {
    return (NATIVE_INT_TYPE) FW_NUM_ARRAY_ELEMENTS(this->m_from_RateGroupMemberOut);
  }

  // ----------------------------------------------------------------------
  // Connectors for to ports 
  // ----------------------------------------------------------------------

  void PassiveRateGroupTesterBase ::
    connect_to_CycleIn(
        const NATIVE_INT_TYPE portNum,
        Svc::InputCyclePort *const CycleIn
    ) 
  {
    FW_ASSERT(portNum < this->getNum_to_CycleIn(),static_cast<AssertArg>(portNum));
    this->m_to_CycleIn[portNum].addCallPort(CycleIn);
  }


  // ----------------------------------------------------------------------
  // Invocation functions for to ports
  // ----------------------------------------------------------------------

  void PassiveRateGroupTesterBase ::
    invoke_to_CycleIn(
        const NATIVE_INT_TYPE portNum,
        Svc::TimerVal &cycleStart
    )
  {
    FW_ASSERT(portNum < this->getNum_to_CycleIn(),static_cast<AssertArg>(portNum));
    FW_ASSERT(portNum < this->getNum_to_CycleIn(),static_cast<AssertArg>(portNum));
    this->m_to_CycleIn[portNum].invoke(
        cycleStart
    );
  }

  // ----------------------------------------------------------------------
  // Connection status for to ports
  // ----------------------------------------------------------------------

  bool PassiveRateGroupTesterBase ::
    isConnected_to_CycleIn(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_to_CycleIn(), static_cast<AssertArg>(portNum));
    return this->m_to_CycleIn[portNum].isConnected();
  }

  // ----------------------------------------------------------------------
  // Getters for from ports
  // ----------------------------------------------------------------------
 
  Svc::InputSchedPort *PassiveRateGroupTesterBase ::
    get_from_RateGroupMemberOut(const NATIVE_INT_TYPE portNum)
  {
    FW_ASSERT(portNum < this->getNum_from_RateGroupMemberOut(),static_cast<AssertArg>(portNum));
    return &this->m_from_RateGroupMemberOut[portNum];
  }

  // ----------------------------------------------------------------------
  // Static functions for from ports
  // ----------------------------------------------------------------------

  void PassiveRateGroupTesterBase ::
    from_RateGroupMemberOut_static(
        Fw::PassiveComponentBase *const callComp,
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(callComp);
    PassiveRateGroupTesterBase* _testerBase = 
      static_cast<PassiveRateGroupTesterBase*>(callComp);
    _testerBase->from_RateGroupMemberOut_handlerBase(
        portNum,
        context
    );
  }

  // ----------------------------------------------------------------------
  // Histories for typed from ports
  // ----------------------------------------------------------------------

  void PassiveRateGroupTesterBase ::
    clearFromPortHistory(void)
  {
    this->fromPortHistorySize = 0;
    this->fromPortHistory_RateGroupMemberOut->clear();
  }

  // ---------------------------------------------------------------------- 
  // From port: RateGroupMemberOut
  // ---------------------------------------------------------------------- 

  void PassiveRateGroupTesterBase ::
    pushFromPortEntry_RateGroupMemberOut(
        NATIVE_UINT_TYPE context
    )
  {
    FromPortEntry_RateGroupMemberOut _e = {
      context
    };
    this->fromPortHistory_RateGroupMemberOut->push_back(_e);
    ++this->fromPortHistorySize;
  }

  // ----------------------------------------------------------------------
  // Handler base functions for from ports
  // ----------------------------------------------------------------------

  void PassiveRateGroupTesterBase ::
    from_RateGroupMemberOut_handlerBase(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    FW_ASSERT(portNum < this->getNum_from_RateGroupMemberOut(),static_cast<AssertArg>(portNum));
    this->from_RateGroupMemberOut_handler(
        portNum,
        context
    );
  }

  // ----------------------------------------------------------------------
  // History 
  // ----------------------------------------------------------------------

  void PassiveRateGroupTesterBase ::
    clearHistory()
  {
    this->clearFromPortHistory();
  }

} // end namespace Svc
