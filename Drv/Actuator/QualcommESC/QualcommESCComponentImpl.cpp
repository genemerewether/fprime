// ======================================================================
// \title  QualcommESCComponentImpl.cpp
// \author genemerewether
// \brief  cpp file for QualcommESC component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Drv/Actuator/QualcommESC/QualcommESCComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  QualcommESCComponentImpl ::
#if FW_OBJECT_NAMES == 1
    QualcommESCComponentImpl(
        const char *const compName
    ) :
      QualcommESCComponentBase(compName)
#else
    QualcommESCComponentImpl(void)
#endif
  {

  }

  void QualcommESCComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    QualcommESCComponentBase::init(instance);
  }

  QualcommESCComponentImpl ::
    ~QualcommESCComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void QualcommESCComponentImpl ::
    prmTrigger_handler(
        const NATIVE_INT_TYPE portNum,
        FwPrmIdType dummy
    )
  {
    // TODO
  }

  void QualcommESCComponentImpl ::
    motor_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Actuators &Actuators
    )
  {
    // TODO
  }

  void QualcommESCComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void QualcommESCComponentImpl ::
    SerReadPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        SerialReadStatus &status
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void QualcommESCComponentImpl ::
    QCESC_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

  void QualcommESCComponentImpl ::
    QCESC_Arm_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        bool armState
    )
  {
    // TODO
    this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

} // end namespace Drv
