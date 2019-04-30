// ====================================================================== 
// \title  ImuProcImpl.cpp
// \author mereweth
// \brief  cpp file for ImuProc component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
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


#include <Gnc/Utils/ImuProc/ImuProcComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ImuProcComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ImuProcComponentImpl(
        const char *const compName
    ) :
      ImuProcComponentBase(compName)
#else
    ImuProcImpl(void)
#endif
  {

  }

  void ImuProcComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    ImuProcComponentBase::init(instance);
  }

  ImuProcComponentImpl ::
    ~ImuProcComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ImuProcComponentImpl ::
    HighRateImu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void ImuProcComponentImpl ::
    IMUPROC_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

} // end namespace Gnc
