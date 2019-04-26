// ====================================================================== 
// \title  FixedAxisSe3AdapterImpl.cpp
// \author gene
// \brief  cpp file for FixedAxisSe3Adapter component implementation class
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


#include <Gnc/Utils/FixedAxisSe3Adapter/FixedAxisSe3AdapterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  FixedAxisSe3AdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    FixedAxisSe3AdapterComponentImpl(
        const char *const compName
    ) :
      FixedAxisSe3AdapterComponentBase(compName)
#else
    FixedAxisSe3AdapterImpl(void)
#endif
  {

  }

  void FixedAxisSe3AdapterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    FixedAxisSe3AdapterComponentBase::init(instance);
  }

  FixedAxisSe3AdapterComponentImpl ::
    ~FixedAxisSe3AdapterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void FixedAxisSe3AdapterComponentImpl ::
    flatOutput_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void FixedAxisSe3AdapterComponentImpl ::
    AXSE3ADAP_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    // TODO
    this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

} // end namespace Gnc
