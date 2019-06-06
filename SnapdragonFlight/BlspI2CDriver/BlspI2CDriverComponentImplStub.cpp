// ====================================================================== 
// \title  BlspI2CDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspI2CDriver component implementation class
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

#include <SnapdragonFlight/BlspI2CDriver/BlspI2CDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace SnapdragonFlight {

    void BlspI2CDriverComponentImpl::open(NATIVE_INT_TYPE device) {

    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void BlspI2CDriverComponentImpl ::
      I2CReadWrite_handler(
	  const NATIVE_INT_TYPE portNum,
	  Fw::Buffer &writeBuffer,
	  Fw::Buffer &readBuffer
      )
    {
      // TODO
    }

    void BlspI2CDriverComponentImpl ::
      I2CConfig_handler(
	  const NATIVE_INT_TYPE portNum,
	  U32 busSpeed,
	  U32 slaveAddr,
	  U32 timeout
      )
    {
      // TODO
    }

    BlspI2CDriverComponentImpl::~BlspI2CDriverComponentImpl(void) {

    }

} // end namespace SnapdragonFlight
