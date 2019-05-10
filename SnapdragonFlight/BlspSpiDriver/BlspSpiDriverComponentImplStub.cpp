// ====================================================================== 
// \title  BlspSpiDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspSpiDriver component implementation class
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

#include <SnapdragonFlight/BlspSpiDriver/BlspSpiDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace SnapdragonFlight {

    void BlspSpiDriverComponentImpl::open(NATIVE_INT_TYPE device, SpiFrequency clock) {

    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void BlspSpiDriverComponentImpl ::
      SpiConfig_handler(
          const NATIVE_INT_TYPE portNum,
          U32 busSpeed
      )
    {
    }
      
    void BlspSpiDriverComponentImpl::SpiReadWrite_handler(
            const NATIVE_INT_TYPE portNum, Fw::Buffer &WriteBuffer,
            Fw::Buffer &readBuffer) {
        // TODO
    }

    BlspSpiDriverComponentImpl::~BlspSpiDriverComponentImpl(void) {

    }

} // end namespace SnapdragonFlight
