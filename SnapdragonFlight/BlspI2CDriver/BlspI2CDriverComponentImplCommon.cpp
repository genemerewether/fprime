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

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    BlspI2CDriverComponentImpl::
#if FW_OBJECT_NAMES == 1
    BlspI2CDriverComponentImpl(const char * const compName) :
            BlspI2CDriverComponentBase(compName)
#else
    BlspI2CDriverImpl(void)
#endif
      ,m_fd(-1),m_device(-1),m_addr(0u),m_readBytes(0u),m_writeBytes(0u)
    {

    }

    void BlspI2CDriverComponentImpl::init(const NATIVE_INT_TYPE instance) {
        BlspI2CDriverComponentBase::init(instance);
    }


} // end namespace SnapdragonFlight
