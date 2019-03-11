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

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    BlspSpiDriverComponentImpl::
#if FW_OBJECT_NAMES == 1
    BlspSpiDriverComponentImpl(const char * const compName) :
            BlspSpiDriverComponentBase(compName)
#else
    BlspSpiDriverImpl(void)
#endif
    ,m_fd(-1),m_device(-1),m_bytes(0)
    {

    }

    void BlspSpiDriverComponentImpl::init(const NATIVE_INT_TYPE instance) {
        BlspSpiDriverComponentBase::init(instance);
    }


} // end namespace SnapdragonFlight
