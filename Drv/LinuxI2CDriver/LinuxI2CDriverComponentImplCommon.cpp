// ======================================================================
// \title  LinuxI2CDriverImpl.cpp
// \author tcanham
// \brief  cpp file for LinuxI2CDriver component implementation class
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

#include <Drv/LinuxI2CDriver/LinuxI2CDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Drv {

    // ----------------------------------------------------------------------
    // Construction, initialization, and destruction
    // ----------------------------------------------------------------------

    LinuxI2CDriverComponentImpl::
#if FW_OBJECT_NAMES == 1
    LinuxI2CDriverComponentImpl(const char * const compName) :
            LinuxI2CDriverComponentBase(compName)
#else
    LinuxI2CDriverImpl(void)
#endif
    ,m_fd(-1),m_device(-1),m_addr(0u), m_readBytes(0),m_writeBytes(0)
    {

    }

    void LinuxI2CDriverComponentImpl::init(const NATIVE_INT_TYPE instance) {
        LinuxI2CDriverComponentBase::init(instance);
    }


} // end namespace Drv
