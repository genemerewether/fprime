// ======================================================================
// \title  LinuxPwmDriverImpl.cpp
// \author tcanham
// \brief  cpp file for LinuxPwmDriver component implementation class
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


#include <Drv/LinuxPwmDriver/LinuxPwmDriverComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  LinuxPwmDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    LinuxPwmDriverComponentImpl(
        const char *const compName
    ) :
      LinuxPwmDriverComponentBase(compName)
#else
    LinuxPwmDriverImpl(void)
#endif
    ,m_pwmchip(-1)
    ,m_fd(-1)
    ,m_handle(NULL)
  {

  }

  void LinuxPwmDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    LinuxPwmDriverComponentBase::init(instance);
  }

} // end namespace Drv
