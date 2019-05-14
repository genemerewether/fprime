// ======================================================================
// \title  BlspPwmDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspPwmDriver component implementation class
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


#include <SnapdragonFlight/BlspPwmDriver/BlspPwmDriverComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  BlspPwmDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    BlspPwmDriverComponentImpl(
        const char *const compName
    ) :
      BlspPwmDriverComponentBase(compName)
#else
    BlspPwmDriverImpl(void)
#endif
    ,m_pwmchip(-1)
    ,m_fd(-1)
    ,m_periodInUsecs(0u)
    ,m_handle(NULL)
    ,m_numGpios(0u)
  {

  }

  void BlspPwmDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    BlspPwmDriverComponentBase::init(instance);
  }

} // end namespace SnapdragonFlight
