// ======================================================================
// \title  BlspGpioDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspGpioDriver component implementation class
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


#include <SnapdragonFlight/BlspGpioDriver/BlspGpioDriverComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  BlspGpioDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    BlspGpioDriverComponentImpl(
        const char *const compName
    ) :
      BlspGpioDriverComponentBase(compName)
#else
    BlspGpioDriverImpl(void)
#endif
    ,m_gpio(-1)
    ,m_direction(GPIO_IN)
    ,m_fd(-1)
    ,m_quitThread(false)
  {

  }

  void BlspGpioDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    BlspGpioDriverComponentBase::init(instance);
  }

} // end namespace SnapdragonFlight
