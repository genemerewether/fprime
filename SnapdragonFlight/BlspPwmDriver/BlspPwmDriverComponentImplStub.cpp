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
#include "Fw/Types/BasicTypes.hpp"

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  bool BlspPwmDriverComponentImpl ::
    open(NATIVE_UINT_TYPE pwmchip,
         NATIVE_UINT_TYPE * channel,
         NATIVE_UINT_TYPE channelSize,
         F32 * initDutyCycle,
         NATIVE_UINT_TYPE period_in_usecs) {
      return false;
  }

  void BlspPwmDriverComponentImpl ::
    pwmConfig_handler(
        const NATIVE_INT_TYPE portNum,
        Drv::PwmConfig pwmConfig
    )
  {
    // TODO
  }

  void BlspPwmDriverComponentImpl ::
    pwmSetDuty_handler(
        const NATIVE_INT_TYPE portNum,
        Drv::PwmSetDutyCycle pwmSetDutyCycle
    )
  {
    // TODO
  }

  BlspPwmDriverComponentImpl ::
    ~BlspPwmDriverComponentImpl(void)
  {

  }

} // end namespace SnapdragonFlight
