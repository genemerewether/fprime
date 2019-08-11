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
#include "Fw/Types/BasicTypes.hpp"

namespace Drv {

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  bool LinuxPwmDriverComponentImpl ::
    open(NATIVE_UINT_TYPE pwmchip,
         NATIVE_UINT_TYPE * channel,
         NATIVE_UINT_TYPE channelSize,
         F32 * initDutyCycle,
         NATIVE_UINT_TYPE period_in_usecs) {
      return false;
  }

  void LinuxPwmDriverComponentImpl ::
    pwmConfig_handler(
        const NATIVE_INT_TYPE portNum,
        PwmConfig pwmConfig
    )
  {
    // TODO
  }

  void LinuxPwmDriverComponentImpl ::
    pwmSetDuty_handler(
        const NATIVE_INT_TYPE portNum,
        PwmSetDutyCycle pwmSetDutyCycle
    )
  {
    // TODO
  }

  LinuxPwmDriverComponentImpl ::
    ~LinuxPwmDriverComponentImpl(void)
  {

  }

} // end namespace Drv
