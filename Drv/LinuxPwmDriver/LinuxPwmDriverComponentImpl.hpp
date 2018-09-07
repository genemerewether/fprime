// ======================================================================
// \title  LinuxPwmDriverImpl.hpp
// \author mereweth
// \brief  hpp file for LinuxPwmDriver component implementation class
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

#ifndef LinuxPwmDriver_HPP
#define LinuxPwmDriver_HPP

#include "Drv/LinuxPwmDriver/LinuxPwmDriverComponentAc.hpp"

namespace Drv {

  class LinuxPwmDriverComponentImpl :
    public LinuxPwmDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object LinuxPwmDriver
      //!
      LinuxPwmDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object LinuxPwmDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object LinuxPwmDriver
      //!
      ~LinuxPwmDriverComponentImpl(void);

      //! Open PWM chip
      //!
      bool open(NATIVE_INT_TYPE pwmchip);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for pwmConfig
      //!
      void pwmConfig_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 period
      );

      //! Handler implementation for pwmSetDuty
      //!
      void pwmSetDuty_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          F32 dutyCycle,
          U32 bitmask
      );

      //! store pwmchip ID
      NATIVE_INT_TYPE m_pwmchip;
      //! file descriptor for GPIO
      NATIVE_INT_TYPE m_fd;

    };

} // end namespace Drv

#endif
