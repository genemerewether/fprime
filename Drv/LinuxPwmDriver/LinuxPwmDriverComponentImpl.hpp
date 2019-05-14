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
      bool open(NATIVE_UINT_TYPE pwmchip,
                NATIVE_UINT_TYPE * channel,
                NATIVE_UINT_TYPE channelSize,
                F32 * initDutyCycle,
                NATIVE_UINT_TYPE period_in_usecs);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for pwmConfig
      //!
      void pwmConfig_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          PwmConfig pwmConfig
      );

      //! Handler implementation for pwmSetDuty
      //!
      void pwmSetDuty_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          PwmSetDutyCycle pwmSetDutyCycle
      );

      //! store pwmchip ID
      NATIVE_INT_TYPE m_pwmchip;
      //! file descriptor for GPIO
      NATIVE_INT_TYPE m_fd;

      NATIVE_UINT_TYPE m_periodInUsecs;

      //! Implementation-specific handle
      void * m_handle;

      U32 m_numGpios;

    };

} // end namespace Drv

#endif
