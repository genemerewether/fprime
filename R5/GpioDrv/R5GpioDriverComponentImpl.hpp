// ======================================================================
// \title  R5GpioDriverImpl.hpp
// \author tcanham
// \brief  hpp file for R5GpioDriver component implementation class
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

#ifndef R5GpioDriver_HPP
#define R5GpioDriver_HPP

#include "R5/GpioDrv/R5GpioDriverComponentAc.hpp"

namespace R5 {

  class R5GpioDriverComponentImpl :
    public R5GpioDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5GpioDriver
      //!
      R5GpioDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5GpioDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object R5GpioDriver
      //!
      ~R5GpioDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for set
      //!
      void set_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          GpioSetBank bank,
          U32 bit,
          GpioSetVal val
      );

      //! Handler implementation for get
      //!
      void get_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          GpioGetBank bank,
          U32 bit,
          GpioGetVal &val
      );

      //! Handler implementation for wait
      //!
      void wait_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          GpioWaitBank bank,
          U32 bit,
          bool &alreadySet
      );


    };

} // end namespace R5

#endif
