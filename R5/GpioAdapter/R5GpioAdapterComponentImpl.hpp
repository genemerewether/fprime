// ======================================================================
// \title  R5GpioAdapterImpl.hpp
// \author mereweth
// \brief  hpp file for R5GpioAdapter component implementation class
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

#ifndef R5GpioAdapter_HPP
#define R5GpioAdapter_HPP

#include "R5/GpioAdapter/R5GpioAdapterComponentAc.hpp"

namespace R5 {

  class R5GpioAdapterComponentImpl :
    public R5GpioAdapterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5GpioAdapter
      //!
      R5GpioAdapterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5GpioAdapter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object R5GpioAdapter
      //!
      ~R5GpioAdapterComponentImpl(void);

      void waitMapping(GpioWaitBank bank, U32 bit);
      void getMapping(GpioGetBank bank, U32 bit);
      void setMapping(GpioSetBank bank, U32 bit);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for setIn
      //!
      void setIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool state
      );

      //! Handler implementation for getIn
      //!
      void getIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool &state
      );

      //! Handler implementation for waitIn
      //!
      void waitIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool &alreadySet
      );

      GpioWaitBank m_waitBank;
      U32 m_waitBit;
      bool m_waitValid;

      GpioGetBank m_getBank;
      U32 m_getBit;
      bool m_getValid;

      GpioSetBank m_setBank;
      U32 m_setBit;
      bool m_setValid;

    };

} // end namespace R5

#endif
