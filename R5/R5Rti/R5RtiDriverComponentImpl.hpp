// ====================================================================== 
// \title  R5RtiDriverImpl.hpp
// \author kubiak
// \brief  hpp file for R5RtiDriver component implementation class
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

#ifndef R5RtiDriver_HPP
#define R5RtiDriver_HPP

#include "R5/R5Rti/R5RtiDriverComponentAc.hpp"

namespace R5 {

  class R5RtiDriverComponentImpl :
    public R5RtiDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5RtiDriver
      //!
      R5RtiDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5RtiDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object R5RtiDriver
      //!
      ~R5RtiDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

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
