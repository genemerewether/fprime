// ====================================================================== 
// \title  R5A2DDriverImpl.hpp
// \author tcanham
// \brief  hpp file for R5A2DDriver component implementation class
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

#ifndef R5A2DDriver_HPP
#define R5A2DDriver_HPP

#include "R5/A2DDrv/R5A2DDriverComponentAc.hpp"

namespace R5 {

  class R5A2DDriverComponentImpl :
    public R5A2DDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5A2DDriver
      //!
      R5A2DDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5A2DDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object R5A2DDriver
      //!
      ~R5A2DDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for get
      //!
      void get_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          A2DGetBank bank,
          U32 channel, 
          F32 &val,
          U16 &raw
      );

    };

} // end namespace R5

#endif
