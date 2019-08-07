// ====================================================================== 
// \title  R5RtiImpl.hpp
// \author kubiak
// \brief  hpp file for R5Rti component implementation class
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

#ifndef R5Rti_HPP
#define R5Rti_HPP

#include "R5/R5Rti/R5RtiComponentAc.hpp"

namespace R5 {

  class R5RtiComponentImpl :
    public R5RtiComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5Rti
      //!
      R5RtiComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5Rti
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object R5Rti
      //!
      ~R5RtiComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for wait
      //!
      void wait_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool &alreadySet 
      );


    };

} // end namespace 

#endif
