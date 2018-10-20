// ====================================================================== 
// \title  R5TimeImpl.hpp
// \author tcanham
// \brief  hpp file for R5Time component implementation class
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

#ifndef R5Time_HPP
#define R5Time_HPP

#include "R5/R5Time/R5TimeComponentAc.hpp"

namespace R5 {

  class R5TimeComponentImpl :
    public R5TimeComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5Time
      //!
      R5TimeComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5Time
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object R5Time
      //!
      ~R5TimeComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

    //! Handler implementation for timeGetPort
    //!
    void timeGetPort_handler(
        const NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::Time &time /*!< The U32 cmd argument*/
    );

    //! Handler implementation for timeSetPort
    //!
//     void timeSetPort_handler(
//         const NATIVE_INT_TYPE portNum, /*!< The port number*/
//         Fw::Time &time /*!< The time to set the system time to*/
//     );

    //! Handler implementation for timeStartPort
    //!
//     void timeStartPort_handler(
//         const NATIVE_INT_TYPE portNum, /*!< The port number*/
//         Fw::TimeAction action /*!< Whether to start or stop clocks*/
//     );
  };
} // end namespace R5

#endif
