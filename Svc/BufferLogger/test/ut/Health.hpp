// ======================================================================
// \title  Health.hpp
// \author bocchino
// \brief  Interface for BufferLogger health tests
//
// \copyright
// Copyright (C) 2017 California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef Svc_Health_HPP
#define Svc_Health_HPP

#include "Tester.hpp"

namespace Svc {

  namespace Health {

    class Tester :
      public Svc::Tester
    {

      public:

        //! Construct object Tester
        //!
        Tester(
            BufferLoggerFileMode writeMode = BL_REGULAR_WRITE,  //! The write mode - direct, bulk, or looping
            BufferLoggerCloseMode closeMode = BL_CLOSE_SYNC,
            U32 directChunkSize = 512, //! The filesystem chunk size - direct mode uses this
            bool doInitLog = true
        );

        // ----------------------------------------------------------------------
        // Tests
        // ----------------------------------------------------------------------

        //! Health ping test
        void Ping(void);

    };

  }

}

#endif
