// ======================================================================
// \title  Errors.hpp
// \author bocchino, mereweth
// \brief  Interface for BufferLogger error tests
//
// \copyright
// Copyright (C) 2017 California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef Svc_Errors_HPP
#define Svc_Errors_HPP

#include "Tester.hpp"

namespace Svc {

  namespace Errors {

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

        //! Log file open error
        void LogFileOpen(void);

        //! Log file write error
        void LogFileWrite(void);

        //! Log file validation error
        void LogFileValidation(void);

    };

  }

}

#endif
