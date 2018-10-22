// ====================================================================== 
// \title  R5TimeImpl.cpp
// \author tcanham
// \brief  cpp file for R5Time component implementation class
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

#include <R5/R5Time/R5TimeComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5 {

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void R5TimeComponentImpl::timeGetPort_handler(const NATIVE_INT_TYPE portNum,
            Fw::Time &time) {
    }

    void R5TimeComponentImpl::timeSetPort_handler(const NATIVE_INT_TYPE portNum,
            Fw::Time &time) {
    }

    void R5TimeComponentImpl::timeStartPort_handler(
            const NATIVE_INT_TYPE portNum, Fw::TimeAction action) {
    }

} // end namespace R5
