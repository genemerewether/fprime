// ======================================================================
// \title  R5A2DDriverImpl.cpp
// \author tcanham
// \brief  cpp file for R5A2DDriver component implementation class
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


#include <R5/A2DDrv/R5A2DDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5 {

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5A2DDriverComponentImpl ::
    get_handler(
        const NATIVE_INT_TYPE portNum,
        A2DGetBank bank,
        U32 channel,
        F32 &val,
        U16 &raw
    )
  {
    raw = 0xFFFF;
    val = 1.23;
  }

} // end namespace R5
