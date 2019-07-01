// ====================================================================== 
// \title  R5RtiComponentImplStub.cpp
// \author kubiak
// \brief  cpp file for R5RtiDriver component implementation class
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


#include <R5/R5Rti/R5RtiComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  R5RtiComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5RtiComponentImpl(
        const char *const compName
    ) :
      R5RtiComponentBase(compName)
#else
    R5RtiComponentImpl(void)
#endif
  {

  }

  void R5RtiComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    R5RtiComponentBase::init(instance);
  }

  R5RtiComponentImpl ::
    ~R5RtiComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5RtiComponentImpl ::
    wait_handler(
        const NATIVE_INT_TYPE portNum,
        bool &alreadySet
    )
  {
    alreadySet = false;
  }

} // end namespace R5
