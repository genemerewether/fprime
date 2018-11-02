// ====================================================================== 
// \title  R5GpioAdapterImpl.cpp
// \author mereweth
// \brief  cpp file for R5GpioAdapter component implementation class
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


#include <R5/GpioAdapter/R5GpioAdapterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  R5GpioAdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5GpioAdapterComponentImpl(
        const char *const compName
    ) :
      R5GpioAdapterComponentBase(compName)
#else
    R5GpioAdapterImpl(void)
#endif
  {

  }

  void R5GpioAdapterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    R5GpioAdapterComponentBase::init(instance);
  }

  R5GpioAdapterComponentImpl ::
    ~R5GpioAdapterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5GpioAdapterComponentImpl ::
    setIn_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
    // TODO
  }

  void R5GpioAdapterComponentImpl ::
    getIn_handler(
        const NATIVE_INT_TYPE portNum,
        bool &state
    )
  {
    // TODO
  }

  void R5GpioAdapterComponentImpl ::
    waitIn_handler(
        const NATIVE_INT_TYPE portNum,
        GpioWaitBank bank,
        U32 bit,
        bool &alreadySet
    )
  {
    // TODO
  }

} // end namespace R5
