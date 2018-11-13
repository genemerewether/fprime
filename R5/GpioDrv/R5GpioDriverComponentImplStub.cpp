// ======================================================================
// \title  R5GpioDriverImpl.cpp
// \author tcanham
// \brief  cpp file for R5GpioDriver component implementation class
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

#include <R5/GpioDrv/R5GpioDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/Assert.hpp>

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  R5GpioDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5GpioDriverComponentImpl(
        const char *const compName
    ) :
      R5GpioDriverComponentBase(compName)
#else
    R5GpioDriverImpl(void)
#endif
  {

  }

  void R5GpioDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    R5GpioDriverComponentBase::init(instance);
  }

  R5GpioDriverComponentImpl ::
    ~R5GpioDriverComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5GpioDriverComponentImpl ::
    set_handler(
        const NATIVE_INT_TYPE portNum,
        GpioSetBank bank,
        U32 bit,
        GpioSetVal val
    )
  {
  }

  void R5GpioDriverComponentImpl ::
    get_handler(
        const NATIVE_INT_TYPE portNum,
        GpioGetBank bank,
        U32 bit,
        GpioGetVal &val
    )
  {
  }

  void R5GpioDriverComponentImpl ::
    wait_handler(
        const NATIVE_INT_TYPE portNum,
        GpioWaitBank bank,
        U32 bit,
        bool &alreadySet
    )
  {
  }

} // end namespace R5
