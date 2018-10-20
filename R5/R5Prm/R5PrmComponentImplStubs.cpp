// ====================================================================== 
// \title  R5PrmImpl.cpp
// \author tcanham
// \brief  cpp file for R5Prm component implementation class
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


#include <R5/R5Prm/R5PrmComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  R5PrmComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5PrmComponentImpl(
        const char *const compName
    ) :
      R5PrmComponentBase(compName)
#else
    R5PrmImpl(void)
#endif
  {

  }

  void R5PrmComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    R5PrmComponentBase::init(instance);
  }

  R5PrmComponentImpl ::
    ~R5PrmComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5PrmComponentImpl ::
    set_handler(
        const NATIVE_INT_TYPE portNum,
        R5::PrmEntry &prm,
        bool &valid
    )
  {
  }

  void R5PrmComponentImpl ::
    get_handler(
        const NATIVE_INT_TYPE portNum,
        R5::PrmEntry &prm,
        bool &valid
    )
  {
  }

  void R5PrmComponentImpl ::
    write_handler(
        const NATIVE_INT_TYPE portNum,
        bool force
    )
  {
  }

  void R5PrmComponentImpl ::
    check_handler(
        const NATIVE_INT_TYPE portNum,
        PrmWriteStatus &stat
    )
  {
  }

  //! load from flash
  void R5PrmComponentImpl ::
    load(void) {

  }

} // end namespace R5
