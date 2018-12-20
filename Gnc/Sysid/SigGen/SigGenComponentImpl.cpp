// ====================================================================== 
// \title  SigGenImpl.cpp
// \author mereweth
// \brief  cpp file for SigGen component implementation class
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


#include <Gnc/Sysid/SigGen/SigGenComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  SigGenComponentImpl ::
#if FW_OBJECT_NAMES == 1
    SigGenComponentImpl(
        const char *const compName
    ) :
      SigGenComponentBase(compName)
#else
    SigGenImpl(void)
#endif
  {

  }

  void SigGenComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    SigGenComponentBase::init(instance);
  }

  SigGenComponentImpl ::
    ~SigGenComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void SigGenComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

} // end namespace Gnc
