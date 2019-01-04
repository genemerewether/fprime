// ====================================================================== 
// \title  ActiveDecouplerImpl.cpp
// \author mereweth
// \brief  cpp file for ActiveDecoupler component implementation class
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


#include <Svc/ActiveDecoupler/ActiveDecouplerComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ActiveDecouplerComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ActiveDecouplerComponentImpl(
        const char *const compName
    ) :
      ActiveDecouplerComponentBase(compName)
#else
    ActiveDecouplerImpl(void)
#endif
  {

  }

  void ActiveDecouplerComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE msgSize,
        const NATIVE_INT_TYPE instance
    ) 
  {
    ActiveDecouplerComponentBase::init(queueDepth, msgSize, instance);
  }

  ActiveDecouplerComponentImpl ::
    ~ActiveDecouplerComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void ActiveDecouplerComponentImpl ::
    DataIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  { 
      if (isConnected_DataOut_OutputPort(portNum)) {
          this->DataOut_out(portNum, Buffer);
      }
  }

} // end namespace Svc
