// ====================================================================== 
// \title  QueuedDecouplerImpl.cpp
// \author mereweth
// \brief  cpp file for QueuedDecoupler component implementation class
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


#include <Svc/QueuedDecoupler/QueuedDecouplerComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/Assert.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  QueuedDecouplerComponentImpl ::
#if FW_OBJECT_NAMES == 1
    QueuedDecouplerComponentImpl(
        const char *const compName
    ) :
      QueuedDecouplerComponentBase(compName)
#else
    QueuedDecouplerImpl(void)
#endif
  {

  }

  void QueuedDecouplerComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE msgSize,
        const NATIVE_INT_TYPE instance
    ) 
  {
    QueuedDecouplerComponentBase::init(queueDepth, msgSize, instance);
  }

  QueuedDecouplerComponentImpl ::
    ~QueuedDecouplerComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------
  
  void QueuedDecouplerComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      bool quitLoop = false;
      while (!quitLoop) {
	  MsgDispatchStatus loopStatus = this->doDispatch();
	  switch (loopStatus) {
	      case MSG_DISPATCH_OK: // if normal message processing, continue
		  break;
	      case MSG_DISPATCH_EMPTY:
	      case MSG_DISPATCH_EXIT:
		  quitLoop = true;
		  break;
	      default:
		  FW_ASSERT(0,(NATIVE_INT_TYPE)loopStatus);
	  }
      }
  }
    
  void QueuedDecouplerComponentImpl ::
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
