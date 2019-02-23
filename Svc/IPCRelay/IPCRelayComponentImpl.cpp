// ======================================================================
// \title  IPCRelayImpl.cpp
// \author mereweth
// \brief  cpp file for IPCRelay component implementation class
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


#include <Svc/IPCRelay/IPCRelayComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  IPCRelayComponentImpl ::
#if FW_OBJECT_NAMES == 1
    IPCRelayComponentImpl(
        const char *const compName
    ) :
      IPCRelayComponentBase(compName)
#else
    IPCRelayImpl(void)
#endif
  {

  }

  void IPCRelayComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE msgSize,
        const NATIVE_INT_TYPE instance
    )
  {
    IPCRelayComponentBase::init(queueDepth, msgSize, instance);
  }

  IPCRelayComponentImpl ::
    ~IPCRelayComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void IPCRelayComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void IPCRelayComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
      this->pingOut_out(0, key);
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void IPCRelayComponentImpl ::
    proc1In_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      DEBUG_PRINT("IPCRelay received on port %d\n", portNum);
      this->proc2Out_out(portNum, Buffer);
  }

} // end namespace Svc
