// ====================================================================== 
// \title  KraitRouterImpl.cpp
// \author vagrant
// \brief  cpp file for KraitRouter component implementation class
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


#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <unistd.h>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  KraitRouterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    KraitRouterComponentImpl(
        const char *const compName
    ) :
      KraitRouterComponentBase(compName),
      m_initialized(false)
#else
    KraitRouterImpl(void)
#endif
  {

  }

  void KraitRouterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    KraitRouterComponentBase::init(instance);
    m_initialized = true;
  }

  KraitRouterComponentImpl ::
    ~KraitRouterComponentImpl(void)
  {
    m_initialized = false;
  }
  
  int KraitRouterComponentImpl::buffRead(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    DEBUG_PRINT("buffRead called on object 0x%X, init %d\n", (unsigned long) this, this->m_initialized);
    while (!this->m_initialized) {
      usleep(1000);
    }
    *port = 0;
    *bytes = 0;
    return 1;
  }
  
  int KraitRouterComponentImpl::portRead(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    DEBUG_PRINT("portRead called on object 0x%X, init %d\n", (unsigned long) this, this->m_initialized);
    while (!this->m_initialized) {
      usleep(1000);
    }
    *port = 0;
    *bytes = 0;
    return 1;
  }
  
  int KraitRouterComponentImpl::write(unsigned int port, const unsigned char* buff, int buffLen) {
    DEBUG_PRINT("write called on object 0x%X, port %d, init %d\n", (unsigned long) this, port, this->m_initialized);
    while (!this->m_initialized) {
      usleep(1000);
    }
    // if connected, call output port
    if (this->isConnected_KraitPortsOut_OutputPort(port)) {
      Fw::ExternalSerializeBuffer portBuff((unsigned char*) buff, buffLen);

      DEBUG_PRINT("Calling port %d with %d bytes.\n", port, buffLen);
      Fw::SerializeStatus stat = this->KraitPortsOut_out(port, portBuff);
      if (stat != Fw::FW_SERIALIZE_OK) {
	DEBUG_PRINT("KraitPortsOut_out() serialize status error\n");
	// TODO(mereweth) - status codes
	return -1;
      }
    }
    return 0;
  }
  
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void KraitRouterComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void KraitRouterComponentImpl ::
    HexPortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
    // TODO
  }

} // end namespace SnapdragonFlight
