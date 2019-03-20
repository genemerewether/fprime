// ======================================================================
// \title  BlspSerialDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspSerialDriver component implementation class
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


#include <SnapdragonFlight/BlspSerialDriver/BlspSerialDriverComponentImpl.hpp>

namespace SnapdragonFlight {

  BlspSerialDriverComponentImpl ::
    ~BlspSerialDriverComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void BlspSerialDriverComponentImpl ::
    serialSend_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    // TODO
  }

  void BlspSerialDriverComponentImpl ::
    startReadThread(NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize, NATIVE_INT_TYPE cpuAffinity) {
  }

  void BlspSerialDriverComponentImpl::open(NATIVE_UINT_TYPE device, UartBaudRate baud, UartFlowControl fc, bool block) {

  }

  void BlspSerialDriverComponentImpl ::
      quitReadThread(void) {
  }

} // end namespace Snapdragon
