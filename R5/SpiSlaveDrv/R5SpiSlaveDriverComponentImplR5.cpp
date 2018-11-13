// ======================================================================
// \title  R5SpiSlaveDriverImpl.cpp
// \author tcanham
// \brief  cpp file for R5SpiSlaveDriver component implementation class
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


#include <R5/SpiSlaveDrv/R5SpiSlaveDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/Assert.hpp>

#include <R5/SpiSlaveDrv/SpiSlaveDrv.hpp>

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  R5SpiSlaveDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5SpiSlaveDriverComponentImpl(
        const char *const compName
    ) :
       R5SpiSlaveDriverComponentBase(compName)
#else
    R5SpiSlaveDriverImpl(void)
#endif
  {

  }

  void R5SpiSlaveDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    R5SpiSlaveDriverComponentBase::init(instance);
  }

  void R5SpiSlaveDriverComponentImpl ::
    initDriver(NATIVE_UINT_TYPE instance, NATIVE_UINT_TYPE id, NATIVE_UINT_TYPE size, Fw::MemAllocator& allocator, U16 syncByte, U32 packetSize)
  {
      U16* receivePtr = static_cast<U16*>(allocator.allocate(id, size));
      FW_ASSERT(0 != receivePtr);

      SpiSlaveDrvInit(receivePtr, (size / sizeof(U16)), syncByte, packetSize);
  }


  R5SpiSlaveDriverComponentImpl ::
    ~R5SpiSlaveDriverComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5SpiSlaveDriverComponentImpl ::
    spiRecv_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &buff
    )
  {
      buff.setsize(SpiSlaveDrvReceive((U8*)(buff.getdata()), buff.getsize()));
  }

} // end namespace R5
