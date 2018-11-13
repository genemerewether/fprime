// ======================================================================
// \title  R5UartDriverImpl.cpp
// \author tcanham
// \brief  cpp file for R5UartDriver component implementation class
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


#include <R5/UartDrv/R5UartDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/Assert.hpp>

#include <R5/UartDrv/UartDrv.hpp>

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  R5UartDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5UartDriverComponentImpl(
        const char *const compName
    ) :
      R5UartDriverComponentBase(compName)
#else
    R5UartDriverImpl(void)
#endif
  {

  }

  void R5UartDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
      R5UartDriverComponentBase::init(instance);
  }

    void R5UartDriverComponentImpl ::
    initDriver(NATIVE_UINT_TYPE sendId, NATIVE_UINT_TYPE sendSize, NATIVE_UINT_TYPE receiveId, NATIVE_UINT_TYPE receiveSize, Fw::MemAllocator& allocator)
    {
        FW_ASSERT((0 < sendSize) || (0 < receiveSize));

        U8* transmitPtr = 0;
        U8* receivePtr = 0;

        if(0 < sendSize) {
            transmitPtr = static_cast<U8*>(allocator.allocate(sendId, sendSize));
            FW_ASSERT(0 != transmitPtr);
        }

        if(0 < receiveSize) {
            receivePtr = static_cast<U8*>(allocator.allocate(receiveId, receiveSize));
            FW_ASSERT(0 != receivePtr);
        }

        UartDrvInit(getInstance(), transmitPtr, sendSize, receivePtr, receiveSize);
    }


  R5UartDriverComponentImpl ::
    ~R5UartDriverComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5UartDriverComponentImpl ::
    serialRecv_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        Drv::SerialReadStatus &status
    )
  {
      serBuffer.setsize(UartDrvReceive(getInstance(), (U8*)(serBuffer.getdata()), serBuffer.getsize()));
      status = Drv::SER_OK;
  }

  void R5UartDriverComponentImpl ::
    serialSend_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
      UartDrvSend(getInstance(), (U8*)(serBuffer.getdata()), serBuffer.getsize());
  }

} // end namespace R5
