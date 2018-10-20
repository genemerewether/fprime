// ======================================================================
// \title  R5SpiMasterDriverImpl.cpp
// \author tcanham
// \brief  cpp file for R5SpiMasterDriver component implementation class
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


#include <R5/SpiMasterDrv/R5SpiMasterDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/MemAllocator.hpp>

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  R5SpiMasterDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5SpiMasterDriverComponentImpl(
        const char *const compName
    ) :
      R5SpiMasterDriverComponentBase(compName)
#else
    R5SpiMasterDriverImpl(void)
#endif
  {

  }

  void R5SpiMasterDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    R5SpiMasterDriverComponentBase::init(instance);
  }

  void R5SpiMasterDriverComponentImpl ::
    initDriver(NATIVE_UINT_TYPE instance,
          NATIVE_UINT_TYPE sendId, NATIVE_UINT_TYPE sendSize,
          NATIVE_UINT_TYPE recvId, NATIVE_UINT_TYPE recvSize,
          Fw::MemAllocator& allocator)
  {
  }




  R5SpiMasterDriverComponentImpl ::
    ~R5SpiMasterDriverComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5SpiMasterDriverComponentImpl ::
    spiSend_handler(
            NATIVE_INT_TYPE portNum, /*!< The port number*/
            Fw::Buffer *buff,
            U32 numBuffs
    )
  {
  }

  void R5SpiMasterDriverComponentImpl ::
    spiRecv_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &buff
    )
  {
  }

} // end namespace R5
