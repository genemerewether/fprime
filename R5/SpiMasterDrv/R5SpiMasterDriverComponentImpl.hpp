// ======================================================================
// \title  R5SpiMasterDriverImpl.hpp
// \author tcanham
// \brief  hpp file for R5SpiMasterDriver component implementation class
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

#ifndef R5SpiMasterDriver_HPP
#define R5SpiMasterDriver_HPP

#include "R5/SpiMasterDrv/R5SpiMasterDriverComponentAc.hpp"
#include <Fw/Types/MemAllocator.hpp>

namespace R5 {

  class R5SpiMasterDriverComponentImpl :
    public R5SpiMasterDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5SpiMasterDriver
      //!
      R5SpiMasterDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5SpiMasterDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! The sendSize is a number of bytes in integral number of U32s (should be evenly divisible by sizeof(U32)).
      //! The recvSize is a number of bytes in integral number of U16s (should be evenly divisible by sizeof(U16)).
      void initDriver(NATIVE_UINT_TYPE instance,
              NATIVE_UINT_TYPE sendId, NATIVE_UINT_TYPE sendSize,
              NATIVE_UINT_TYPE recvId, NATIVE_UINT_TYPE recvSize,
              Fw::MemAllocator& allocator);


      //! Destroy object R5SpiMasterDriver
      //!
      ~R5SpiMasterDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for spiSend
      //!
      void spiSend_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer *buff,
          U32 numBuffs
      );

      //! Handler implementation for spiRecv
      //!
      void spiRecv_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &buff
      );


    };

} // end namespace R5

#endif
