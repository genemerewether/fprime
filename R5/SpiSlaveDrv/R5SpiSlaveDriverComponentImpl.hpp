// ====================================================================== 
// \title  R5SpiSlaveDriverImpl.hpp
// \author tcanham
// \brief  hpp file for R5SpiSlaveDriver component implementation class
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

#ifndef R5SpiSlaveDriver_HPP
#define R5SpiSlaveDriver_HPP

#include "R5/SpiSlaveDrv/R5SpiSlaveDriverComponentAc.hpp"
#include <Fw/Types/MemAllocator.hpp>

namespace R5 {

  class R5SpiSlaveDriverComponentImpl :
    public R5SpiSlaveDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5SpiSlaveDriver
      //!
      R5SpiSlaveDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5SpiSlaveDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      void initDriver(NATIVE_UINT_TYPE instance, NATIVE_UINT_TYPE id, NATIVE_UINT_TYPE size, Fw::MemAllocator& allocator, U16 syncByte, U32 packetSize);

      //! Destroy object R5SpiSlaveDriver
      //!
      ~R5SpiSlaveDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for spiRecv
      //!
      void spiRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &buff 
      );


    };

} // end namespace R5

#endif
