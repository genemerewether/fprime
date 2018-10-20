// ======================================================================
// \title  R5UartDriverImpl.hpp
// \author tcanham
// \brief  hpp file for R5UartDriver component implementation class
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

#ifndef R5UartDriver_HPP
#define R5UartDriver_HPP

#include "R5/UartDrv/R5UartDriverComponentAc.hpp"
#include <Fw/Types/MemAllocator.hpp>

namespace R5 {

  class R5UartDriverComponentImpl :
    public R5UartDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5UartDriver
      //!
      R5UartDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5UartDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      void initDriver(NATIVE_UINT_TYPE sendId, NATIVE_UINT_TYPE sendSize, NATIVE_UINT_TYPE receiveId, NATIVE_UINT_TYPE receiveSize, Fw::MemAllocator& allocator);

      //! Destroy object R5UartDriver
      //!
      ~R5UartDriverComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for serialRecv
      //!
      void serialRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

      //! Handler implementation for serialSend
      //!
      void serialSend_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer
      );


    };

} // end namespace R5

#endif
