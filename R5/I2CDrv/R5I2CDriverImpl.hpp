// ====================================================================== 
// \title  R5I2CDriverImpl.hpp
// \author kubiak
// \brief  hpp file for R5I2CDriver component implementation class
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

#ifndef R5I2CDriver_HPP
#define R5I2CDriver_HPP

#include "R5/I2CDrv/R5I2CDriverComponentAc.hpp"
#include <Fw/Types/MemAllocator.hpp>

namespace R5 {

  class R5I2CDriverComponentImpl :
    public R5I2CDriverComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5I2CDriver
      //!
      R5I2CDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5I2CDriver
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object R5I2CDriver
      //!
      ~R5I2CDriverComponentImpl(void);

      void initDriver(NATIVE_UINT_TYPE rxSize,
                      NATIVE_UINT_TYPE txSize,
                      Fw::MemAllocator& allocator);
    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for config
      //!
      void config_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 busSpeed, 
          U32 slaveAddr, 
          U32 timeout 
      );

      //! Handler implementation for writeRead
      //!
      void writeRead_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &writeBuffer, 
          Fw::Buffer &readBuffer 
      );

      //! Handler implementation for status
      //!
      Drv::I2CStatus status_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool shouldBlock
      );

      //! Handler for input port service
      //
      void service_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      NATIVE_INT_TYPE m_instance;

    };

} // end namespace R5

#endif
