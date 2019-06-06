// ====================================================================== 
// \title  FatalHandlerImpl.hpp
// \author tcanham
// \brief  hpp file for FatalHandler component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
// 
// ====================================================================== 

#ifndef FatalHandler_HPP
#define FatalHandler_HPP

#include "Svc/FatalHandler/FatalHandlerComponentAc.hpp"

namespace Svc {

  class FatalHandlerComponentImpl :
    public FatalHandlerComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object FatalHandler
      //!
      FatalHandlerComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object FatalHandler
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object FatalHandler
      //!
      ~FatalHandlerComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for FatalReceive
      //!
      void FatalReceive_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType Id /*!< The ID of the FATAL event*/
      );

      //! Implementation for FH_ENABLE_ASSERT command handler
      //! Enable/Disable asserts
      void FH_ENABLE_ASSERT_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          AssertEnable enable /*!< Enable/Disable asserts*/
      );

      bool m_disableAssert; //!< flag to assert or not.


    };

} // end namespace Svc

#endif
