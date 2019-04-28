// ====================================================================== 
// \title  FixedAxisSe3AdapterImpl.hpp
// \author gene
// \brief  hpp file for FixedAxisSe3Adapter component implementation class
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

#ifndef FixedAxisSe3Adapter_HPP
#define FixedAxisSe3Adapter_HPP

#include "Gnc/Utils/FixedAxisSe3Adapter/FixedAxisSe3AdapterComponentAc.hpp"

namespace Gnc {

  class FixedAxisSe3AdapterComponentImpl :
    public FixedAxisSe3AdapterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object FixedAxisSe3Adapter
      //!
      FixedAxisSe3AdapterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object FixedAxisSe3Adapter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object FixedAxisSe3Adapter
      //!
      ~FixedAxisSe3AdapterComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for flatOutput
      //!
      void flatOutput_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations 
      // ----------------------------------------------------------------------

      //! Implementation for AXSE3ADAP_InitParams command handler
      //! 
      void AXSE3ADAP_InitParams_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );


    };

} // end namespace Gnc

#endif
