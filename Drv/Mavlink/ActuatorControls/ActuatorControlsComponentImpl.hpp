// ======================================================================
// \title  ActuatorControlsComponentImpl.hpp
// \author mereweth
// \brief  hpp file for ActuatorControls component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef ActuatorControls_HPP
#define ActuatorControls_HPP

#include "Drv/Mavlink/ActuatorControls/ActuatorControlsComponentAc.hpp"

namespace Drv {

  class ActuatorControlsComponentImpl :
    public ActuatorControlsComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ActuatorControls
      //!
      ActuatorControlsComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ActuatorControls
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ActuatorControls
      //!
      ~ActuatorControlsComponentImpl(void);

    PRIVATE:
      void parameterUpdated(FwPrmIdType id /*!< The parameter ID*/);
    
      void parametersLoaded();

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for prmTrigger
      //!
      void prmTrigger_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType dummy 
      );
    
      //! Handler implementation for rateThrust
      //!
      void rateThrust_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::RateThrust &RateThrust 
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for SerReadPort
      //!
      void SerReadPort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          SerialReadStatus &status /*!< Status of read*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for ACTCTRL_InitParams command handler
      //! 
      void ACTCTRL_InitParams_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      F32 minThrust;
      F32 maxThrust;
    
      bool paramsInited;
    
      Fw::Buffer m_outputBufObj;
      char m_outputBuf[512];
    };

} // end namespace Drv

#endif
