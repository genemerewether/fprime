// ======================================================================
// \title  AttFilterImpl.hpp
// \author mereweth
// \brief  hpp file for AttFilter component implementation class
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

#ifndef AttFilter_HPP
#define AttFilter_HPP

#include "Gnc/Est/AttFilter/AttFilterComponentAc.hpp"
#include "Gnc/Est/AttFilter/AttFilterComponentImplCfg.hpp"

#include "quest_gnc/est/att_filter.h"
#include "quest_gnc/utils/world_params.h"

namespace Gnc {

  class AttFilterComponentImpl :
    public AttFilterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object AttFilter
      //!
      AttFilterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object AttFilter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object AttFilter
      //!
      ~AttFilterComponentImpl(void);

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PRIVATE:
      void parameterUpdated(FwPrmIdType id /*!< The parameter ID*/);
    
      void parametersLoaded();

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------
    
      //! Handler implementation for Imu
      //!
      void Imu_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &ImuNoCov
      );

      //! Handler implementation for ImuStateUpdate
      //!
      void ImuStateUpdate_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::ImuStateUpdateNoCov &ImuStateUpdate
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations 
      // ----------------------------------------------------------------------

      //! Implementation for ATTFILTER_InitParams command handler
      //! 
      void ATTFILTER_InitParams_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      //! Implementation for ATTFILTER_Reinit command handler
      //! 
      void ATTFILTER_Reinit_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      U32 seq;

      quest_gnc::estimation::AttFilter attFilter;

      bool paramsInited;

    };

} // end namespace Gnc

#endif
