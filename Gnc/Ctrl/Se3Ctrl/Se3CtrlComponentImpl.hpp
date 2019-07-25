// ======================================================================
// \title  Se3CtrlImpl.hpp
// \author mereweth
// \brief  hpp file for Se3Ctrl component implementation class
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

#ifndef Se3Ctrl_HPP
#define Se3Ctrl_HPP

#include "Gnc/Ctrl/Se3Ctrl/Se3CtrlComponentAc.hpp"
#include <Gnc/Ctrl/Se3Ctrl/Se3CtrlComponentImplCfg.hpp>

#include "quest_gnc/ctrl/se3_control.h"
#include "quest_gnc/utils/rigidbody_model.h"
#include "quest_gnc/utils/multirotor_model.h"
#include "quest_gnc/utils/world_params.h"

namespace Gnc {

  class Se3CtrlComponentImpl :
    public Se3CtrlComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object Se3Ctrl
      //!
      Se3CtrlComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object Se3Ctrl
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object Se3Ctrl
      //!
      ~Se3CtrlComponentImpl(void);

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

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

      //! Handler implementation for odometry
      //!
      void odometry_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::nav_msgs::OdometryAccel &Odometry
      );

      //! Handler implementation for se3Cmd
      //!
      void se3Cmd_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::Se3FeedForward &Se3FeedForward 
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

      //! Implementation for SE3CTRL_SetPosCtrlMode command handler
      //! Set controller mode
      void SE3CTRL_SetPosCtrlMode_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          PosCtrlMode mode 
      );

      //! Implementation for SE3CTRL_SetAttCtrlMode command handler
      //! Set controller mode
      void SE3CTRL_SetAttCtrlMode_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          AttCtrlMode mode 
      );
    
      //! Implementation for SE3CTRL_LinearSetpoint command handler
      //! 
      void SE3CTRL_LinearSetpoint_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          F64 x_w__x, 
          F64 x_w__y, 
          F64 x_w__z, 
          F64 v_w__x, 
          F64 v_w__y, 
          F64 v_w__z
      );

      //! Implementation for SE3CTRL_AngularSetpoint command handler
      //! 
      void SE3CTRL_AngularSetpoint_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          F64 w_q_b__x, 
          F64 w_q_b__y, 
          F64 w_q_b__z,
          F64 w_q_b__w,
          F64 omega_b__x, 
          F64 omega_b__y, 
          F64 omega_b__z
      );
    
      //! Implementation for SE3CTRL_InitParams command handler
      //! 
      void SE3CTRL_InitParams_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      // ----------------------------------------------------------------------
      // Private member variables
      // ----------------------------------------------------------------------

      U32 seq;

      F32 u_tlm[6];

      double mass;

      Eigen::Matrix3d J_b;

      ROS::geometry_msgs::Point x_w;

      ROS::geometry_msgs::Point x_w__des;

      ROS::geometry_msgs::Quaternion w_q_b;

      ROS::geometry_msgs::Quaternion w_q_b__des;

      ROS::geometry_msgs::Vector3 v_b;

      ROS::geometry_msgs::Vector3 v_w__des;

      ROS::geometry_msgs::Vector3 omega_b;

      ROS::geometry_msgs::Vector3 omega_b__des;

      ROS::geometry_msgs::Vector3 a_w__des;

      ROS::geometry_msgs::Vector3 alpha_b__des;

      quest_gnc::Se3Control se3Control;

      PosCtrlMode posCtrlMode;
    
      AttCtrlMode attCtrlMode;

      bool paramsInited;
    };

} // end namespace Gnc

#endif
