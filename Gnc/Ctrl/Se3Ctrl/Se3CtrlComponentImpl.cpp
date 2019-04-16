// ======================================================================
// \title  Se3CtrlImpl.cpp
// \author mereweth
// \brief  cpp file for Se3Ctrl component implementation class
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


#include <Gnc/Ctrl/Se3Ctrl/Se3CtrlComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <stdio.h>

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#include <stdio.h>
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

#undef DEBUG_PRINT
#define DEBUG_PRINT(x,...)

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Se3CtrlComponentImpl ::
#if FW_OBJECT_NAMES == 1
    Se3CtrlComponentImpl(
        const char *const compName
    ) :
      Se3CtrlComponentBase(compName),
#else
    Se3CtrlComponentImpl(void) :
      Se3CtrlComponentImpl(void),
#endif
      seq(0u),
      u_tlm(),
      mass(0.0f),
      J_b(3,3),
      x_w(0.0f, 0.0f, 0.0f),
      x_w__des(0.0f, 0.0f, 0.0f),
      w_q_b(0.0f, 0.0f, 0.0f, 1.0f),
      w_q_b__des(0.0f, 0.0f, 0.0f, 1.0f),
      v_b(0.0f, 0.0f, 0.0f),
      v_w__des(0.0f, 0.0f, 0.0f),
      omega_b(0.0f, 0.0f, 0.0f),
      omega_b__des(0.0f, 0.0f, 0.0f),
      a_w__des(0.0f, 0.0f, 0.0f),
      alpha_b__des(0.0f, 0.0f, 0.0f),
      se3Control(),
      posCtrlMode(POSCTRL_DISABLED),
      attCtrlMode(ATTCTRL_DISABLED),
      paramsInited(false)
  {
      for (NATIVE_UINT_TYPE i = 0; i < FW_NUM_ARRAY_ELEMENTS(this->u_tlm); i++) {
          this->u_tlm[i] = 0.0f;
      }

      quest_gnc::WorldParams wParams = {9.80665f, 1.2f};
      (void) se3Control.SetWorldParams(wParams);
  }

  void Se3CtrlComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
#ifndef BUILD_TIR5
    printf("prm %d updated\n", id);
#endif
  }
  
  void Se3CtrlComponentImpl ::
    parametersLoaded()
  {
      this->paramsInited = false;
      Fw::ParamValid valid[12];
      int stat;

      quest_gnc::RigidBodyModel rigidBody = {paramGet_mass(valid[0]),
					     paramGet_I_xx(valid[1]),
					     paramGet_I_yy(valid[2]),
					     paramGet_I_zz(valid[3]),
					     paramGet_I_xy(valid[4]),
					     paramGet_I_xz(valid[5]),
					     paramGet_I_yz(valid[6])};
      
      quest_gnc::Se3Model se3Model = {rigidBody,
				      paramGet_force_z(valid[7])};

      for (U32 i = 0; i < 8; i++) {
          if (Fw::PARAM_VALID != valid[i]) {  return;  }
      }

      this->mass = rigidBody.mass;
      this->J_b << rigidBody.Ixx, rigidBody.Ixy, rigidBody.Ixz,
                   rigidBody.Ixy, rigidBody.Iyy, rigidBody.Iyz,
                   rigidBody.Ixz, rigidBody.Iyz, rigidBody.Izz;
      stat = se3Control.SetModel(se3Model);
      if (stat) {  return;  }

      stat = se3Control.SetGains(Eigen::Vector3d(paramGet_k_x__x(valid[0]),
                                                 paramGet_k_x__y(valid[1]),
                                                 paramGet_k_x__z(valid[2])),
                                 Eigen::Vector3d(paramGet_k_v__x(valid[3]),
                                                 paramGet_k_v__y(valid[4]),
                                                 paramGet_k_v__z(valid[5])),
                                 Eigen::Vector3d(paramGet_k_R__x(valid[6]),
                                                 paramGet_k_R__y(valid[7]),
                                                 paramGet_k_R__z(valid[8])),
                                 Eigen::Vector3d(paramGet_k_omega__x(valid[9]),
                                                 paramGet_k_omega__y(valid[10]),
                                                 paramGet_k_omega__z(valid[11])));
      if (stat) {  return;  }

      for (U32 i = 0; i < 12; i++) {
          if (Fw::PARAM_VALID != valid[i]) {  return;  }
      }

      stat = se3Control.SetSaturation(Eigen::Vector3d(paramGet_sat_x__x(valid[0]),
                                                      paramGet_sat_x__y(valid[1]),
                                                      paramGet_sat_x__z(valid[2])),
                                      Eigen::Vector3d(paramGet_sat_v__x(valid[3]),
                                                      paramGet_sat_v__y(valid[4]),
                                                      paramGet_sat_v__z(valid[5])),
                                      Eigen::Vector3d(paramGet_sat_R__x(valid[6]),
                                                      paramGet_sat_R__y(valid[7]),
                                                      paramGet_sat_R__z(valid[8])),
                                      Eigen::Vector3d(paramGet_sat_omega__x(valid[9]),
                                                      paramGet_sat_omega__y(valid[10]),
                                                      paramGet_sat_omega__z(valid[11])));
      if (stat) {  return;  }

      for (U32 i = 0; i < 12; i++) {
          if ((Fw::PARAM_VALID != valid[i]) &&
              (Fw::PARAM_DEFAULT != valid[i])) {
              return;
          }
      }

      this->paramsInited = true;
  }

  void Se3CtrlComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Se3CtrlComponentBase::init(instance);
  }

  Se3CtrlComponentImpl ::
    ~Se3CtrlComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void Se3CtrlComponentImpl ::
    SE3CTRL_SetPosCtrlMode_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        PosCtrlMode mode
    )
  {    
      if (!paramsInited &&
          mode != POSCTRL_DISABLED) {
          this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
      }
      else {
          this->posCtrlMode = mode;
          this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
      }

      if ((ATTCTRL_DISABLED == this->attCtrlMode) &&
	  (POSCTRL_DISABLED == mode))              {
	  using namespace ROS::geometry_msgs;
          // TODO(mereweth) - convert frame id
          ROS::std_msgs::Header h(this->seq, this->getTime(), 0/*"body"*/);
          WrenchStamped u_b__comm(h,
				  Wrench(Vector3(0.0, 0.0, 0.0),
					 Vector3(0.0, 0.0, 0.0)));
          if (this->isConnected_controls_OutputPort(0)) {
              this->controls_out(0, u_b__comm);
          }
      }
  }
 
  void Se3CtrlComponentImpl ::
    SE3CTRL_SetAttCtrlMode_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        AttCtrlMode mode
    )
  {    
      if (!paramsInited &&
          mode != ATTCTRL_DISABLED) {
          this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
      }
      else {
          this->attCtrlMode = mode;
          this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
      }

      if ((ATTCTRL_DISABLED == mode)              &&
	  (POSCTRL_DISABLED == this->posCtrlMode)) {
          using namespace ROS::geometry_msgs;
          // TODO(mereweth) - convert frame id
          ROS::std_msgs::Header h(this->seq, this->getTime(), 0/*"body"*/);
          WrenchStamped u_b__comm(h,
				  Wrench(Vector3(0.0, 0.0, 0.0),
					 Vector3(0.0, 0.0, 0.0)));
          if (this->isConnected_controls_OutputPort(0)) {
              this->controls_out(0, u_b__comm);
          }
      }
  }

  void Se3CtrlComponentImpl ::
    SE3CTRL_LinearSetpoint_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F64 x_w__x,
        F64 x_w__y,
        F64 x_w__z,
        F64 v_w__x,
        F64 v_w__y,
        F64 v_w__z
    )
  {
      this->x_w__des.set(x_w__x, x_w__y, x_w__z);
      this->v_w__des.set(v_w__x, v_w__y, v_w__z);
      this->a_w__des.set(0.0, 0.0, 0.0);

      this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

  void Se3CtrlComponentImpl ::
    SE3CTRL_AngularSetpoint_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F64 w_q_b__x,
        F64 w_q_b__y,
        F64 w_q_b__z,
        F64 w_q_b__w,
        F64 omega_b__x,
        F64 omega_b__y,
        F64 omega_b__z
    )
  {
      this->w_q_b__des.setx(w_q_b__x);
      this->w_q_b__des.sety(w_q_b__y);
      this->w_q_b__des.setz(w_q_b__z);
      this->w_q_b__des.setw(w_q_b__w);
      this->omega_b__des.set(omega_b__x,
			     omega_b__y,
			     omega_b__z);
      this->alpha_b__des.set(0.0, 0.0, 0.0);

      this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

  void Se3CtrlComponentImpl ::
    SE3CTRL_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
      this->parametersLoaded();
      if (this->paramsInited) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
      else {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
      }
  }
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Se3CtrlComponentImpl ::
    odometry_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::nav_msgs::Odometry &Odometry
    )
  {
      ROS::std_msgs::Header h = Odometry.getheader();
      this->seq = h.getseq();
      // TODO(mereweth) check h.getstamp()

      this->x_w = Odometry.getpose().getpose().getposition();
      this->w_q_b = Odometry.getpose().getpose().getorientation();

      this->v_b = Odometry.gettwist().gettwist().getlinear();
      this->omega_b = Odometry.gettwist().gettwist().getangular();
  }

  void Se3CtrlComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {

      if (context == SE3CTRL_SCHED_CONTEXT_CTRL) {
          using ROS::geometry_msgs::Vector3;

          // set desired position velocity acceleration
          if (LINVEL == this->posCtrlMode) {
              this->se3Control.SetVelocityDes(Eigen::Vector3d(this->v_w__des.getx(),
                                                              this->v_w__des.gety(),
                                                              this->v_w__des.getz()),
                                              Eigen::Vector3d(this->a_w__des.getx(),
                                                              this->a_w__des.gety(),
                                                              this->a_w__des.getz()));
          }
	  else if (POS_LINVEL == this->posCtrlMode) {
              this->se3Control.SetPositionDes(Eigen::Vector3d(this->x_w__des.getx(),
                                                              this->x_w__des.gety(),
                                                              this->x_w__des.getz()),
                                              Eigen::Vector3d(this->v_w__des.getx(),
                                                              this->v_w__des.gety(),
                                                              this->v_w__des.getz()),
                                              Eigen::Vector3d(this->a_w__des.getx(),
                                                              this->a_w__des.gety(),
                                                              this->a_w__des.getz()));
          }

          // set position feedback
          this->se3Control.SetPositionLinVel(Eigen::Vector3d(this->x_w.getx(),
                                                             this->x_w.gety(),
                                                             this->x_w.getz()),
                                             Eigen::Vector3d(this->v_b.getx(),
                                                             this->v_b.gety(),
                                                             this->v_b.getz()));

          Eigen::Vector3d a_w__comm(0, 0, 0);
          if (LINVEL == this->posCtrlMode) {
              // don't use position
              this->se3Control.GetAccelCommand(&a_w__comm, true);
          }
	  else if (POS_LINVEL == this->posCtrlMode) {
              this->se3Control.GetAccelCommand(&a_w__comm, false);
          }
	  
          using ROS::geometry_msgs::Vector3;

          Eigen::Vector3d force_b__comm;

          if ((ATT_ATTRATE == this->attCtrlMode) ||
              (RPRATE_YAW == this->attCtrlMode)  ||
              (ATTRATE == this->attCtrlMode)     ||
              (YAW_ONLY == this->attCtrlMode))    {

              bool rpVelOnly = false;
              bool yawVelOnly = false;
	      bool doSaturation = true;

              if (RPRATE_YAW == this->attCtrlMode) {
                  rpVelOnly = true;
              }
              
              if (ATTRATE == this->attCtrlMode) {
                  rpVelOnly = true;
                  yawVelOnly = true;
              }

	      if (YAW_ONLY == this->attCtrlMode) {
		  doSaturation = false;
	      }
            
              Eigen::Quaterniond _w_q_b__des = Eigen::Quaterniond(this->w_q_b__des.getw(),
                                                                  this->w_q_b__des.getx(),
                                                                  this->w_q_b__des.gety(),
                                                                  this->w_q_b__des.getz());
              this->se3Control.SetAttitudeDes(_w_q_b__des,
                                              Eigen::Vector3d(
                                                  this->omega_b__des.getx(),
                                                  this->omega_b__des.gety(),
                                                  this->omega_b__des.getz()),
                                              Eigen::Vector3d(
                                                  this->omega_b__des.getx(),
                                                  this->omega_b__des.gety(),
                                                  this->omega_b__des.getz()),
                                              rpVelOnly, yawVelOnly,
					      doSaturation);
          }

          // set angular feedback
          Eigen::Quaterniond _w_q_b = Eigen::Quaterniond(this->w_q_b.getw(),
                                                        this->w_q_b.getx(),
                                                        this->w_q_b.gety(),
                                                        this->w_q_b.getz());
          this->se3Control.SetAttitudeAngVel(_w_q_b,
                                             Eigen::Vector3d(
                                                 this->omega_b.getx(),
                                                 this->omega_b.gety(),
                                                 this->omega_b.getz()));

	  force_b__comm = this->mass * (_w_q_b.inverse() * a_w__comm);

          Eigen::Vector3d alpha_b__comm(0, 0, 0);
          if (RPRATE_YAW == this->attCtrlMode) {
              this->se3Control.GetAngAccelCommand(&alpha_b__comm, true, false);
          }
          else if (ATTRATE == this->attCtrlMode) {
              this->se3Control.GetAngAccelCommand(&alpha_b__comm, true, true);
          }
          else if (YAW_ONLY == this->attCtrlMode) {
	      this->se3Control.GetAngAxisAlignedCommand(&alpha_b__comm, 1<<2);
          }
          else if (ATT_ATTRATE == this->attCtrlMode) {
              this->se3Control.GetAngAccelCommand(&alpha_b__comm);
          }

          Eigen::Vector3d moment_b__comm = this->J_b * alpha_b__comm;

	  // NOTE(mereweth) - double-checking that zero if disabled
	  if (POSCTRL_DISABLED == this->posCtrlMode) {
	      force_b__comm = Eigen::Vector3d(0.0, 0.0, 0.0);
	  }
	  if (ATTCTRL_DISABLED == this->attCtrlMode) {
	      moment_b__comm = Eigen::Vector3d(0.0, 0.0, 0.0);
	  }
	  
          // TODO(mereweth) - convert frame id
          ROS::std_msgs::Header h(this->seq, this->getTime(), 0/*"body"*/);
          ROS::geometry_msgs::WrenchStamped u_b__comm(h,
		 ROS::geometry_msgs::Wrench(
					    Vector3(force_b__comm(0), force_b__comm(1), force_b__comm(2)),
					    Vector3(moment_b__comm(0), moment_b__comm(1), moment_b__comm(2))));
          if (this->isConnected_controls_OutputPort(0) &&
              ((ATTCTRL_DISABLED != this->attCtrlMode) ||
	       (POSCTRL_DISABLED != this->posCtrlMode))) {
              this->controls_out(0, u_b__comm);
          }

          ROS::geometry_msgs::AccelStamped accel__comm(h,
            ROS::geometry_msgs::Accel(Vector3(a_w__comm(0), a_w__comm(1), a_w__comm(2)),
                                      Vector3(alpha_b__comm(0), alpha_b__comm(1), alpha_b__comm(2))));
          if (this->isConnected_accelCommand_OutputPort(0) &&
              ((ATTCTRL_DISABLED != this->attCtrlMode) ||
	       (POSCTRL_DISABLED != this->posCtrlMode))) {
              this->accelCommand_out(0, accel__comm);
          }

          this->u_tlm[0] = force_b__comm(0);
          this->u_tlm[1] = force_b__comm(1);
          this->u_tlm[2] = force_b__comm(2);
          this->u_tlm[3] = moment_b__comm(0);
          this->u_tlm[4] = moment_b__comm(1);
          this->u_tlm[5] = moment_b__comm(2);
      }
      else if (context == SE3CTRL_SCHED_CONTEXT_TLM) {
          COMPILE_TIME_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->u_tlm) == 6,
                              SE3CTRL_WRENCH_TLM_SIZE);
          this->tlmWrite_SE3CTRL_XThrustComm(this->u_tlm[0]);
          this->tlmWrite_SE3CTRL_YThrustComm(this->u_tlm[1]);
          this->tlmWrite_SE3CTRL_ZThrustComm(this->u_tlm[2]);
	  
          this->tlmWrite_SE3CTRL_MomentCommX(this->u_tlm[3]);
          this->tlmWrite_SE3CTRL_MomentCommY(this->u_tlm[4]);
          this->tlmWrite_SE3CTRL_MomentCommZ(this->u_tlm[5]);

          this->tlmWrite_SE3CTRL_w_q_b__des(this->w_q_b__des);
          this->tlmWrite_SE3CTRL_w_q_b(this->w_q_b);

	  this->tlmWrite_SE3CTRL_Error_x_w(this->x_w__des.getx()
					   - this->x_w.getx());
	  this->tlmWrite_SE3CTRL_Error_y_w(this->x_w__des.gety()
					   - this->x_w.gety());
	  this->tlmWrite_SE3CTRL_Error_z_w(this->x_w__des.getz()
					   - this->x_w.getz());
      }
      else {
          // TODO(mereweth) - assert invalid port
      }
  }

} // end namespace Gnc
