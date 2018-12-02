// ======================================================================
// \title  LeeCtrlImpl.cpp
// \author mereweth
// \brief  cpp file for LeeCtrl component implementation class
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


#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  LeeCtrlComponentImpl ::
#if FW_OBJECT_NAMES == 1
    LeeCtrlComponentImpl(
        const char *const compName
    ) :
      LeeCtrlComponentBase(compName),
#else
    LeeCtrlComponentImpl(void) :
      LeeCtrlComponentImpl(void),
#endif
      seq(0u),
      u_tlm(),
      thrust_x_tlm(0.0f),
      thrust_y_tlm(0.0f),
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
      a_w__comm(0.0f, 0.0f, 0.0f),
      a_w__des(0.0f, 0.0f, 0.0f),
      thrust_b__des(0.0f, 0.0f, 0.0f),
      yaw__des(0.0f),
      leeControl(),
      ctrlMode(CTRL_DISABLED)
  {
      for (NATIVE_UINT_TYPE i = 0; i < FW_NUM_ARRAY_ELEMENTS(this->u_tlm); i++) {
          this->u_tlm[i] = 0.0f;
      }

      quest_gnc::WorldParams wParams = {9.80665f, 1.2f};
      (void) leeControl.SetWorldParams(wParams);
  }

  void LeeCtrlComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
  }
  
  void LeeCtrlComponentImpl ::
    parametersLoaded()
  {
      Fw::ParamValid valid[12];

      quest_gnc::multirotor::MultirotorModel mrModel = {paramGet_mass(valid[0]),
                                                        paramGet_I_xx(valid[1]),
                                                        paramGet_I_yy(valid[2]),
                                                        paramGet_I_zz(valid[3]),
                                                        paramGet_I_xy(valid[4]),
                                                        paramGet_I_xz(valid[5]),
                                                        paramGet_I_yz(valid[6])};

      for (U32 i = 0; i < 7; i++) {
          if (Fw::PARAM_VALID != valid[i]) {  return;  }
      }

      this->mass = mrModel.mass;
      this->J_b << mrModel.Ixx, mrModel.Ixy, mrModel.Ixz,
                   mrModel.Ixy, mrModel.Iyy, mrModel.Iyz,
                   mrModel.Ixz, mrModel.Iyz, mrModel.Izz;
      (void) leeControl.SetModel(mrModel);

      (void) leeControl.SetGains(Eigen::Vector3d(paramGet_k_x__x(valid[0]),
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

      for (U32 i = 0; i < 12; i++) {
          if (Fw::PARAM_VALID != valid[i]) {  return;  }
      }
      // TODO(mereweth) - mark params as initialized
  }

  void LeeCtrlComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    LeeCtrlComponentBase::init(instance);
  }

  LeeCtrlComponentImpl ::
    ~LeeCtrlComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void LeeCtrlComponentImpl ::
    LCTRL_SetCtrlMode_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        CtrlMode mode
    )
  {
      // TODO(Mereweth) - bounds on discontinuity in commanded thrust/moment
      this->ctrlMode = mode;

      this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }


  void LeeCtrlComponentImpl ::
    LCTRL_FlatOutputSetpoint_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F64 x_w__x,
        F64 x_w__y,
        F64 x_w__z,
        F64 yaw
    )
  {
      this->x_w__des.set(x_w__x, x_w__y, x_w__z);
      this->v_w__des.set(0.0, 0.0, 0.0);
      this->a_w__des.set(0.0, 0.0, 0.0);
      this->yaw__des = yaw;

      this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

  void LeeCtrlComponentImpl ::
    LCTRL_AttThrustSetpoint_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F64 thrust,
        F64 w_q_b__x,
        F64 w_q_b__y,
        F64 w_q_b__z,
        F64 w_q_b__w
    )
  {
      this->w_q_b__des.setx(w_q_b__x);
      this->w_q_b__des.sety(w_q_b__y);
      this->w_q_b__des.setz(w_q_b__z);
      this->w_q_b__des.setw(w_q_b__w);
      this->omega_b__des.set(0.0, 0.0, 0.0);
      this->thrust_b__des.set(0.0, 0.0, thrust);

      this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void LeeCtrlComponentImpl ::
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

  void LeeCtrlComponentImpl ::
    flatOutput_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
      this->x_w__des = FlatOutput.getposition();
      this->v_w__des = FlatOutput.getvelocity();
      this->a_w__des = FlatOutput.getacceleration();
      this->yaw__des = FlatOutput.getyaw();
  }

  void LeeCtrlComponentImpl ::
    attRateThrust_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::AttitudeRateThrust &AttitudeRateThrust
    )
  {
      this->w_q_b__des = AttitudeRateThrust.getattitude();
      this->omega_b__des = AttitudeRateThrust.getangular_rates();
      this->thrust_b__des = AttitudeRateThrust.getthrust();
  }

  void LeeCtrlComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {

      if (context == LCTRL_SCHED_CONTEXT_POS) {
          using ROS::geometry_msgs::Vector3;

          if (!((FLAT_OUTPUT_LIN_ACC_FF == this->ctrlMode) ||
                (FLAT_OUTPUT_ANG_ACC_FF == this->ctrlMode))) {
              return;
          }

          // set desired position velocity acceleration
          this->leeControl.SetPositionDes(Eigen::Vector3d(this->x_w__des.getx(),
                                                          this->x_w__des.gety(),
                                                          this->x_w__des.getz()),
                                          Eigen::Vector3d(this->v_w__des.getx(),
                                                          this->v_w__des.gety(),
                                                          this->v_w__des.getz()),
                                          Eigen::Vector3d(this->a_w__des.getx(),
                                                          this->a_w__des.gety(),
                                                          this->a_w__des.getz()));

          this->leeControl.SetYawDes(this->yaw__des);

          // set position feedback
          this->leeControl.SetPositionLinVel(Eigen::Vector3d(this->x_w.getx(),
                                                             this->x_w.gety(),
                                                             this->x_w.getz()),
                                             Eigen::Vector3d(this->v_b.getx(),
                                                             this->v_b.gety(),
                                                             this->v_b.getz()));

          Eigen::Vector3d a_w__comm(0, 0, 0);
          this->leeControl.GetAccelCommand(&a_w__comm);
          this->a_w__comm = Vector3(a_w__comm(0), a_w__comm(1), a_w__comm(2));
      }
      else if (context == LCTRL_SCHED_CONTEXT_ATT) {
          using ROS::geometry_msgs::Vector3;

          if (!((FLAT_OUTPUT_LIN_ACC_FF == this->ctrlMode) ||
                (FLAT_OUTPUT_ANG_ACC_FF == this->ctrlMode) ||
                (ATT_RATE_THRUST        == this->ctrlMode))) {
              return;
          }
          
          Eigen::Vector3d thrust_b__comm;

          if (ATT_RATE_THRUST == this->ctrlMode) {
              Eigen::Quaterniond _w_q_b__des = Eigen::Quaterniond(this->w_q_b__des.getw(),
                                                                  this->w_q_b__des.getx(),
                                                                  this->w_q_b__des.gety(),
                                                                  this->w_q_b__des.getz());
              this->leeControl.SetAttitudeDes(_w_q_b__des,
                                              Eigen::Vector3d(
                                                  this->omega_b__des.getx(),
                                                  this->omega_b__des.gety(),
                                                  this->omega_b__des.getz()));
              thrust_b__comm = Eigen::Vector3d(this->thrust_b__des.getx(),
                                               this->thrust_b__des.gety(),
                                               this->thrust_b__des.getz());
          }

          // set angular feedback
          Eigen::Quaterniond _w_q_b = Eigen::Quaterniond(this->w_q_b.getw(),
                                                        this->w_q_b.getx(),
                                                        this->w_q_b.gety(),
                                                        this->w_q_b.getz());
          this->leeControl.SetAttitudeAngVel(_w_q_b,
                                             Eigen::Vector3d(
                                                 this->omega_b.getx(),
                                                 this->omega_b.gety(),
                                                 this->omega_b.getz()));

          if ((FLAT_OUTPUT_LIN_ACC_FF == this->ctrlMode) ||
              (FLAT_OUTPUT_ANG_ACC_FF == this->ctrlMode)) {
              Eigen::Vector3d _a_w__comm(this->a_w__comm.getx(),
                                         this->a_w__comm.gety(),
                                         this->a_w__comm.getz());
              thrust_b__comm = this->mass * (_w_q_b.inverse() * _a_w__comm);
          }

          Eigen::Vector3d alpha_b__comm(0, 0, 0);
          this->leeControl.GetAngAccelCommand(&alpha_b__comm);

          Eigen::Vector3d moment_b__comm = this->J_b * alpha_b__comm;

          // TODO(mereweth) - convert frame id
          ROS::std_msgs::Header h(this->seq, this->getTime(), 0/*"body"*/);
          ROS::mav_msgs::TorqueThrust u_b__comm(h,
            Vector3(moment_b__comm(0), moment_b__comm(1), moment_b__comm(2)),
            Vector3(thrust_b__comm(0), thrust_b__comm(1), thrust_b__comm(2)));
          if (this->isConnected_controls_OutputPort(0) &&
              (CTRL_DISABLED != this->ctrlMode)) {
              this->controls_out(0, u_b__comm);
          }

          this->thrust_x_tlm = thrust_b__comm(0);
          this->thrust_y_tlm = thrust_b__comm(1);

          this->u_tlm[0] = thrust_b__comm(2); // z-axis only
          this->u_tlm[1] = moment_b__comm(0);
          this->u_tlm[2] = moment_b__comm(1);
          this->u_tlm[3] = moment_b__comm(2);
      }
      else if (context == LCTRL_SCHED_CONTEXT_TLM) {
          COMPILE_TIME_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->u_tlm) == 4,
                              LCTRL_THRUST_MOMENT_TLM_SIZE);
          this->tlmWrite_LCTRL_ThrustComm(this->u_tlm[0]);
          this->tlmWrite_LCTRL_MomentCommX(this->u_tlm[1]);
          this->tlmWrite_LCTRL_MomentCommY(this->u_tlm[2]);
          this->tlmWrite_LCTRL_MomentCommZ(this->u_tlm[3]);

          this->tlmWrite_LCTRL_XThrustComm(this->thrust_x_tlm);
          this->tlmWrite_LCTRL_YThrustComm(this->thrust_y_tlm);

          this->tlmWrite_LCTRL_w_q_b__des(this->w_q_b__des);
          this->tlmWrite_LCTRL_w_q_b(this->w_q_b);

	  this->tlmWrite_LCTRL_Error_x_w(this->x_w__des.getx()
					 - this->x_w.getx());
	  this->tlmWrite_LCTRL_Error_y_w(this->x_w__des.gety()
					 - this->x_w.gety());
	  this->tlmWrite_LCTRL_Error_z_w(this->x_w__des.getz()
					 - this->x_w.getz());
      }
      else {
          // TODO(mereweth) - assert invalid port
      }
  }

} // end namespace Gnc
