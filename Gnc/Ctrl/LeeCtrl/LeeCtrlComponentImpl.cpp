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
      x_w(0.0f, 0.0f, 0.0f),
      w_q_b(0.0f, 0.0f, 0.0f, 1.0f),
      v_b(0.0f, 0.0f, 0.0f),
      omega_b(0.0f, 0.0f, 0.0f),
      a_w__comm(0.0f, 0.0f, 0.0f),
      leeControl()
  {
      for (NATIVE_UINT_TYPE i = 0; i < FW_NUM_ARRAY_ELEMENTS(this->u_tlm); i++) {
          this->u_tlm[i] = 0.0f;
      }

      (void) leeControl.SetGains(Eigen::Vector3d(1.0f, 1.0f, 1.0f),
                                 Eigen::Vector3d(1.0f, 1.0f, 1.0f),
                                 Eigen::Vector3d(1.0f, 1.0f, 1.0f),
                                 Eigen::Vector3d(1.0f, 1.0f, 1.0f));

      quest_gnc::multirotor::MultirotorModel mrModel = {1.0f,
                                                        1.0f, 1.0f, 1.0f,
                                                        0.0f, 0.0f, 0.0f};
      quest_gnc::WorldParams wParams = {9.80665f, 1.2f};
      (void) leeControl.SetWorldParams(wParams);
      (void) leeControl.SetModel(mrModel);

      //TODO(mereweth) - remove this
      (void) leeControl.SetPositionDes(Eigen::Vector3d(0.0f, 0.0f, 1.5f),
                                       Eigen::Vector3d(0.0f, 0.0f, 0.0f),
                                       Eigen::Vector3d(0.0f, 0.0f, 0.0f));
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
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void LeeCtrlComponentImpl ::
    odometry_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::nav_msgs::Odometry &Odometry
    )
  {
      using namespace ROS::geometry_msgs;
      ROS::std_msgs::Header h = Odometry.getheader();
      this->seq = h.getseq();
      // TODO(mereweth) check h.getstamp()

      this->x_w = Odometry.getpose().getpose().getposition();
      this->w_q_b = Odometry.getpose().getpose().getorientation();

      this->v_b = Odometry.gettwist().gettwist().getlinear();
      this->omega_b = Odometry.gettwist().gettwist().getangular();
  }

  void LeeCtrlComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      // TODO(mereweth) - use context instead? only if in same rategroup

      if (context == LCTRL_SCHED_CONTEXT_POS) {
          using ROS::geometry_msgs::Vector3;

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

          // set angular feedback
          Eigen::Quaterniond w_q_b = Eigen::Quaterniond(this->w_q_b.getw(),
                                                        this->w_q_b.getx(),
                                                        this->w_q_b.gety(),
                                                        this->w_q_b.getz());
          this->leeControl.SetAttitudeAngVel(w_q_b,
                                             Eigen::Vector3d(
                                                 this->omega_b.getx(),
                                                 this->omega_b.gety(),
                                                 this->omega_b.getz()));

          Eigen::Vector3d a_w__comm(this->a_w__comm.getx(),
                                    this->a_w__comm.gety(),
                                    this->a_w__comm.getz());
          double mass = 1.0f; // TODO(mereweth) - replace with parm
          Eigen::Vector3d thrust_b__comm = mass * (w_q_b.inverse() * a_w__comm);

          Eigen::Vector3d alpha_b__comm(0, 0, 0);
          this->leeControl.GetAngAccelCommand(&alpha_b__comm);

          // TODO(mereweth) - replace with parm
          Eigen::Matrix3d J_b;
          J_b << 1.0f, 0.0f, 0.0f,
                 0.0f, 1.0f, 0.0f,
                 0.0f, 0.0f, 1.0f;
          Eigen::Vector3d moment_b__comm = J_b * alpha_b__comm;

          ROS::std_msgs::Header h(this->seq, this->getTime(), "body");
          ROS::mav_msgs::TorqueThrust u_b__comm(h,
            Vector3(moment_b__comm(0), moment_b__comm(1), moment_b__comm(2)),
            Vector3(thrust_b__comm(0), thrust_b__comm(1), thrust_b__comm(2)));
          if (this->isConnected_controls_OutputPort(0)) {
              this->controls_out(0, u_b__comm);
          }

          this->u_tlm[0] = thrust_b__comm(2); // z-axis only
          this->u_tlm[1] = moment_b__comm(0);
          this->u_tlm[2] = moment_b__comm(1);
          this->u_tlm[3] = moment_b__comm(2);
      }
      else if (context == LCTRL_SCHED_CONTEXT_TLM) {
          #pragma GCC diagnostic push
          #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
          COMPILE_TIME_ASSERT(FW_NUM_ARRAY_ELEMENTS(this->u_tlm) == 4,
                              LCTRL_THRUST_MOMENT_TLM_SIZE);
          #pragma GCC diagnostic pop
          this->tlmWrite_LCTRL_ThrustComm(this->u_tlm[0]);
          this->tlmWrite_LCTRL_MomentCommX(this->u_tlm[1]);
          this->tlmWrite_LCTRL_MomentCommY(this->u_tlm[2]);
          this->tlmWrite_LCTRL_MomentCommZ(this->u_tlm[3]);
      }
      else {
          // TODO(mereweth) - assert invalid port
      }
  }

} // end namespace Gnc
