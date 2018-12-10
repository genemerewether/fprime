// ======================================================================
// \title  ImuIntegImpl.cpp
// \author mereweth
// \brief  cpp file for ImuInteg component implementation class
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


#include <Gnc/Est/ImuInteg/ImuIntegComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

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

  ImuIntegComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ImuIntegComponentImpl(
        const char *const compName
    ) :
      ImuIntegComponentBase(compName),
#else
    ImuIntegComponentImpl(void) :
      ImuIntegComponentImpl(void),
#endif
      seq(0u),
      omega_b(0.0f, 0.0f, 0.0f),
      a_b(0.0f, 0.0f, 0.0f),
      x_w(0.0f, 0.0f, 0.0f),
      w_q_b(0.0f, 0.0f, 0.0f, 1.0f),
      v_b(0.0f, 0.0f, 0.0f),
      imuInteg(),
      paramsInited(false)
  {
      quest_gnc::WorldParams wParams = {9.80665f, 1.2f};
      (void) imuInteg.SetWorldParams(wParams);

  }

  void ImuIntegComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    ImuIntegComponentBase::init(instance);
  }

  ImuIntegComponentImpl ::
    ~ImuIntegComponentImpl(void)
  {

  }
  
  void ImuIntegComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
    printf("prm %d updated\n", id);
  }
  
  void ImuIntegComponentImpl ::
    parametersLoaded()
  {
      Fw::ParamValid valid[1];
      imuInteg.SetTimeStep(paramGet_dt(valid[0]));
      if (Fw::PARAM_VALID != valid[0]) {  return;  }
      
      paramsInited = true;
  }
      
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------
  
  void ImuIntegComponentImpl ::
    Imu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
      ROS::std_msgs::Header h = ImuNoCov.getheader();
      this->seq = h.getseq();
      // TODO(mereweth) check h.getstamp()

      this->omega_b = ImuNoCov.getangular_velocity();
      this->a_b = ImuNoCov.getlinear_acceleration();

      this->imuInteg.AddImu(Eigen::Vector3d(this->omega_b.getx(),
                                            this->omega_b.gety(),
                                            this->omega_b.getz()),
                            Eigen::Vector3d(this->a_b.getx(),
                                            this->a_b.gety(),
                                            this->a_b.getz()));
  }


  void ImuIntegComponentImpl ::
    ImuStateUpdate_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate
    )
  {
      DEBUG_PRINT("IMU state update\n");
    
      ROS::std_msgs::Header h = ImuStateUpdate.getheader();
      //this->seq = h.getseq();
      // TODO(mereweth) convert h.getstamp()

      this->x_w = ImuStateUpdate.getpose().getpose().getposition();
      this->w_q_b = ImuStateUpdate.getpose().getpose().getorientation();

      ROS::geometry_msgs::Vector3 v_w = ImuStateUpdate.gettwist().gettwist().getlinear();
      this->omega_b = ImuStateUpdate.gettwist().gettwist().getangular();

      this->wBias = ImuStateUpdate.getangular_velocity_bias();
      this->aBias = ImuStateUpdate.getlinear_acceleration_bias();

      Eigen::Quaterniond w_q_b = Eigen::Quaterniond(this->w_q_b.getw(),
                                                    this->w_q_b.getx(),
                                                    this->w_q_b.gety(),
                                                    this->w_q_b.getz());

      this->imuInteg.SetUpdate(h.getstamp().getSeconds()
                               + h.getstamp().getUSeconds() / 1000.0 / 1000.0,
                               Eigen::Vector3d(this->x_w.getx(),
                                               this->x_w.gety(),
                                               this->x_w.getz()),
                               w_q_b,
                               Eigen::Vector3d(v_w.getx(),
                                               v_w.gety(),
                                               v_w.getz()),
                               Eigen::Vector3d(this->wBias.getx(),
                                               this->wBias.gety(),
                                               this->wBias.getz()),
                               Eigen::Vector3d(this->aBias.getx(),
                                               this->aBias.gety(),
                                               this->aBias.getz()));
  }

  void ImuIntegComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      //TODO(mereweth) - report uninitialized status if params not good
    
      if ((context == IMUINTEG_SCHED_CONTEXT_POS) ||
          (context == IMUINTEG_SCHED_CONTEXT_ATT)) {
          using namespace ROS::geometry_msgs;
          this->imuInteg.PropagateState();
          Eigen::Vector3d x_w(0, 0, 0);
          Eigen::Quaterniond w_q_b(1, 0, 0, 0);
          Eigen::Vector3d v_b(0, 0, 0);
          Eigen::Vector3d omega_b(0, 0, 0);
          this->imuInteg.GetState(&x_w,
                                  &w_q_b,
                                  &v_b,
                                  &omega_b);

          // TODO(mereweth) - convert frame name to U32 idx
          ROS::std_msgs::Header h(this->seq, this->getTime(), 0/*"odom"*/);
          ROS::nav_msgs::Odometry odom(h, 0/*"body"*/,
              PoseWithCovariance(Pose(Point(x_w(0), x_w(1), x_w(2)),
                                      Quaternion(w_q_b.x(), w_q_b.y(),
                                                 w_q_b.z(), w_q_b.w())),
                                 NULL, 0), // don't use covariance estimates
              TwistWithCovariance(Twist(Vector3(v_b(0), v_b(1), v_b(2)),
                                        Vector3(omega_b(0), omega_b(1), omega_b(2))),
                                  NULL, 0) // don't use covariance estimates
              );
          if (this->isConnected_odometry_OutputPort(0)) {
              this->odometry_out(0, odom);
          }

          if (context == IMUINTEG_SCHED_CONTEXT_POS) {
              ROS::nav_msgs::OdometryNoCov odomNoCov(h, 0/*"body"*/,
                                     Pose(Point(x_w(0), x_w(1), x_w(2)),
                                          Quaternion(w_q_b.x(), w_q_b.y(),
                                                     w_q_b.z(), w_q_b.w())),
                                     Twist(Vector3(v_b(0), v_b(1), v_b(2)),
                                            Vector3(omega_b(0), omega_b(1), omega_b(2)))
                  );
              if (this->isConnected_odomNoCov_OutputPort(0)) {
                  this->odomNoCov_out(0, odomNoCov);
              }
          }
      }
      else if (context == IMUINTEG_SCHED_CONTEXT_TLM) {

      }
      else {
          // TODO(mereweth) - assert invalid context
      }
  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void ImuIntegComponentImpl ::
    IMUINTEG_InitParams_cmdHandler(
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

} // end namespace Gnc
