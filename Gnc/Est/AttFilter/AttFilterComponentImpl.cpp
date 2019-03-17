// ======================================================================
// \title  AttFilterImpl.cpp
// \author mereweth
// \brief  cpp file for AttFilter component implementation class
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


#include <Gnc/Est/AttFilter/AttFilterComponentImpl.hpp>
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

  AttFilterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    AttFilterComponentImpl(
        const char *const compName
    ) :
      AttFilterComponentBase(compName),
#else
    AttFilterComponentImpl(void) :
      AttFilterComponentBase(void),
#endif
      seq(0u),
      attFilter(),
      paramsInited(false)
  {
      quest_gnc::WorldParams wParams = {9.80665f, 1.2f};
      (void) attFilter.SetWorldParams(wParams);

  }

  void AttFilterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    AttFilterComponentBase::init(instance);
  }

  AttFilterComponentImpl ::
    ~AttFilterComponentImpl(void)
  {

  }
  
  void AttFilterComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
#ifndef BUILD_TIR5
    printf("prm %d updated\n", id);
#endif
  }
  
  void AttFilterComponentImpl ::
    parametersLoaded()
  {
      this->paramsInited = false;
      Fw::ParamValid valid[1];
      attFilter.SetTimeStep(paramGet_dt(valid[0]));
      if (Fw::PARAM_VALID != valid[0]) {  return;  }

      attFilter.SetAccelGain(paramGet_accelGain(valid[0]));
      if ((Fw::PARAM_VALID != valid[0]) &&
          (Fw::PARAM_DEFAULT != valid[0])) {  return;  }

      attFilter.SetBiasAlpha(paramGet_biasAlpha(valid[0]));
      if ((Fw::PARAM_VALID != valid[0]) &&
          (Fw::PARAM_DEFAULT != valid[0])) {  return;  }

      attFilter.SetSteadyStateThresh(0.1, 0.01, 0.2);
      
      this->paramsInited = true;
  }
      
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------
  
  void AttFilterComponentImpl ::
    Imu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
      ROS::std_msgs::Header h = ImuNoCov.getheader();
      this->seq = h.getseq();
      // TODO(mereweth) check h.getstamp()

      ROS::geometry_msgs::Vector3 omega_b = ImuNoCov.getangular_velocity();
      ROS::geometry_msgs::Vector3 a_b = ImuNoCov.getlinear_acceleration();
      
      quest_gnc::ImuSample imu;
      imu.t = h.getstamp().getSeconds() 
        + h.getstamp().getUSeconds() / 1000.0 / 1000.0;
      imu.omega_b = Eigen::Vector3d(omega_b.getx(),
                                    omega_b.gety(),
                                    omega_b.getz());
      imu.a_b = Eigen::Vector3d(a_b.getx(),
                                a_b.gety(),
                                a_b.getz());

      this->attFilter.AddImu(imu);
  }


  void AttFilterComponentImpl ::
    ImuStateUpdate_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::ImuStateUpdateNoCov &ImuStateUpdate
    )
  {
      DEBUG_PRINT("IMU state update\n");
    
      ROS::std_msgs::Header h = ImuStateUpdate.getheader();
      
      //this->seq = h.getseq();

      ROS::geometry_msgs::Point x_w = ImuStateUpdate.getpose().getposition();
      ROS::geometry_msgs::Quaternion w_q_b = ImuStateUpdate.getpose().getorientation();
      ROS::geometry_msgs::Vector3 v_w = ImuStateUpdate.gettwist().getlinear();
      //this->omega_b = ImuStateUpdate.gettwist().getangular();

      ROS::geometry_msgs::Vector3 wBias = ImuStateUpdate.getangular_velocity_bias();
      ROS::geometry_msgs::Vector3 aBias = ImuStateUpdate.getlinear_acceleration_bias();

      this->attFilter.SetUpdate(h.getstamp().getSeconds()
                               + h.getstamp().getUSeconds() / 1000.0 / 1000.0,
                               Eigen::Vector3d(x_w.getx(),
                                               x_w.gety(),
                                               x_w.getz()),
                               Eigen::Quaterniond(w_q_b.getw(),
                                                  w_q_b.getx(),
                                                  w_q_b.gety(),
                                                  w_q_b.getz()),
                               Eigen::Vector3d(v_w.getx(),
                                               v_w.gety(),
                                               v_w.getz()),
                               Eigen::Vector3d(wBias.getx(),
                                               wBias.gety(),
                                               wBias.getz()),
                               Eigen::Vector3d(aBias.getx(),
                                               aBias.gety(),
                                               aBias.getz()));
  }

  void AttFilterComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      //TODO(mereweth) - report uninitialized status if params not good
    
      if ((context == ATTFILTER_SCHED_CONTEXT_POS) ||
          (context == ATTFILTER_SCHED_CONTEXT_ATT)) {
          using namespace ROS::geometry_msgs;
          this->attFilter.PropagateState();
          Eigen::Vector3d x_w(0, 0, 0);
          Eigen::Quaterniond w_q_b(1, 0, 0, 0);
          Eigen::Vector3d v_b(0, 0, 0);
          Eigen::Vector3d omega_b(0, 0, 0);
          this->attFilter.GetState(&x_w,
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
      else if (context == ATTFILTER_SCHED_CONTEXT_TLM) {

      }
      else {
          // TODO(mereweth) - assert invalid context
      }
  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void AttFilterComponentImpl ::
    ATTFILTER_InitParams_cmdHandler(
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
