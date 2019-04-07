// ======================================================================
// \title  FrameTransformImpl.cpp
// \author gene
// \brief  cpp file for FrameTransform component implementation class
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


#include <Gnc/Utils/FrameTransform/FrameTransformComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Eigen/Geometry>

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  FrameTransformComponentImpl ::
#if FW_OBJECT_NAMES == 1
    FrameTransformComponentImpl(
        const char *const compName
    ) :
      FrameTransformComponentBase(compName),
#else
      FrameTransformImpl(void),
#endif
      paramsInited(false),
      a_X_b(Eigen::Affine3d::Identity()),
      b_X_a(Eigen::Affine3d::Identity())
  {

  }

  void FrameTransformComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    FrameTransformComponentBase::init(instance);
  }

  FrameTransformComponentImpl ::
    ~FrameTransformComponentImpl(void)
  {

  }

  void FrameTransformComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
#ifndef BUILD_TIR5
    printf("prm %d updated\n", id);
#endif
  }

  void FrameTransformComponentImpl ::
    parametersLoaded()
  {
      using namespace Eigen;

      Fw::ParamValid valid[7];
      const Matrix3d a_R_b(Quaterniond(paramGet_a_q_b__w(valid[0]),
                                       paramGet_a_q_b__x(valid[1]),
                                       paramGet_a_q_b__y(valid[2]),
                                       paramGet_a_q_b__z(valid[3])));
      const Translation3d a_r_b(paramGet_a_r_b__x(valid[4]),
                                paramGet_a_r_b__y(valid[5]),
                                paramGet_a_r_b__z(valid[6]));

      for (unsigned int i = 0; i < 7; i++) {
          if (Fw::PARAM_VALID != valid[i]) {  return;  }
      }

      this->a_X_b = a_R_b * a_r_b; // translation after rotation
      this->b_X_a = this->a_X_b.inverse();

      this->paramsInited = true;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void FrameTransformComponentImpl ::
    odomInA_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::nav_msgs::Odometry &Odometry
    )
  {
      using namespace Eigen;
      ROS::geometry_msgs::PoseWithCovariance poseCov = Odometry.getpose();
      ROS::geometry_msgs::Pose pose = poseCov.getpose();
      ROS::geometry_msgs::Point p = pose.getposition();
      const Vector3d p_b = this->b_X_a * Vector3d(p.getx(), p.gety(), p.getz());
      p.setx(p_b(0));
      p.sety(p_b(1));
      p.setz(p_b(2));
      pose.setposition(p);
      
      ROS::geometry_msgs::Quaternion q = pose.getorientation();
      const Quaterniond q_b(this->b_X_a.rotation()
			    * Quaterniond(q.getw(), q.getx(), q.gety(), q.getz()));
      q.setw(q_b.w());
      q.setx(q_b.x());
      q.sety(q_b.y());
      q.setz(q_b.z());
      pose.setorientation(q);
      poseCov.setpose(pose);
      Odometry.setpose(poseCov);
      
      ROS::geometry_msgs::TwistWithCovariance twistCov = Odometry.gettwist();
      ROS::geometry_msgs::Twist twist = twistCov.gettwist();
      ROS::geometry_msgs::Vector3 lin = twist.getlinear();
      const Vector3d lin_b = this->b_X_a * Vector3d(lin.getx(), lin.gety(), lin.getz());
      lin.setx(lin_b(0));
      lin.sety(lin_b(1));
      lin.setz(lin_b(2));
      twist.setlinear(lin);

      ROS::geometry_msgs::Vector3 ang = twist.getangular();
      const Vector3d ang_b = this->b_X_a * Vector3d(ang.getx(), ang.gety(), ang.getz());
      ang.setx(ang_b(0));
      ang.sety(ang_b(1));
      ang.setz(ang_b(2));
      twist.setangular(ang);
      twistCov.settwist(twist);
      Odometry.settwist(twistCov);

      this->odomOutB_out(portNum, Odometry);
  }

  void FrameTransformComponentImpl ::
    odomInB_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::nav_msgs::Odometry &Odometry
    )
  {
      using namespace Eigen;
      ROS::geometry_msgs::PoseWithCovariance poseCov = Odometry.getpose();
      ROS::geometry_msgs::Pose pose = poseCov.getpose();
      ROS::geometry_msgs::Point p = pose.getposition();
      const Vector3d p_a = this->a_X_b * Vector3d(p.getx(), p.gety(), p.getz());
      p.setx(p_a(0));
      p.sety(p_a(1));
      p.setz(p_a(2));
      pose.setposition(p);
      
      ROS::geometry_msgs::Quaternion q = pose.getorientation();
      const Quaterniond q_a(this->a_X_b.rotation()
			    * Quaterniond(q.getw(), q.getx(), q.gety(), q.getz()));
      q.setw(q_a.w());
      q.setx(q_a.x());
      q.sety(q_a.y());
      q.setz(q_a.z());
      pose.setorientation(q);
      poseCov.setpose(pose);
      Odometry.setpose(poseCov);
      
      ROS::geometry_msgs::TwistWithCovariance twistCov = Odometry.gettwist();
      ROS::geometry_msgs::Twist twist = twistCov.gettwist();
      ROS::geometry_msgs::Vector3 lin = twist.getlinear();
      const Vector3d lin_a = this->a_X_b * Vector3d(lin.getx(), lin.gety(), lin.getz());
      lin.setx(lin_a(0));
      lin.sety(lin_a(1));
      lin.setz(lin_a(2));
      twist.setlinear(lin);

      ROS::geometry_msgs::Vector3 ang = twist.getangular();
      const Vector3d ang_a = this->a_X_b * Vector3d(ang.getx(), ang.gety(), ang.getz());
      ang.setx(ang_a(0));
      ang.sety(ang_a(1));
      ang.setz(ang_a(2));
      twist.setangular(ang);
      twistCov.settwist(twist);
      Odometry.settwist(twistCov);

      this->odomOutA_out(portNum, Odometry);
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void FrameTransformComponentImpl ::
    FTFO_InitParams_cmdHandler(
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
