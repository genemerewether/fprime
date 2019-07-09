// ====================================================================== 
// \title  ImuSplitterImpl.cpp
// \author kubiak
// \brief  cpp file for ImuSplitter component implementation class
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


#include <R5RELAY/ImuSplitter/ImuSplitterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5RELAY {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ImuSplitterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ImuSplitterComponentImpl(
        const char *const compName
    ) :
      ImuSplitterComponentBase(compName)
#else
    ImuSplitterImpl(void)
#endif
  {

  }

  void ImuSplitterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    ImuSplitterComponentBase::init(instance);
  }

  ImuSplitterComponentImpl ::
    ~ImuSplitterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ImuSplitterComponentImpl ::
    IMU_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
    Fw::Time time;
    ROS::geometry_msgs::Quaternion orientation;
    ROS::geometry_msgs::Vector3 ang_vel;
    ROS::geometry_msgs::Vector3 lin_accel;

    time = ImuNoCov.getheader().getstamp();
    orientation = ImuNoCov.getorientation();
    ang_vel = ImuNoCov.getangular_velocity();
    lin_accel = ImuNoCov.getlinear_acceleration();

    printf("%.3f %.3f %.3f %.3f %.3f %.3f\n",
           lin_accel.getx(),
           lin_accel.gety(),
           lin_accel.getz(),
           ang_vel.getx(),
           ang_vel.gety(),
           ang_vel.getz());

    tlmWrite_ImuSeq(ImuNoCov.getheader().getseq());
    tlmWrite_ImuStampSec(time.getSeconds());
    tlmWrite_ImuStampUSec(time.getUSeconds());
    tlmWrite_ImuFrame(ImuNoCov.getheader().getframe_id());
    tlmWrite_ImuOrientation(orientation);
    tlmWrite_ImuAngularVelocity(ang_vel);
    tlmWrite_ImuLinearAcceleration(lin_accel);
  }

} // end namespace R5RELAY
