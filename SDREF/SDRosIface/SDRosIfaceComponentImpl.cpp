// ======================================================================
// \title  SDRosIfaceImpl.cpp
// \author mereweth
// \brief  cpp file for SDRosIface component implementation class
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


#include <SDREF/SDRosIface/SDRosIfaceComponentImpl.hpp>
#include <SDREF/SDRosIface/SDRosIfaceComponentImplCfg.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace SDREF {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  SDRosIfaceComponentImpl ::
#if FW_OBJECT_NAMES == 1
    SDRosIfaceComponentImpl(
        const char *const compName
    ) :
      SDRosIfaceComponentBase(compName),
#else
    SDRosIfaceImpl(void),
#endif
    m_rgNH(NULL)
  {

  }

    void SDRosIfaceComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      SDRosIfaceComponentBase::init(instance);
    }

    SDRosIfaceComponentImpl ::
      ~SDRosIfaceComponentImpl(void)
    {

    }

    void SDRosIfaceComponentImpl ::
      startPub() {
        ros::NodeHandle n;
        m_rgNH = &n;

        char buf[32];
        for (int i = 0; i < NUM_IMU_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "imu_%d", i);
            m_imuPub[i] = m_rgNH->advertise<sensor_msgs::Imu>(buf, 5);
        }

        for (int i = 0; i < NUM_ODOMETRY_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "odometry_%d", i);
            m_odomPub[i] = m_rgNH->advertise<nav_msgs::Odometry>(buf, 5);
        }
    }
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void SDRosIfaceComponentImpl ::
      Imu_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::sensor_msgs::ImuNoCov &Imu
      )
    {
        sensor_msgs::Imu msg;

        ROS::std_msgs::Header header = Imu.getheader();
        // TODO(mereweth) - time-translate from DSP & use message there
        Fw::Time stamp = header.getstamp();
        msg.header.stamp = ros::Time::now();

        msg.header.seq = header.getseq();

        // TODO(mereweth) - convert frame ID
        Fw::EightyCharString frame_id = header.getframe_id();
        msg.header.frame_id = "mpu9250";

        msg.orientation_covariance[0] = -1;

        ROS::geometry_msgs::Vector3 vec = Imu.getlinear_acceleration();
        msg.linear_acceleration.x = vec.getx();
        msg.linear_acceleration.y = vec.gety();
        msg.linear_acceleration.z = vec.getz();
        vec = Imu.getangular_velocity();
        msg.angular_velocity.x = vec.getx();
        msg.angular_velocity.y = vec.gety();
        msg.angular_velocity.z = vec.getz();
        m_imuPub[portNum].publish(msg);
    }

    void SDRosIfaceComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        // TODO(mereweth) -
        if ((context == SDROSIFACE_SCHED_CONTEXT_ATT) ||
            (context == SDROSIFACE_SCHED_CONTEXT_POS)) {
            // NOTE(mereweth) - in case we add odometry in from ROS-land
            // TODO(mereweth) check context == odometry out context
            /*for (int i = 0; i < NUM_ODOMETRY_OUTPUT_PORTS; i++) {
                m_odomSet[i].mutex.lock();
                if (m_odomSet[i].fresh) {
                    if (this->isConnected_odometry_OutputPort(i)) {
                        // mimics driver hardware getting and sending sensor data
                        this->odometry_out(i, m_odomSet[i].odom);
                    }
                    else {
                        DEBUG_PRINT("Odom port %d not connected\n", i);
                    }
                    m_odomSet[i].fresh = false;
                }
                // TODO(mereweth) - notify that no new odometry received?
                m_odomSet[i].mutex.unLock();
            }*/

            // TODO(mereweth) check context == imu out context
        }
        else if (context == SDROSIFACE_SCHED_CONTEXT_TLM) {
            // NOTE(mereweth) - in case we add odometry in from ROS-land
            /*FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(m_odomSet) >= 2,
                      FW_NUM_ARRAY_ELEMENTS(m_odomSet));
            this->tlmWrite_SDROSIFACE_Odometry1Overflows(m_odomSet[0].overflows);
            this->tlmWrite_SDROSIFACE_Odometry2Overflows(m_odomSet[1].overflows);*/
        }
    }

    void SDRosIfaceComponentImpl ::
      Odometry_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::nav_msgs::OdometryNoCov &Odometry
      )
    {
        nav_msgs::Odometry msg;

        ROS::std_msgs::Header header = Odometry.getheader();
        // TODO(mereweth) - time-translate from DSP & use message there
        Fw::Time stamp = header.getstamp();
        msg.header.stamp = ros::Time::now();

        msg.header.seq = header.getseq();

        // TODO(mereweth) - convert frame ID
        Fw::EightyCharString frame_id = header.getframe_id();
        msg.header.frame_id = "odom";

        msg.child_frame_id = "body";

        ROS::geometry_msgs::Point p = Odometry.getpose().getposition();
        msg.pose.pose.position.x = p.getx();
        msg.pose.pose.position.y = p.gety();
        msg.pose.pose.position.z = p.getz();
        ROS::geometry_msgs::Quaternion q = Odometry.getpose().getorientation();
        msg.pose.pose.orientation.w = q.getw();
        msg.pose.pose.orientation.x = q.getx();
        msg.pose.pose.orientation.y = q.gety();
        msg.pose.pose.orientation.z = q.getz();
        ROS::geometry_msgs::Vector3 vec = Odometry.gettwist().getlinear();
        msg.twist.twist.linear.x = vec.getx();
        msg.twist.twist.linear.y = vec.gety();
        msg.twist.twist.linear.z = vec.getz();
        vec = Odometry.gettwist().getangular();
        msg.twist.twist.angular.x = vec.getx();
        msg.twist.twist.angular.y = vec.gety();
        msg.twist.twist.angular.z = vec.getz();

        m_odomPub[portNum].publish(msg);
    }

    void SDRosIfaceComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        // TODO(mereweth) - check that message-wait task is OK if we add one
        this->pingOut_out(portNum, key);
    }

} // end namespace
