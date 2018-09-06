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

#include <stdio.h>

#include <ros/callback_queue.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

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
    m_rgNH(NULL),
    m_imuStateUpdateSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
          m_imuStateUpdateSet[i].fresh = false;
          m_imuStateUpdateSet[i].overflows = 0u;
      }
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

    Os::Task::TaskStatus SDRosIfaceComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("SDROSIFACE");
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, SDRosIfaceComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
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
        if ((context == SDROSIFACE_SCHED_CONTEXT_ATT) ||
            (context == SDROSIFACE_SCHED_CONTEXT_POS)) {
            // TODO(mereweth) check context == odometry out context
            for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
                m_imuStateUpdateSet[i].mutex.lock();
                if (m_imuStateUpdateSet[i].fresh) {
                    if (this->isConnected_ImuStateUpdate_OutputPort(i)) {
                        // mimics driver hardware getting and sending sensor data
                        this->ImuStateUpdate_out(i, m_imuStateUpdateSet[i].imuStateUpdate);
                    }
                    else {
                        DEBUG_PRINT("IMU state update port %d not connected\n", i);
                    }
                    m_imuStateUpdateSet[i].fresh = false;
                }
                // TODO(mereweth) - notify that no new odometry received?
                m_imuStateUpdateSet[i].mutex.unLock();
            }

            // TODO(mereweth) check context == imu out context
        }
        else if (context == SDROSIFACE_SCHED_CONTEXT_TLM) {
            /*FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet) >= 2,
                      FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet));
            this->tlmWrite_SDROSIFACE_ImuStateUpdate1Overflows(m_imuStateUpdateSet[0].overflows);
            this->tlmWrite_SDROSIFACE_ImuStateUpdate2Overflows(m_imuStateUpdateSet[1].overflows);*/
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

    // ----------------------------------------------------------------------
    // Member function definitions
    // ----------------------------------------------------------------------

    //! Entry point for task waiting for messages
    void SDRosIfaceComponentImpl ::
      intTaskEntry(void * ptr) {
        DEBUG_PRINT("SDRosIface task entry\n");

        FW_ASSERT(ptr);
        SDRosIfaceComponentImpl* compPtr = (SDRosIfaceComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_SDROSIFACE_IntTaskStarted();

        ros::NodeHandle n;
        ros::CallbackQueue localCallbacks;
        n.setCallbackQueue(&localCallbacks);
/*
        OdometryHandler gtHandler(compPtr, 0);

        ros::Subscriber gtSub = n.subscribe("ground_truth/odometry", 1000,
                                            &ImuStateUpdateHandler::imuStateUpdateCallback,
                                            &gtHandler,
                                            ros::TransportHints().tcpNoDelay());
*/
        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }

    SDRosIfaceComponentImpl :: ImuStateUpdateHandler ::
      ImuStateUpdateHandler(SDRosIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS); //compPtr->getNum_ImuStateUpdate_OutputPorts());
    }

    SDRosIfaceComponentImpl :: ImuStateUpdateHandler :: ~ImuStateUpdateHandler()
    {

    }

/*    void SDRosIfaceComponentImpl :: ImuStateUpdateHandler ::
      imuStateUpdateCallback(const sensor_msgs::ImuStateUpdate::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS);

        DEBUG_PRINT("imuStateUpdate port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::sensor_msgs;
            using namespace ROS::geometry_msgs;
            /*Odometry odom(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
                     Fw::EightyCharString(msg->header.frame_id.data())),
              Fw::EightyCharString(msg->child_frame_id.data()),
              PoseWithCovariance(Pose(Point(msg->pose.pose.position.x,
                                            msg->pose.pose.position.y,
                                            msg->pose.pose.position.z),
                                      Quaternion(msg->pose.pose.orientation.x,
                                                 msg->pose.pose.orientation.y,
                                                 msg->pose.pose.orientation.z,
                                                 msg->pose.pose.orientation.w)),
                                 msg->pose.covariance.data(), 36),
              TwistWithCovariance(Twist(Vector3(msg->twist.twist.linear.x,
                                                msg->twist.twist.linear.y,
                                                msg->twist.twist.linear.z),
                                        Vector3(msg->twist.twist.angular.x,
                                                msg->twist.twist.angular.y,
                                                msg->twist.twist.angular.z)),
                                  msg->twist.covariance.data(), 36)
            ); // end Odometry constructor
*/
/*            this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.lock();
            if (this->compPtr->m_imuStateUpdateSet[this->portNum].fresh) {
                this->compPtr->m_imuStateUpdateSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting imuStateUpdate port %d before Sched\n", this->portNum);
            }
            //this->compPtr->m_imuStateUpdateSet[this->portNum].imuStateUpdate = imuStateUpdate;
            this->compPtr->m_imuStateUpdateSet[this->portNum].fresh = true;
            this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.unLock();
        }
    }*/


} // end namespace
