// ======================================================================
// \title  HLRosIfaceImpl.cpp
// \author mereweth
// \brief  cpp file for HLRosIface component implementation class
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


#include <HLProc/HLRosIface/HLRosIfaceComponentImpl.hpp>
#include <HLProc/HLRosIface/HLRosIfaceComponentImplCfg.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <stdio.h>

#include <ros/callback_queue.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace HLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  HLRosIfaceComponentImpl ::
#if FW_OBJECT_NAMES == 1
    HLRosIfaceComponentImpl(
        const char *const compName
    ) :
      HLRosIfaceComponentBase(compName),
#else
    HLRosIfaceImpl(void),
#endif
    m_rgNH(NULL),
    m_imuStateUpdateSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
          m_imuStateUpdateSet[i].fresh = false;
          m_imuStateUpdateSet[i].overflows = 0u;
      }
  }

    void HLRosIfaceComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      HLRosIfaceComponentBase::init(instance);
    }

    HLRosIfaceComponentImpl ::
      ~HLRosIfaceComponentImpl(void)
    {

    }

    void HLRosIfaceComponentImpl ::
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

    Os::Task::TaskStatus HLRosIfaceComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("HLROSIFACE");
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, HLRosIfaceComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
    }
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void HLRosIfaceComponentImpl ::
      Imu_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::sensor_msgs::ImuNoCov &Imu
      )
    {
        if (NULL == m_rgNH) {
            return;
        }
      
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

    void HLRosIfaceComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
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

        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_actuatorsSet); i++) {
            m_actuatorsSet[i].mutex.lock();
            if (m_actuatorsSet[i].fresh) {
                if (this->isConnected_ActuatorsData_OutputPort(i)) {
                    // mimics driver hardware getting and sending sensor data
                    this->ActuatorsData_out(i, m_actuatorsSet[i].actuators);
                }
                else {
                    DEBUG_PRINT("Actuators port %d not connected\n", i);
                }
                m_actuatorsSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new actuators received?
            m_actuatorsSet[i].mutex.unLock();
        }
    }

    void HLRosIfaceComponentImpl ::
      Odometry_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::nav_msgs::OdometryNoCov &Odometry
      )
    {
        if (NULL == m_rgNH) {
            return;
        }
        
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

    void HLRosIfaceComponentImpl ::
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
    void HLRosIfaceComponentImpl ::
      intTaskEntry(void * ptr) {
        DEBUG_PRINT("HLRosIface task entry\n");

        FW_ASSERT(ptr);
        HLRosIfaceComponentImpl* compPtr = (HLRosIfaceComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_HLROSIFACE_IntTaskStarted();

        ros::NodeHandle n;
        ros::CallbackQueue localCallbacks;
        n.setCallbackQueue(&localCallbacks);

        ImuStateUpdateHandler gtHandler(compPtr, 0);

        ros::Subscriber gtSub = n.subscribe("ground_truth/odometry", 10,
                                            &ImuStateUpdateHandler::imuStateUpdateCallback,
                                            &gtHandler,
                                            ros::TransportHints().tcpNoDelay());


        ActuatorsHandler actuatorsHandler0(compPtr, 0);
        ros::Subscriber actuatorsSub0 = n.subscribe("dsp_pwm_command", 10,
                                            &ActuatorsHandler::actuatorsCallback,
                                            &actuatorsHandler0,
                                            ros::TransportHints().tcpNoDelay());
        
        ActuatorsHandler actuatorsHandler1(compPtr, 1);
        ros::Subscriber actuatorsSub1 = n.subscribe("dsp_i2c_command", 10,
                                            &ActuatorsHandler::actuatorsCallback,
                                            &actuatorsHandler1,
                                            ros::TransportHints().tcpNoDelay());

        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }

    HLRosIfaceComponentImpl :: ActuatorsHandler ::
      ActuatorsHandler(HLRosIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_ACTUATORSDATA_OUTPUT_PORTS); //compPtr->getNum_ImuStateUpdate_OutputPorts());
    }

    HLRosIfaceComponentImpl :: ActuatorsHandler :: ~ActuatorsHandler()
    {

    }

    void HLRosIfaceComponentImpl :: ActuatorsHandler ::
      actuatorsCallback(const mav_msgs::Actuators::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_ACTUATORSDATA_OUTPUT_PORTS);

        DEBUG_PRINT("ActuatorsData port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            Header head(msg->header.seq,
                        Fw::Time(TB_ROS_TIME, 0,
                                 msg->header.stamp.sec,
                                 msg->header.stamp.nsec / 1000),
                        Fw::EightyCharString(msg->header.frame_id.data()));

            Actuators actuators;
            actuators.setheader(head);
            NATIVE_INT_TYPE size;
            /* TODO(mereweth) - convention on whether to make count fields the original
             * ROS value or the min of that and the allocated size
             */
            size = msg->angles.size();
            actuators.setangles(msg->angles.data(), size);
            actuators.setangles_count(msg->angles.size());
            
            size = msg->angular_velocities.size();
            actuators.setangular_velocities(msg->angular_velocities.data(), size);
            actuators.setangular_velocities_count(msg->angular_velocities.size());

            size = msg->normalized.size();
            actuators.setnormalized(msg->normalized.data(), size);
            actuators.setnormalized_count(msg->normalized.size());

            DEBUG_PRINT("Actuators port %d sizes %u, %u, %u\n",
                        msg->angles.size(),
                        msg->angular_velocities.size(),
                        msg->normalized.size());

            this->compPtr->m_actuatorsSet[this->portNum].mutex.lock();
            if (this->compPtr->m_actuatorsSet[this->portNum].fresh) {
                this->compPtr->m_actuatorsSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting Actuators port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_actuatorsSet[this->portNum].actuators = actuators;
            this->compPtr->m_actuatorsSet[this->portNum].fresh = true;
            this->compPtr->m_actuatorsSet[this->portNum].mutex.unLock();
        }
    }

    HLRosIfaceComponentImpl :: ImuStateUpdateHandler ::
      ImuStateUpdateHandler(HLRosIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS); //compPtr->getNum_ImuStateUpdate_OutputPorts());
    }

    HLRosIfaceComponentImpl :: ImuStateUpdateHandler :: ~ImuStateUpdateHandler()
    {

    }

    void HLRosIfaceComponentImpl :: ImuStateUpdateHandler ::
      imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS);

        DEBUG_PRINT("imuStateUpdate port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            ImuStateUpdate imuStateUpdate(
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
                                  msg->twist.covariance.data(), 36),
              Vector3(msg->angular_velocity_bias.x,
                      msg->angular_velocity_bias.y,
                      msg->angular_velocity_bias.z),
              Vector3(msg->linear_acceleration_bias.x,
                      msg->linear_acceleration_bias.y,
                      msg->linear_acceleration_bias.z)
            ); // end ImuStateUpdate constructor

            this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.lock();
            if (this->compPtr->m_imuStateUpdateSet[this->portNum].fresh) {
                this->compPtr->m_imuStateUpdateSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting imuStateUpdate port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_imuStateUpdateSet[this->portNum].imuStateUpdate = imuStateUpdate;
            this->compPtr->m_imuStateUpdateSet[this->portNum].fresh = true;
            this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.unLock();
        }
    }


} // end namespace
