// ======================================================================
// \title  FilterIfaceImpl.cpp
// \author mereweth
// \brief  cpp file for FilterIface component implementation class
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


#include <Gnc/Est/FilterIface/FilterIfaceComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

#include <Os/File.hpp>

#include "nav_msgs/Odometry.h"

#include <math.h>
#include <stdio.h>

#include <ros/callback_queue.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  FilterIfaceComponentImpl ::
#if FW_OBJECT_NAMES == 1
    FilterIfaceComponentImpl(
        const char *const compName
    ) :
      FilterIfaceComponentBase(compName),
#else
    FilterIfaceImpl(void),
#endif
    m_rosInited(false),
    m_tbDes(TB_NONE),
    m_nodeHandle(NULL),
    m_trBroad(NULL),
    m_imuStateUpdateSet(), // zero-initialize instead of default-initializing
    m_imuSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
          m_imuStateUpdateSet[i].fresh = false;
          m_imuStateUpdateSet[i].overflows = 0u;
      }
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuSet); i++) {
          m_imuSet[i].fresh = false;
          m_imuSet[i].overflows = 0u;
      }
  }

    void FilterIfaceComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      FilterIfaceComponentBase::init(instance);
    }

    FilterIfaceComponentImpl ::
      ~FilterIfaceComponentImpl(void)
    {

    }

    void FilterIfaceComponentImpl ::
      setTBDes(TimeBase tbDes) {
        this->m_tbDes = tbDes;
    }
  
    Os::Task::TaskStatus FilterIfaceComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("FILTIFACE");
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, FilterIfaceComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
    }

    void FilterIfaceComponentImpl ::
      disableRos() {
        this->m_rosInited = false;
    }
  
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void FilterIfaceComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
            m_imuStateUpdateSet[i].mutex.lock();
            if (m_imuStateUpdateSet[i].fresh) {
                if (this->isConnected_ImuStateUpdate_OutputPort(i)) {
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

        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuSet); i++) {
            m_imuSet[i].mutex.lock();
            if (m_imuSet[i].fresh) {
                if (this->isConnected_Imu_OutputPort(i)) {
                    this->Imu_out(i, m_imuSet[i].imu);
                }
                else {
                    DEBUG_PRINT("IMU port %d not connected\n", i);
                }
                m_imuSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new odometry received?
            m_imuSet[i].mutex.unLock();
        }
    }

    void FilterIfaceComponentImpl ::
      Odometry_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::nav_msgs::OdometryNoCov &Odometry
      )
    {
        nav_msgs::Odometry msg;

        ROS::std_msgs::Header header = Odometry.getheader();
        Fw::Time stamp = header.getstamp();

        // if port is not connected, default to no conversion
        Fw::Time convTime = stamp;

        if (this->isConnected_convertTime_OutputPort(0)) {
            bool success = false;
            convTime = this->convertTime_out(0, stamp, TB_ROS_TIME, 0, success);
            if (!success) {
                // TODO(Mereweth) - EVR
                DEBUG_PRINT("Failed to convert time in Odometry handler\n");
                return;
            }
        }

        header.setstamp(convTime);
        Odometry.setheader(header);
        
        if (this->isConnected_FileLogger_OutputPort(0)) {
            Svc::ActiveFileLoggerPacket fileBuff;
            Fw::SerializeStatus stat;
            Fw::Time recvTime = this->getTime();
            fileBuff.resetSer();
            stat = fileBuff.serialize((U8)AFL_FILTIFACE_ODOMNOCOV);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getUSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = Odometry.serialize(fileBuff);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);

            this->FileLogger_out(0,fileBuff);
        }

        if (!m_rosInited) {
            return;
        }

        msg.header.stamp.sec = header.getstamp().getSeconds();
        msg.header.stamp.nsec = header.getstamp().getUSeconds() * 1000L;
        
        msg.header.seq = header.getseq();

        // TODO(mereweth) - convert frame ID
        U32 frame_id = header.getframe_id();
        msg.header.frame_id = "world";

        if (0 == portNum) {
            msg.child_frame_id = "quest-base-link";
        }
        else {
            msg.child_frame_id = "quest-unknown";
        }

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

        tf::Transform tfo;
        tf::poseMsgToTF(msg.pose.pose, tfo);

        tf::StampedTransform tfoStamp;
        tfoStamp.stamp_ = msg.header.stamp;
        tfoStamp.child_frame_id_ = msg.child_frame_id;
        tfoStamp.frame_id_ = msg.header.frame_id;
        tfoStamp.setData(tfo);

        FW_ASSERT(this->m_trBroad);
        this->m_trBroad->sendTransform(tfoStamp);
    }

    void FilterIfaceComponentImpl ::
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
    void FilterIfaceComponentImpl ::
      intTaskEntry(void * ptr) {
        // TODO(mereweth) - prevent calling twice; free m_nodeHandle
        DEBUG_PRINT("FilterIface task entry\n");

        FW_ASSERT(ptr);
        FilterIfaceComponentImpl* compPtr = (FilterIfaceComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_HLROSIFACE_IntTaskStarted();

        compPtr->m_nodeHandle = new ros::NodeHandle();
        ros::NodeHandle* n = compPtr->m_nodeHandle;
        FW_ASSERT(n);
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);

        if (ros::isShuttingDown()) {
            return;
        }

        compPtr->m_trBroad = new tf::TransformBroadcaster();

        char buf[32];
        for (int i = 0; i < NUM_ODOMETRY_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "odometry_%d", i);
            compPtr->m_odomPub[i] = n->advertise<nav_msgs::Odometry>(buf, 5);
        }

        ImuStateUpdateHandler updateHandler(compPtr, 0);
        updateHandler.tbDes = compPtr->m_tbDes;
        ImuHandler imuHandler(compPtr, 0);
        imuHandler.tbDes = compPtr->m_tbDes;

        ros::Subscriber updateSub = n->subscribe("imu_state_update", 10,
                                                &ImuStateUpdateHandler::imuStateUpdateCallback,
                                                &updateHandler,
                                                ros::TransportHints().tcpNoDelay());

        ros::Subscriber imuSub = n->subscribe("ext_imu", 100,
                                              &ImuHandler::imuCallback,
                                              &imuHandler,
                                              ros::TransportHints().tcpNoDelay());
	
        compPtr->m_rosInited = true;
	
        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }

    FilterIfaceComponentImpl :: TimeBaseHolder ::
      TimeBaseHolder() :
      tbDes(TB_NONE)
    {
    }
  
    FilterIfaceComponentImpl :: ImuStateUpdateHandler ::
      ImuStateUpdateHandler(FilterIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS); //compPtr->getNum_ImuStateUpdate_OutputPorts());
    }

    FilterIfaceComponentImpl :: ImuStateUpdateHandler :: ~ImuStateUpdateHandler()
    {

    }

    void FilterIfaceComponentImpl :: ImuStateUpdateHandler ::
      imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS);

        DEBUG_PRINT("imuStateUpdate port handler %d\n", this->portNum);

        if (!std::isfinite(msg->header.stamp.sec) ||
            !std::isfinite(msg->header.stamp.nsec)) {
            //TODO(mereweth) - EVR
            return;
        }

        if (!std::isfinite(msg->pose.pose.position.x) ||
            !std::isfinite(msg->pose.pose.position.y) ||
            !std::isfinite(msg->pose.pose.position.z) ||

            !std::isfinite(msg->pose.pose.orientation.x) ||
            !std::isfinite(msg->pose.pose.orientation.y) ||
            !std::isfinite(msg->pose.pose.orientation.z) ||
            !std::isfinite(msg->pose.pose.orientation.w) ||

            !std::isfinite(msg->twist.twist.linear.x) ||
            !std::isfinite(msg->twist.twist.linear.y) ||
            !std::isfinite(msg->twist.twist.linear.z) ||

            !std::isfinite(msg->twist.twist.angular.x) ||
            !std::isfinite(msg->twist.twist.angular.y) ||
            !std::isfinite(msg->twist.twist.angular.z) ||

            !std::isfinite(msg->angular_velocity_bias.x) ||
            !std::isfinite(msg->angular_velocity_bias.y) ||
            !std::isfinite(msg->angular_velocity_bias.z) ||

            !std::isfinite(msg->linear_acceleration_bias.x) ||
            !std::isfinite(msg->linear_acceleration_bias.y) ||
            !std::isfinite(msg->linear_acceleration_bias.z)) {
            //TODO(mereweth) - EVR
            return;
        }

        Fw::Time rosTime(TB_ROS_TIME, 0,
                         msg->header.stamp.sec,
                         msg->header.stamp.nsec / 1000);

        // if port is not connected, default to no conversion
        Fw::Time convTime = rosTime;

        if (this->compPtr->isConnected_convertTime_OutputPort(0)) {
            bool success = false;
            convTime = this->compPtr->convertTime_out(0, rosTime, this->tbDes, 0, success);
            if (!success) {
                DEBUG_PRINT("Failed to convert time in ImuStateUpdate handler\n");
                // TODO(Mereweth) - EVR
                return;
            }
        }

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;

            ImuStateUpdateNoCov imuStateUpdate(
              Header(msg->header.seq,
                     convTime,
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),
              0/*Fw::EightyCharString(msg->child_frame_id.data())*/,
              Pose(Point(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z),
                   Quaternion(msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z,
                              msg->pose.pose.orientation.w)),
              Twist(Vector3(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.linear.z),
                    Vector3(msg->twist.twist.angular.x,
                            msg->twist.twist.angular.y,
                            msg->twist.twist.angular.z)),
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
        }
        this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.unLock();
    }

    FilterIfaceComponentImpl :: ImuHandler ::
      ImuHandler(FilterIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_IMU_OUTPUT_PORTS); //compPtr->getNum_Imu_OutputPorts());
    }

    FilterIfaceComponentImpl :: ImuHandler :: ~ImuHandler()
    {

    }

    void FilterIfaceComponentImpl :: ImuHandler ::
      imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_IMU_OUTPUT_PORTS);

        DEBUG_PRINT("imu port handler %d\n", this->portNum);

        if (!std::isfinite(msg->header.stamp.sec) ||
            !std::isfinite(msg->header.stamp.nsec)) {
            //TODO(mereweth) - EVR
            return;
        }

        if (!std::isfinite(msg->orientation.x) ||
            !std::isfinite(msg->orientation.y) ||
            !std::isfinite(msg->orientation.z) ||
            !std::isfinite(msg->orientation.w) ||

            !std::isfinite(msg->linear_acceleration.x) ||
            !std::isfinite(msg->linear_acceleration.y) ||
            !std::isfinite(msg->linear_acceleration.z) ||

            !std::isfinite(msg->angular_velocity.x) ||
            !std::isfinite(msg->angular_velocity.y) ||
            !std::isfinite(msg->angular_velocity.z)) {
            //TODO(mereweth) - EVR
            return;
        }
        
        Fw::Time rosTime(TB_ROS_TIME, 0,
                         msg->header.stamp.sec,
                         msg->header.stamp.nsec / 1000);

        // if port is not connected, default to no conversion
        Fw::Time convTime = rosTime;

        if (this->compPtr->isConnected_convertTime_OutputPort(0)) {
            bool success = false;
            convTime = this->compPtr->convertTime_out(0, rosTime, this->tbDes, 0, success);
            if (!success) {
                DEBUG_PRINT("Failed to convert time in Imu handler\n");
                // TODO(Mereweth) - EVR
                return;
            }
        }

        {
            using namespace ROS::std_msgs;
            using namespace ROS::sensor_msgs;
            using namespace ROS::geometry_msgs;

            ImuNoCov imu(
              Header(msg->header.seq,
                     convTime,
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),
              Quaternion(msg->orientation.x,
                         msg->orientation.y,
                         msg->orientation.z,
                         msg->orientation.w),
              Vector3(msg->angular_velocity.x,
                      msg->angular_velocity.y,
                      msg->angular_velocity.z),
              Vector3(msg->linear_acceleration.x,
                      msg->linear_acceleration.y,
                      msg->linear_acceleration.z)
            ); // end Imu constructor

            this->compPtr->m_imuSet[this->portNum].mutex.lock();
            if (this->compPtr->m_imuSet[this->portNum].fresh) {
                this->compPtr->m_imuSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting imu port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_imuSet[this->portNum].imu = imu;
            this->compPtr->m_imuSet[this->portNum].fresh = true;
        }
        this->compPtr->m_imuSet[this->portNum].mutex.unLock();
    }

} // end namespace
