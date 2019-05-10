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

#include <math.h>
#include <stdio.h>

#include <ros/callback_queue.h>

#define DO_TIME_CONV

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
    m_nodeHandle(NULL),
    m_trBroad(NULL),
    m_imuStateUpdateSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
          m_imuStateUpdateSet[i].fresh = false;
          m_imuStateUpdateSet[i].overflows = 0u;
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
      startPub() {
        // TODO(mereweth) - prevent calling twice
        FW_ASSERT(m_nodeHandle);
        ros::NodeHandle* n = this->m_nodeHandle;
        this->m_trBroad = new tf::TransformBroadcaster();

        char buf[32];
        for (int i = 0; i < NUM_ODOMETRY_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "odometry_%d", i);
            m_odomPub[i] = n->advertise<nav_msgs::Odometry>(buf, 5);
        }

        m_rosInited = true;
    }

    Os::Task::TaskStatus FilterIfaceComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("FILTIFACE");
        this->m_nodeHandle = new ros::NodeHandle();
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, FilterIfaceComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
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
#ifdef DO_TIME_CONV

        //TODO(mereweth) - BEGIN convert time instead using HLTimeConv

        const I64 usecDsp = (I64) stamp.getSeconds() * 1000LL * 1000LL + (I64) stamp.getUSeconds();
        Os::File::Status stat = Os::File::OTHER_ERROR;
        Os::File file;
        stat = file.open("/sys/kernel/dsp_offset/walltime_dsp_diff", Os::File::OPEN_READ);
        if (stat != Os::File::OP_OK) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        char buff[255];
        NATIVE_INT_TYPE size = sizeof(buff);
        stat = file.read(buff, size, false);
        file.close();
        if ((stat != Os::File::OP_OK) ||
            !size) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        // Make sure buffer is null-terminated:
        buff[sizeof(buff)-1] = 0;
        const I64 walltimeDspLeadUs = strtoll(buff, NULL, 10);

        if (-walltimeDspLeadUs > usecDsp) {
            // TODO(mereweth) - EVR; can't have difference greater than time
            printf("linux-dsp diff %lld negative; greater than message time %lu\n",
                   walltimeDspLeadUs, usecDsp);
            return;
        }
        const I64 usecRos = usecDsp + walltimeDspLeadUs;
        msg.header.stamp.sec = (U32) (usecRos / 1000LL / 1000LL);
        msg.header.stamp.nsec = (usecRos % (1000LL * 1000LL)) * 1000LU;

        //TODO(mereweth) - END convert time instead using HLTimeConv

        stamp.set((U32) (usecRos / 1000LL / 1000LL),
                  (U32) (usecRos % (1000LL * 1000LL)));
        header.setstamp(stamp);
        Odometry.setheader(header);
#else
        msg.header.stamp.sec = stamp.getSeconds();
        msg.header.stamp.nsec = stamp.getUSeconds() * 1000LL;
#endif // ifdef DO_TIME_CONV

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

        msg.header.seq = header.getseq();

        // TODO(mereweth) - convert frame ID
        U32 frame_id = header.getframe_id();
        msg.header.frame_id = "world";

        msg.child_frame_id = "quest-base-link";

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

        ros::NodeHandle* n = compPtr->m_nodeHandle;
        FW_ASSERT(n);
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);

        ImuStateUpdateHandler updateHandler(compPtr, 0);

        ros::Subscriber updateSub = n->subscribe("imu_state_update", 10,
                                                &ImuStateUpdateHandler::imuStateUpdateCallback,
                                                &updateHandler,
                                                ros::TransportHints().tcpNoDelay());

        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
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

#ifdef DO_TIME_CONV
        //TODO(mereweth) - BEGIN convert time instead using HLTimeConv

        I64 usecRos = (I64) msg->header.stamp.sec * 1000LL * 1000LL
                      + (I64) msg->header.stamp.nsec / 1000LL;
        Os::File::Status stat = Os::File::OTHER_ERROR;
        Os::File file;
        stat = file.open("/sys/kernel/dsp_offset/walltime_dsp_diff", Os::File::OPEN_READ);
        if (stat != Os::File::OP_OK) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        char buff[255];
        NATIVE_INT_TYPE size = sizeof(buff);
        stat = file.read(buff, size, false);
        file.close();
        if ((stat != Os::File::OP_OK) ||
            !size) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        // Make sure buffer is null-terminated:
        buff[sizeof(buff)-1] = 0;
        I64 walltimeDspLeadUs = strtoll(buff, NULL, 10);

        if (walltimeDspLeadUs > usecRos) {
            // TODO(mereweth) - EVR; can't have difference greater than time
            printf("linux-dsp diff %lld greater than message time %lu\n",
                   walltimeDspLeadUs, usecRos);
            return;
        }
        I64 usecDsp = usecRos - walltimeDspLeadUs;
        Fw::Time convTime(TB_WORKSTATION_TIME,
                          0,
                          (U32) (usecDsp / 1000 / 1000),
                          (U32) (usecDsp % (1000 * 1000)));
#else
        Fw::Time convTime(TB_WORKSTATION_TIME,
                          0,
                          (U32) (msg->header.stamp.sec),
                          (U32) (msg->header.stamp.nsec / 1000));
#endif //DO_TIME_CONV
        //TODO(mereweth) - END convert time instead using HLTimeConv

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

} // end namespace
