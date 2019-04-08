// ======================================================================
// \title  RotorSDrvImpl.cpp
// \author mereweth
// \brief  cpp file for RotorSDrv component implementation class
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


#include <SIMREF/RotorSDrv/RotorSDrvComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

#include <stdio.h>

#include <ros/callback_queue.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace SIMREF {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    RotorSDrvComponentImpl ::
  #if FW_OBJECT_NAMES == 1
      RotorSDrvComponentImpl(
          const char *const compName
      ) :
        RotorSDrvComponentBase(compName),
  #else
      RotorSDrvImpl(void),
  #endif
      m_rosInited(false),
      m_nodeHandle(NULL),
      m_imuSet(), // zero-initialize instead of default-initializing
      m_imuStateUpdateSet(), // zero-initialize instead of default-initializing
      m_odomSet(), // zero-initialize instead of default-initializing
      m_flatOutSet(), // zero-initialize instead of default-initializing
      m_attRateThrustSet() // zero-initialize instead of default-initializing
    {
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuSet); i++) {
            m_imuSet[i].fresh = false;
            m_imuSet[i].overflows = 0u;
        }
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
            m_imuStateUpdateSet[i].fresh = false;
            m_imuStateUpdateSet[i].overflows = 0u;
        }
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_odomSet); i++) {
            m_odomSet[i].fresh = false;
            m_odomSet[i].overflows = 0u;
        }
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_flatOutSet); i++) {
            m_flatOutSet[i].fresh = false;
            m_flatOutSet[i].overflows = 0u;
        }
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_attRateThrustSet); i++) {
            m_attRateThrustSet[i].fresh = false;
            m_attRateThrustSet[i].overflows = 0u;
        }
    }

    void RotorSDrvComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      RotorSDrvComponentBase::init(instance);
    }

    RotorSDrvComponentImpl ::
      ~RotorSDrvComponentImpl(void)
    {

    }

    void RotorSDrvComponentImpl ::
      startPub() {
        FW_ASSERT(m_nodeHandle);
        ros::NodeHandle* n = this->m_nodeHandle;
	FW_ASSERT(n);

        char buf[32];
        m_motorPub = n->advertise<mav_msgs::Actuators>("command/motor_speed", 5);

        for (int i = 0; i < NUM_ODOMLOG_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "odometry_%d", i);
            m_odomPub[i] = n->advertise<nav_msgs::Odometry>(buf, 5);
        }

        m_rosInited = true;
    }

    Os::Task::TaskStatus RotorSDrvComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("ROTORSDRVROS");
	this->m_nodeHandle = new ros::NodeHandle();
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, RotorSDrvComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void RotorSDrvComponentImpl ::
      OdomLog_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::nav_msgs::OdometryNoCov &Odometry
      )
    {
        if (this->isConnected_FileLogger_OutputPort(0)) {
            Svc::ActiveFileLoggerPacket fileBuff;
            Fw::SerializeStatus stat;
            Fw::Time recvTime = this->getTime();
            fileBuff.resetSer();
            stat = fileBuff.serialize((U8)AFL_HLROSIFACE_ODOMNOCOV);
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

        nav_msgs::Odometry msg;

        ROS::std_msgs::Header header = Odometry.getheader();
        Fw::Time stamp = header.getstamp();
        msg.header.stamp = ros::Time::now();

        msg.header.seq = header.getseq();

        // TODO(mereweth) - convert frame ID
        U32 frame_id = header.getframe_id();
        msg.header.frame_id = "world";

        msg.child_frame_id = "odom";

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

    void RotorSDrvComponentImpl ::
      motor_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::mav_msgs::Actuators &actuator
      )
    {
        if (!m_rosInited) {
            return;
        }

        mav_msgs::Actuators msg;
        int size = 0;
        const F64 *angVel = actuator.getangular_velocities(size);
        size = FW_MIN(size, actuator.getangular_velocities_count());
        msg.angular_velocities.resize(size);
        for (int i = 0; i < size; i++) {
            msg.angular_velocities[i] = angVel[i];
        }
        m_motorPub.publish(msg);
    }

    void RotorSDrvComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        if ((context == RSDRV_SCHED_CONTEXT_ATT) ||
            (context == RSDRV_SCHED_CONTEXT_POS)) {
            for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuSet); i++) {
                m_imuSet[i].mutex.lock();
                if (m_imuSet[i].fresh) {
                    if (this->isConnected_SimImu_OutputPort(i)) {
                        // mimics driver hardware getting and sending sensor data
                        this->SimImu_out(i, m_imuSet[i].imu);
                    }
                    else {
                        DEBUG_PRINT("Imu port %d not connected\n", i);
                    }
                    m_imuSet[i].fresh = false;
                }
                // TODO(mereweth) - notify that no new imu received?
                m_imuSet[i].mutex.unLock();
            }

            for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
                m_imuStateUpdateSet[i].mutex.lock();
                if (m_imuStateUpdateSet[i].fresh) {
                    if (this->isConnected_ImuStateUpdate_OutputPort(i)) {
                        // mimics driver hardware getting and sending sensor data
                        this->ImuStateUpdate_out(i, m_imuStateUpdateSet[i].imuStateUpdate);
                    }
                    else {
                        DEBUG_PRINT("Imu state update port %d not connected\n", i);
                    }
                    m_imuStateUpdateSet[i].fresh = false;
                }
                // TODO(mereweth) - notify that no new imu received?
                m_imuStateUpdateSet[i].mutex.unLock();
            }

            for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_odomSet); i++) {
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
            }

            for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_flatOutSet); i++) {
                m_flatOutSet[i].mutex.lock();
                if (m_flatOutSet[i].fresh) {
                    if (this->isConnected_flatOutput_OutputPort(i)) {
                        // mimics driver hardware getting and sending sensor data
                        this->flatOutput_out(i, m_flatOutSet[i].flatOutput);
                    }
                    else {
                        DEBUG_PRINT("Flat output port %d not connected\n", i);
                    }
                    m_flatOutSet[i].fresh = false;
                }
                // TODO(mereweth) - notify that no new odometry received?
                m_flatOutSet[i].mutex.unLock();
            }

            for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_attRateThrustSet); i++) {
                m_attRateThrustSet[i].mutex.lock();
                if (m_attRateThrustSet[i].fresh) {
                    if (this->isConnected_attRateThrust_OutputPort(i)) {
                        // mimics driver hardware getting and sending sensor data
                        this->attRateThrust_out(i, m_attRateThrustSet[i].attRateThrust);
                    }
                    else {
                        DEBUG_PRINT("Attitude rate thrust port %d not connected\n", i);
                    }
                    m_attRateThrustSet[i].fresh = false;
                }
                // TODO(mereweth) - notify that no new odometry received?
                m_attRateThrustSet[i].mutex.unLock();
            }
        }
        else if (context == RSDRV_SCHED_CONTEXT_TLM) {
            FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(m_odomSet) >= 2,
                      FW_NUM_ARRAY_ELEMENTS(m_odomSet));
            this->tlmWrite_RSDRV_Odometry1Overflows(m_odomSet[0].overflows);
            this->tlmWrite_RSDRV_Odometry2Overflows(m_odomSet[1].overflows);
        }
    }

    void RotorSDrvComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        // TODO(mereweth) - check that message-wait task is OK
        this->pingOut_out(portNum, key);
    }

    // ----------------------------------------------------------------------
    // Member function definitions
    // ----------------------------------------------------------------------

    //! Entry point for task waiting for messages
    void RotorSDrvComponentImpl ::
      intTaskEntry(void * ptr) {
        DEBUG_PRINT("RotorSDrv task entry\n");

        FW_ASSERT(ptr);
        RotorSDrvComponentImpl* compPtr = (RotorSDrvComponentImpl*) ptr;
        compPtr->log_ACTIVITY_LO_RSDRV_IntTaskStarted();

        ros::NodeHandle* n = compPtr->m_nodeHandle;
	FW_ASSERT(n);
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);

        ImuStateUpdateHandler updateHandler(compPtr, 0);
        ros::Subscriber updateSub = n->subscribe("imu_state_update", 10,
                                                &ImuStateUpdateHandler::imuStateUpdateCallback,
                                                &updateHandler,
                                                ros::TransportHints().tcpNoDelay());

        OdometryHandler gtHandler(compPtr, 0);
        OdometryHandler odomHandler(compPtr, 1);
        ImuHandler imuHandler(compPtr, 0);
        FlatOutputHandler flatoutHandler(compPtr, 0);
        AttitudeRateThrustHandler attRateThrustHandler(compPtr, 0);

        ros::Subscriber gtSub = n->subscribe("ground_truth/odometry", 10,
                                            &OdometryHandler::odometryCallback,
                                            &gtHandler,
                                            ros::TransportHints().tcpNoDelay());

        ros::Subscriber odomSub = n->subscribe("odometry_sensor1/odometry", 10,
                                              &OdometryHandler::odometryCallback,
                                              &odomHandler,
                                              ros::TransportHints().tcpNoDelay());

        ros::Subscriber imuSub = n->subscribe("imu", 10,
                                              &ImuHandler::imuCallback,
                                              &imuHandler,
                                              ros::TransportHints().tcpNoDelay());

        // TODO(mgardine) - what should the queue size be?
        ros::Subscriber flatoutSub = n->subscribe("flat_output_setpoint", 1,
                                                 &FlatOutputHandler::flatOutputCallback,
                                                 &flatoutHandler,
                                                 ros::TransportHints().tcpNoDelay());

        ros::Subscriber attRateThrustSub = n->subscribe("attitude_rate_thrust_setpoint", 10,
                                                       &AttitudeRateThrustHandler::attitudeRateThrustCallback,
                                                       &attRateThrustHandler,
                                                       ros::TransportHints().tcpNoDelay());

        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }

    RotorSDrvComponentImpl :: ImuHandler ::
      ImuHandler(RotorSDrvComponentImpl* compPtr,
                 int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_SIMIMU_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    RotorSDrvComponentImpl :: ImuHandler :: ~ImuHandler()
    {

    }

    void RotorSDrvComponentImpl :: ImuHandler ::
      imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_SIMIMU_OUTPUT_PORTS);

        DEBUG_PRINT("Odom port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::sensor_msgs;
            using namespace ROS::geometry_msgs;
            ImuNoCov imu(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
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
            this->compPtr->m_imuSet[this->portNum].mutex.unLock();
        }
    }

    RotorSDrvComponentImpl :: OdometryHandler ::
      OdometryHandler(RotorSDrvComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_ODOMETRY_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    RotorSDrvComponentImpl :: OdometryHandler :: ~OdometryHandler()
    {

    }

    void RotorSDrvComponentImpl :: OdometryHandler ::
      odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_ODOMETRY_OUTPUT_PORTS);

        DEBUG_PRINT("Odom port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::nav_msgs;
            using namespace ROS::geometry_msgs;
            Odometry odom(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),
              0/*Fw::EightyCharString(msg->child_frame_id.data())*/,
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

            this->compPtr->m_odomSet[this->portNum].mutex.lock();
            if (this->compPtr->m_odomSet[this->portNum].fresh) {
                this->compPtr->m_odomSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting odom port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_odomSet[this->portNum].odom = odom;
            this->compPtr->m_odomSet[this->portNum].fresh = true;
            this->compPtr->m_odomSet[this->portNum].mutex.unLock();
        }
    }

    RotorSDrvComponentImpl :: ImuStateUpdateHandler ::
      ImuStateUpdateHandler(RotorSDrvComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS); //compPtr->getNum_ImuStateUpdate_OutputPorts());
    }

    RotorSDrvComponentImpl :: ImuStateUpdateHandler :: ~ImuStateUpdateHandler()
    {

    }

    void RotorSDrvComponentImpl :: ImuStateUpdateHandler ::
      imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS);

        DEBUG_PRINT("imuStateUpdate port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            ImuStateUpdateNoCov imuStateUpdate(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
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
            this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.unLock();
        }
    }

    // Flat output constructor/destructor/callback
    RotorSDrvComponentImpl :: FlatOutputHandler ::
      FlatOutputHandler(RotorSDrvComponentImpl* compPtr,
                        int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_FLATOUTPUT_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    RotorSDrvComponentImpl :: FlatOutputHandler :: ~FlatOutputHandler()
    {

    }

    void RotorSDrvComponentImpl :: FlatOutputHandler ::
      flatOutputCallback(const mav_msgs::FlatOutput::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_FLATOUTPUT_OUTPUT_PORTS);

        DEBUG_PRINT("Flat output port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            FlatOutput flatOutput(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),

              Point(msg->position.x, msg->position.y, msg->position.z),
              Vector3(msg->velocity.x, msg->velocity.y, msg->velocity.z),
              Vector3(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z),
              F64(msg->yaw)

            ); // end FlatOutput constructor

            this->compPtr->m_flatOutSet[this->portNum].mutex.lock();
            if (this->compPtr->m_flatOutSet[this->portNum].fresh) {
                this->compPtr->m_flatOutSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting flatout port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_flatOutSet[this->portNum].flatOutput = flatOutput;
            this->compPtr->m_flatOutSet[this->portNum].fresh = true;
            this->compPtr->m_flatOutSet[this->portNum].mutex.unLock();
        }
    }

    // AttitudeRateThrust constructor/destructor/callback
    RotorSDrvComponentImpl :: AttitudeRateThrustHandler ::
      AttitudeRateThrustHandler(RotorSDrvComponentImpl* compPtr,
                                int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_ATTRATETHRUST_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    RotorSDrvComponentImpl :: AttitudeRateThrustHandler :: ~AttitudeRateThrustHandler()
    {

    }

    void RotorSDrvComponentImpl :: AttitudeRateThrustHandler ::
      attitudeRateThrustCallback(const mav_msgs::AttitudeRateThrust::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_ATTRATETHRUST_OUTPUT_PORTS);

        DEBUG_PRINT("Attitude rate thrust output port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            AttitudeRateThrust attRateThrust(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),

              Quaternion(msg->attitude.x, msg->attitude.y, msg->attitude.z, msg->attitude.w),
              Vector3(msg->angular_rates.x, msg->angular_rates.y, msg->angular_rates.z),
              Vector3(msg->thrust.x, msg->thrust.y, msg->thrust.z)

            ); // end AttitudeRateThrust constructor

            this->compPtr->m_attRateThrustSet[this->portNum].mutex.lock();
            if (this->compPtr->m_attRateThrustSet[this->portNum].fresh) {
                this->compPtr->m_attRateThrustSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting attitude rate thrust port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_attRateThrustSet[this->portNum].attRateThrust = attRateThrust;
            this->compPtr->m_attRateThrustSet[this->portNum].fresh = true;
            this->compPtr->m_attRateThrustSet[this->portNum].mutex.unLock();
        }
    }

} // end namespace SIMREF
