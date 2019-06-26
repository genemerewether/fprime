// ======================================================================
// \title  MultirotorCtrlIfaceImpl.cpp
// \author mereweth
// \brief  cpp file for MultirotorCtrlIface component implementation class
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


#include <Gnc/Ctrl/MultirotorCtrlIface/MultirotorCtrlIfaceComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

#include <stdio.h>

#include <ros/callback_queue.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  MultirotorCtrlIfaceComponentImpl ::
#if FW_OBJECT_NAMES == 1
    MultirotorCtrlIfaceComponentImpl(
        const char *const compName
    ) :
      MultirotorCtrlIfaceComponentBase(compName),
#else
    MultirotorCtrlIfaceImpl(void),
#endif
    m_rosInited(false),
    m_tbDes(TB_NONE),
    m_nodeHandle(NULL),
    m_boolStampedSet(), // zero-initialize instead of default-initializing
    m_flatOutSet(), // zero-initialize instead of default-initializing
    m_attRateThrustSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_boolStampedSet); i++) {
          m_boolStampedSet[i].fresh = false;
          m_boolStampedSet[i].overflows = 0u;
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

    void MultirotorCtrlIfaceComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      MultirotorCtrlIfaceComponentBase::init(instance);
    }

    MultirotorCtrlIfaceComponentImpl ::
      ~MultirotorCtrlIfaceComponentImpl(void)
    {

    }

    void MultirotorCtrlIfaceComponentImpl ::
      startPub() {
        // TODO(mereweth) - prevent calling twice
        FW_ASSERT(m_nodeHandle);
        ros::NodeHandle* n = this->m_nodeHandle;

        m_rosInited = true;
    }

    void MultirotorCtrlIfaceComponentImpl ::
      setTBDes(TimeBase tbDes) {
        this->m_tbDes = tbDes;
    }

    Os::Task::TaskStatus MultirotorCtrlIfaceComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("MRCTRLIFACE");
        this->m_nodeHandle = new ros::NodeHandle();
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, MultirotorCtrlIfaceComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
    }
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void MultirotorCtrlIfaceComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_boolStampedSet); i++) {
            m_boolStampedSet[i].mutex.lock();
            if (m_boolStampedSet[i].fresh) {
                if (this->isConnected_boolStamped_OutputPort(i)) {
                    // mimics driver hardware getting and sending sensor data
                    this->boolStamped_out(i, m_boolStampedSet[i].boolStamped);
                }
                else {
                    DEBUG_PRINT("Bool stamped port %d not connected\n", i);
                }
                m_boolStampedSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new msg received?
            m_boolStampedSet[i].mutex.unLock();
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

    void MultirotorCtrlIfaceComponentImpl ::
      AccelCommand_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::geometry_msgs::AccelStamped &AccelStamped
      )
    {
        ROS::std_msgs::Header header = AccelStamped.getheader();
        Fw::Time stamp = header.getstamp();

        // if port is not connected, default to no conversion
        Fw::Time convTime = stamp;

        if (this->isConnected_convertTime_OutputPort(0)) {
            bool success = false;
            convTime = this->convertTime_out(0, stamp, TB_ROS_TIME, 0, success);
            if (!success) {
                // TODO(Mereweth) - EVR
                return;
            }
        }

        header.setstamp(convTime);
        AccelStamped.setheader(header);

        if (this->isConnected_FileLogger_OutputPort(0)) {
            Svc::ActiveFileLoggerPacket fileBuff;
            Fw::SerializeStatus stat;
            Fw::Time recvTime = this->getTime();
            fileBuff.resetSer();
            stat = fileBuff.serialize((U8)AFL_MRCTRLIFACE_ACCEL_CMD);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getUSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = AccelStamped.serialize(fileBuff);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);

            this->FileLogger_out(0,fileBuff);
        }
    }

    void MultirotorCtrlIfaceComponentImpl ::
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
    void MultirotorCtrlIfaceComponentImpl ::
      intTaskEntry(void * ptr) {
        // TODO(mereweth) - prevent calling twice; free m_nodeHandle
        DEBUG_PRINT("MultirotorCtrlIface task entry\n");

        FW_ASSERT(ptr);
        MultirotorCtrlIfaceComponentImpl* compPtr = (MultirotorCtrlIfaceComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_HLROSIFACE_IntTaskStarted();

        ros::NodeHandle* n = compPtr->m_nodeHandle;
          FW_ASSERT(n);
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);

        BoolStampedHandler boolStampedHandler(compPtr, 0);
        boolStampedHandler.tbDes = compPtr->m_tbDes;
        FlatOutputHandler flatoutHandler(compPtr, 0);
        flatoutHandler.tbDes = compPtr->m_tbDes;
        AttitudeRateThrustHandler attRateThrustHandler(compPtr, 0);
        attRateThrustHandler.tbDes = compPtr->m_tbDes;

        ros::Subscriber flatoutSub = n->subscribe("flat_output_setpoint", 1,
                                                  &FlatOutputHandler::flatOutputCallback,
                                                  &flatoutHandler,
                                                  ros::TransportHints().tcpNoDelay());

        ros::Subscriber boolStampedSub = n->subscribe("flysafe", 1,
                                                      &BoolStampedHandler::boolStampedCallback,
                                                      &boolStampedHandler,
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

    MultirotorCtrlIfaceComponentImpl :: TimeBaseHolder ::
      TimeBaseHolder() :
      tbDes(TB_NONE)
    {
    }
  
    // Bool stamped constructor/destructor/callback
    MultirotorCtrlIfaceComponentImpl :: BoolStampedHandler ::
      BoolStampedHandler(MultirotorCtrlIfaceComponentImpl* compPtr,
                         int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_BOOLSTAMPED_OUTPUT_PORTS);
    }

    MultirotorCtrlIfaceComponentImpl :: BoolStampedHandler :: ~BoolStampedHandler()
    {

    }

    void MultirotorCtrlIfaceComponentImpl :: BoolStampedHandler ::
      boolStampedCallback(const mav_msgs::BoolStamped::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_BOOLSTAMPED_OUTPUT_PORTS);

        DEBUG_PRINT("bool stamped port handler %d\n", this->portNum);

        if (!std::isfinite(msg->header.stamp.sec) ||
            !std::isfinite(msg->header.stamp.nsec)) {
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
                // TODO(Mereweth) - EVR
                return;
            }
        }

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            BoolStamped boolStamped(
              Header(msg->header.seq,
                     convTime,
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),

              Bool(msg->data.data)

            ); // end BoolStamped constructor

            this->compPtr->m_boolStampedSet[this->portNum].mutex.lock();
            if (this->compPtr->m_boolStampedSet[this->portNum].fresh) {
                this->compPtr->m_boolStampedSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting flatout port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_boolStampedSet[this->portNum].boolStamped = boolStamped;
            this->compPtr->m_boolStampedSet[this->portNum].fresh = true;
            this->compPtr->m_boolStampedSet[this->portNum].mutex.unLock();
        }
    }

    // Flat output constructor/destructor/callback
    MultirotorCtrlIfaceComponentImpl :: FlatOutputHandler ::
      FlatOutputHandler(MultirotorCtrlIfaceComponentImpl* compPtr,
                        int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_FLATOUTPUT_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    MultirotorCtrlIfaceComponentImpl :: FlatOutputHandler :: ~FlatOutputHandler()
    {

    }

    void MultirotorCtrlIfaceComponentImpl :: FlatOutputHandler ::
      flatOutputCallback(const mav_msgs::FlatOutput::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_FLATOUTPUT_OUTPUT_PORTS);

        DEBUG_PRINT("Flat output port handler %d\n", this->portNum);

        if (!std::isfinite(msg->header.stamp.sec) ||
            !std::isfinite(msg->header.stamp.nsec)) {
            //TODO(mereweth) - EVR
            return;
        }

        if (!std::isfinite(msg->position.x) ||
            !std::isfinite(msg->position.y) ||
            !std::isfinite(msg->position.z) ||

            !std::isfinite(msg->velocity.x) ||
            !std::isfinite(msg->velocity.y) ||
            !std::isfinite(msg->velocity.z) ||

            !std::isfinite(msg->acceleration.x) ||
            !std::isfinite(msg->acceleration.y) ||
            !std::isfinite(msg->acceleration.z) ||

            !std::isfinite(msg->yaw)) {
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
                // TODO(Mereweth) - EVR
                return;
            }
        }
        
        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            FlatOutput flatOutput(
              Header(msg->header.seq,
                     convTime,
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
    MultirotorCtrlIfaceComponentImpl :: AttitudeRateThrustHandler ::
      AttitudeRateThrustHandler(MultirotorCtrlIfaceComponentImpl* compPtr,
                                int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_ATTRATETHRUST_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    MultirotorCtrlIfaceComponentImpl :: AttitudeRateThrustHandler :: ~AttitudeRateThrustHandler()
    {

    }

    void MultirotorCtrlIfaceComponentImpl :: AttitudeRateThrustHandler ::
      attitudeRateThrustCallback(const mav_msgs::AttitudeRateThrust::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_ATTRATETHRUST_OUTPUT_PORTS);

        DEBUG_PRINT("Attitude rate thrust output port handler %d\n", this->portNum);

        if (!std::isfinite(msg->header.stamp.sec) ||
            !std::isfinite(msg->header.stamp.nsec)) {
            //TODO(mereweth) - EVR
            return;
        }

        if (!std::isfinite(msg->attitude.x) ||
            !std::isfinite(msg->attitude.y) ||
            !std::isfinite(msg->attitude.z) ||
            !std::isfinite(msg->attitude.w) ||

            !std::isfinite(msg->angular_rates.x) ||
            !std::isfinite(msg->angular_rates.y) ||
            !std::isfinite(msg->angular_rates.z) ||

            !std::isfinite(msg->angular_acceleration.x) ||
            !std::isfinite(msg->angular_acceleration.y) ||
            !std::isfinite(msg->angular_acceleration.z) ||

            !std::isfinite(msg->thrust.x) ||
            !std::isfinite(msg->thrust.y) ||
            !std::isfinite(msg->thrust.z)) {
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
                // TODO(Mereweth) - EVR
                return;
            }
        }
        
        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            AttitudeRateThrust attRateThrust(
              Header(msg->header.seq,
                     convTime,
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),

              Quaternion(msg->attitude.x, msg->attitude.y, msg->attitude.z, msg->attitude.w),
              Vector3(msg->angular_rates.x, msg->angular_rates.y, msg->angular_rates.z),
              Vector3(msg->angular_acceleration.x, msg->angular_acceleration.y, msg->angular_acceleration.z),
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

} // end namespace
