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
      m_imuSet() // zero-initialize instead of default-initializing
    {
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuSet); i++) {
            m_imuSet[i].fresh = false;
            m_imuSet[i].overflows = 0u;
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

    Os::Task::TaskStatus RotorSDrvComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("ROTORSDRVROS");
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
        if (context == RSDRV_SCHED_CONTEXT_OP) {
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
        }
        else if (context == RSDRV_SCHED_CONTEXT_TLM) {
            FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(m_imuSet) >= 1,
                      FW_NUM_ARRAY_ELEMENTS(m_imuSet));
            this->tlmWrite_RSDRV_Imu1Overflows(m_imuSet[0].overflows);
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

	compPtr->m_nodeHandle = new ros::NodeHandle();
        ros::NodeHandle* n = compPtr->m_nodeHandle;
	FW_ASSERT(n);
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);

        char buf[32];
        compPtr->m_motorPub = n->advertise<mav_msgs::Actuators>("command/motor_speed", 5);

        ImuHandler imuHandler(compPtr, 0);
        ros::Subscriber imuSub = n->subscribe("imu", 10,
                                              &ImuHandler::imuCallback,
                                              &imuHandler,
                                              ros::TransportHints().tcpNoDelay());

        compPtr->m_rosInited = true;
	
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

} // end namespace SIMREF
