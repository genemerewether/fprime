// ======================================================================
// \title  AckermannIfaceImpl.cpp
// \author mereweth
// \brief  cpp file for AckermannIface component implementation class
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


#include <Gnc/Utils/AckermannIface/AckermannIfaceComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

#include <Os/File.hpp>

#include <math.h>
#include <stdio.h>

#include <ros/callback_queue.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  AckermannIfaceComponentImpl ::
#if FW_OBJECT_NAMES == 1
    AckermannIfaceComponentImpl(
        const char *const compName
    ) :
      AckermannIfaceComponentBase(compName),
#else
    AckermannIfaceImpl(void),
#endif
    m_rosInited(false),
    m_tbDes(TB_NONE),
    m_nodeHandle(NULL),
    m_ackermannDriveStampedSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_ackermannDriveStampedSet); i++) {
          m_ackermannDriveStampedSet[i].fresh = false;
          m_ackermannDriveStampedSet[i].overflows = 0u;
      }
  }

    void AckermannIfaceComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      AckermannIfaceComponentBase::init(instance);
    }

    AckermannIfaceComponentImpl ::
      ~AckermannIfaceComponentImpl(void)
    {

    }
  
    void AckermannIfaceComponentImpl ::
      setTBDes(TimeBase tbDes) {
        this->m_tbDes = tbDes;
    }

    Os::Task::TaskStatus AckermannIfaceComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("FILTIFACE");
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, AckermannIfaceComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
    }
  
    void AckermannIfaceComponentImpl ::
      disableRos() {
        this->m_rosInited = false;
    }
  
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void AckermannIfaceComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_ackermannDriveStampedSet); i++) {
            m_ackermannDriveStampedSet[i].mutex.lock();
            if (m_ackermannDriveStampedSet[i].fresh) {
                if (this->isConnected_ackermannDriveStamped_OutputPort(i)) {
                    // mimics driver hardware getting and sending sensor data
                    this->ackermannDriveStamped_out(i, m_ackermannDriveStampedSet[i].ackermannDriveStamped);
                }
                else {
                    DEBUG_PRINT("IMU state update port %d not connected\n", i);
                }
                m_ackermannDriveStampedSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new odometry received?
            m_ackermannDriveStampedSet[i].mutex.unLock();
        }
    }

    void AckermannIfaceComponentImpl ::
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
    void AckermannIfaceComponentImpl ::
      intTaskEntry(void * ptr) {
        // TODO(mereweth) - prevent calling twice; free m_nodeHandle
        DEBUG_PRINT("AckermannIface task entry\n");

        FW_ASSERT(ptr);
        AckermannIfaceComponentImpl* compPtr = (AckermannIfaceComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_HLROSIFACE_IntTaskStarted();

        compPtr->m_nodeHandle = new ros::NodeHandle();
        ros::NodeHandle* n = compPtr->m_nodeHandle;
        FW_ASSERT(n);
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);

        if (ros::isShuttingDown()) {
            return;
        }

        AckermannDriveStampedHandler updateHandler(compPtr, 0);
        updateHandler.tbDes = compPtr->m_tbDes;

        ros::Subscriber updateSub = n->subscribe("ackermann_cmd", 10,
                                                &AckermannDriveStampedHandler::ackermannDriveStampedCallback,
                                                &updateHandler,
                                                ros::TransportHints().tcpNoDelay());

        compPtr->m_rosInited = true;
	
        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }
  
    AckermannIfaceComponentImpl :: TimeBaseHolder ::
      TimeBaseHolder() :
      tbDes(TB_NONE)
    {
    }
  
    AckermannIfaceComponentImpl :: AckermannDriveStampedHandler ::
      AckermannDriveStampedHandler(AckermannIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_ACKERMANNDRIVESTAMPED_OUTPUT_PORTS); //compPtr->getNum_AckermannDriveStamped_OutputPorts());
    }

    AckermannIfaceComponentImpl :: AckermannDriveStampedHandler :: ~AckermannDriveStampedHandler()
    {

    }

    void AckermannIfaceComponentImpl :: AckermannDriveStampedHandler ::
      ackermannDriveStampedCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_ACKERMANNDRIVESTAMPED_OUTPUT_PORTS);

        DEBUG_PRINT("ackermannDriveStamped port handler %d\n", this->portNum);

        if (!std::isfinite(msg->header.stamp.sec) ||
            !std::isfinite(msg->header.stamp.nsec)) {
            //TODO(mereweth) - EVR
            return;
        }

        if (!std::isfinite(msg->drive.steering_angle)          ||
            !std::isfinite(msg->drive.steering_angle_velocity) ||
            !std::isfinite(msg->drive.speed)                   ||
            !std::isfinite(msg->drive.acceleration)            ||
            !std::isfinite(msg->drive.jerk))                    {
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
                DEBUG_PRINT("failed to convert time in ackermannDriveStamped\n");
                return;
            }
        }
        
        {
            using namespace ROS::std_msgs;
            using namespace ROS::ackermann_msgs;

            AckermannDriveStamped ackermannDriveStamped(
              Header(msg->header.seq,
                     convTime,
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),
              AckermannDrive(msg->drive.steering_angle,
                             msg->drive.steering_angle_velocity,
                             msg->drive.speed,
                             msg->drive.acceleration,
                             msg->drive.jerk)
            ); // end AckermannDriveStamped constructor

            this->compPtr->m_ackermannDriveStampedSet[this->portNum].mutex.lock();
            if (this->compPtr->m_ackermannDriveStampedSet[this->portNum].fresh) {
                this->compPtr->m_ackermannDriveStampedSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting ackermannDriveStamped port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_ackermannDriveStampedSet[this->portNum].ackermannDriveStamped = ackermannDriveStamped;
            this->compPtr->m_ackermannDriveStampedSet[this->portNum].fresh = true;
        }
        this->compPtr->m_ackermannDriveStampedSet[this->portNum].mutex.unLock();
    }

} // end namespace
