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
      m_rgNH(NULL),
      m_odomSet() // zero-initialize instead of default-initializing
    {
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_odomSet); i++) {
            m_odomSet[i].fresh = false;
            m_odomSet[i].overflows = 0u;
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
        ros::NodeHandle n;
        m_rgNH = &n;

        char buf[32];
        m_motorPub = m_rgNH->advertise<mav_msgs::Actuators>("command/motor_speed", 5);
    }

    Os::Task::TaskStatus RotorSDrvComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("ROTORSDRV");
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
        mav_msgs::Actuators msg;
        int size = 0;
        const F64 *angVel = actuator.getangular_velocities(size);
        //TODO(mgardine) - Work out what is going on here
	//        size = 6;
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
            // TODO(mereweth) check context == odometry out context
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

            // TODO(mereweth) check context == imu out context
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

        ros::NodeHandle n;
        ros::CallbackQueue localCallbacks;
        n.setCallbackQueue(&localCallbacks);

        OdometryHandler gtHandler(compPtr, 0);
        OdometryHandler odomHandler(compPtr, 1);

        ros::Subscriber gtSub = n.subscribe("ground_truth/odometry", 1000,
                                            &OdometryHandler::odometryCallback,
                                            &gtHandler,
                                            ros::TransportHints().tcpNoDelay());

        ros::Subscriber odomSub = n.subscribe("odometry_sensor1/odometry", 1000,
                                              &OdometryHandler::odometryCallback,
                                              &odomHandler,
                                              ros::TransportHints().tcpNoDelay());

        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
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

} // end namespace SIMREF
