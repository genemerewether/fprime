// ====================================================================== 
// \title  GazeboManipIfImpl.cpp
// \author mereweth
// \brief  cpp file for GazeboManipIf component implementation class
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


#include <SIMREF/GazeboManipIf/GazeboManipIfComponentImpl.hpp>
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

    GazeboManipIfComponentImpl ::
  #if FW_OBJECT_NAMES == 1
      GazeboManipIfComponentImpl(
          const char *const compName
      ) :
        GazeboManipIfComponentBase(compName),
  #else
        GazeboManipIfImpl(void),
  #endif
        m_rosInited(false),
        m_client(NULL)
    {

    }

    void GazeboManipIfComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      ) 
    {
      GazeboManipIfComponentBase::init(instance);
    }

    GazeboManipIfComponentImpl ::
      ~GazeboManipIfComponentImpl(void)
    {
        if (m_client) {
            delete m_client;
        }
    }

    void GazeboManipIfComponentImpl ::
      startPub() {
        ros::NodeHandle n;

        m_rosInited = true;
    }

    Os::Task::TaskStatus GazeboManipIfComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("GZMANIPIF");
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, GazeboManipIfComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void GazeboManipIfComponentImpl ::
      JointTrajGoal_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::control_msgs::FollowJointTrajectoryGoal &FollowJointTrajectoryGoal
      )
    {
        if (!m_rosInited) {
            return;
        }

        FW_ASSERT(m_client);

        // TODO(mereweth) - convert; add feedback and active callbacks
        control_msgs::FollowJointTrajectoryGoal goal;
        m_client->sendGoal(goal,
                           boost::bind(&GazeboManipIfComponentImpl::doneCb, this, _1, _2));
    }

    void GazeboManipIfComponentImpl ::
      doneCb(const actionlib::SimpleClientGoalState& state,
             const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {
        ROS::control_msgs::FollowJointTrajectoryResult res;
        this->JointTrajResult_out(0, res);
    }

    void GazeboManipIfComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
      // TODO
    }

    void GazeboManipIfComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        this->pingOut_out(0, key);
    }

    // ----------------------------------------------------------------------
    // Member function definitions
    // ----------------------------------------------------------------------

    //! Entry point for task waiting for messages
    void GazeboManipIfComponentImpl ::
      intTaskEntry(void * ptr) {
        DEBUG_PRINT("GazeboManipIf task entry\n");

        FW_ASSERT(ptr);
        GazeboManipIfComponentImpl* compPtr = (GazeboManipIfComponentImpl*) ptr;
        compPtr->log_ACTIVITY_LO_GZMANIPIF_IntTaskStarted();

        ros::NodeHandle n;
        ros::CallbackQueue localCallbacks;
        n.setCallbackQueue(&localCallbacks);

        compPtr->m_client = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>("arm_controller/follow_joint_trajectory");
        FW_ASSERT(compPtr->m_client);

        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }
} // end namespace SIMREF
