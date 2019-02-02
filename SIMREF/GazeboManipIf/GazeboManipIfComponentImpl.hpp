// ====================================================================== 
// \title  GazeboManipIfImpl.hpp
// \author mereweth
// \brief  hpp file for GazeboManipIf component implementation class
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

#ifndef GazeboManipIf_HPP
#define GazeboManipIf_HPP

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>

#include "SIMREF/GazeboManipIf/GazeboManipIfComponentAc.hpp"

namespace SIMREF {

  class GazeboManipIfComponentImpl :
    public GazeboManipIfComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object GazeboManipIf
      //!
      GazeboManipIfComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object GazeboManipIf
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object GazeboManipIf
      //!
      ~GazeboManipIfComponentImpl(void);

      //! Initialize publishers
      //!
      void startPub(void);

      //! Start interrupt task
      Os::Task::TaskStatus startIntTask(NATIVE_INT_TYPE priority,
                                        NATIVE_INT_TYPE stackSize,
                                        NATIVE_INT_TYPE cpuAffinity = -1);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for JointTrajGoal
      //!
      void JointTrajGoal_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::control_msgs::FollowJointTrajectoryGoal &FollowJointTrajectoryGoal 
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for pingIn
      //!
      void pingIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      // ----------------------------------------------------------------------
      // ROS callbacks
      // ----------------------------------------------------------------------

      void doneCb(const actionlib::SimpleClientGoalState& state,
                  const control_msgs::FollowJointTrajectoryResultConstPtr& result);

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------

        //! Entry point for task waiting for interrupt
        static void intTaskEntry(void * ptr);

        //! Task object for RTI task
        //!
        Os::Task m_intTask;

        bool m_rosInited;

        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>* m_client;

    };

} // end namespace SIMREF

#endif
