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
        Os::TaskString name("GZMANIPIFROS");
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

        // TODO(mereweth) - add feedback and active callbacks

        control_msgs::FollowJointTrajectoryGoal msg;
        const ROS::trajectory_msgs::JointTrajectory traj = FollowJointTrajectoryGoal.gettrajectory();
        ROS::std_msgs::Header header = traj.getheader();
        msg.trajectory.header.stamp.sec = header.getstamp().getSeconds();
        msg.trajectory.header.stamp.nsec = header.getstamp().getUSeconds() * 1000LU;
        msg.trajectory.header.seq = header.getseq();
        // TODO(mereweth) - convert frame ID
        msg.trajectory.header.frame_id = "base_link";
        NATIVE_INT_TYPE numNames = 0;
        const Fw::EightyCharString* names = traj.getjoint_names(numNames);
        numNames = FW_MIN(numNames, traj.getjoint_names_count());
        for (U32 i = 0; i < numNames; i++) {
            msg.trajectory.joint_names.push_back(names[i].toChar());
        }
        NATIVE_INT_TYPE numPoints = 0;
        const ROS::trajectory_msgs::JointTrajectoryPoint* point = traj.getpoints(numPoints);
        numPoints = FW_MIN(numPoints, traj.getpoints_count());
        for (U32 i = 0; i < numPoints; i++) {
            trajectory_msgs::JointTrajectoryPoint jPoint;
            NATIVE_INT_TYPE numVals = 0;
            const F64* val = point[i].getpositions(numVals);
            numVals = FW_MIN(numVals, point[i].getpositions_count());
            for (U32 i = 0; i < numVals; i++) {
                jPoint.positions.push_back(val[i]);
            }
            numVals = 0;
            val = point[i].getvelocities(numVals);
            numVals = FW_MIN(numVals, point[i].getvelocities_count());
            for (U32 i = 0; i < numVals; i++) {
                jPoint.velocities.push_back(val[i]);
            }
            numVals = 0;
            val = point[i].getaccelerations(numVals);
            numVals = FW_MIN(numVals, point[i].getaccelerations_count());
            for (U32 i = 0; i < numVals; i++) {
                jPoint.accelerations.push_back(val[i]);
            }
            numVals = 0;
            val = point[i].geteffort(numVals);
            numVals = FW_MIN(numVals, point[i].geteffort_count());
            for (U32 i = 0; i < numVals; i++) {
                jPoint.effort.push_back(val[i]);
            }

            jPoint.time_from_start.sec = point[i].gettime_from_start().getSeconds();
            jPoint.time_from_start.nsec = point[i].gettime_from_start().getUSeconds() * 1000LU;

            msg.trajectory.points.push_back(jPoint);
        }

        NATIVE_INT_TYPE numTol = 0;
        const ROS::control_msgs::JointTolerance* tol = FollowJointTrajectoryGoal.getpath_tolerance(numTol);
        numTol = FW_MIN(numTol,
                        FollowJointTrajectoryGoal.getpath_tolerance_count());
        for (U32 i = 0; i < numTol; i++) {
            control_msgs::JointTolerance jTol;
            jTol.name = tol[i].getname().toChar();
            jTol.position = tol[i].getposition();
            jTol.velocity = tol[i].getvelocity();
            jTol.acceleration = tol[i].getacceleration();
            msg.path_tolerance.push_back(jTol);
        }
          
        numTol = 0;
        tol = FollowJointTrajectoryGoal.getgoal_tolerance(numTol);
        numTol = FW_MIN(numTol,
                        FollowJointTrajectoryGoal.getgoal_tolerance_count());
        for (U32 i = 0; i < numTol; i++) {
            control_msgs::JointTolerance jTol;
            jTol.name = tol[i].getname().toChar();
            jTol.position = tol[i].getposition();
            jTol.velocity = tol[i].getvelocity();
            jTol.acceleration = tol[i].getacceleration();
            msg.goal_tolerance.push_back(jTol);
        }

        Fw::Time goalTol = FollowJointTrajectoryGoal.getgoal_time_tolerance();
        msg.goal_time_tolerance.sec = goalTol.getSeconds();
        msg.goal_time_tolerance.nsec = goalTol.getUSeconds() * 1000LU;

        m_client->sendGoal(msg,
                           boost::bind(&GazeboManipIfComponentImpl::doneCb, this, _1, _2));
    }

    void GazeboManipIfComponentImpl ::
      doneCb(const actionlib::SimpleClientGoalState& state,
             const control_msgs::FollowJointTrajectoryResultConstPtr& result)
    {
        if (this->isConnected_JointTrajFeedback_OutputPort(0)) {
            ROS::control_msgs::FollowJointTrajectoryResult res;
            this->JointTrajResult_out(0, res);
        }
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
  // Command handler implementations 
  // ----------------------------------------------------------------------

    void GazeboManipIfComponentImpl ::
      GZMANIPIF_TestMove_cmdHandler(
          const FwOpcodeType opCode,
          const U32 cmdSeq,
          F64 j1_pos,
          //const Fw::CmdStringArg& j1_name,
          F64 j2_pos,
          //const Fw::CmdStringArg& j2_name,
          F64 j3_pos,
          //const Fw::CmdStringArg& j3_name,
          F64 j4_pos,
          //const Fw::CmdStringArg& j4_name,
          F64 j5_pos,
          //const Fw::CmdStringArg& j5_name,
          F64 j6_pos,
          //const Fw::CmdStringArg& j6_name,
          F64 j7_pos,
          //const Fw::CmdStringArg& j7_name,
          U32 numJoints,
          F64 tol,
          F64 goal_time,
          F64 goal_time_tol,
          F64 goal_tol
      )
    {
        if (!m_rosInited) {
            this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
            return;
        }

        FW_ASSERT(m_client);

        const U32 nJoints = FW_MIN(7, numJoints);
        Fw::CmdStringArg j1_name = "shoulder_pan_joint";
        Fw::CmdStringArg j2_name = "shoulder_lift_joint";
        Fw::CmdStringArg j3_name = "elbow_joint";
        Fw::CmdStringArg j4_name = "wrist_1_joint";
        Fw::CmdStringArg j5_name = "wrist_2_joint";
        Fw::CmdStringArg j6_name = "wrist_3_joint";
        Fw::CmdStringArg j7_name = "";

        control_msgs::FollowJointTrajectoryGoal msg;
        msg.trajectory.header.stamp.sec = 0;
        msg.trajectory.header.stamp.nsec = 0;
        msg.trajectory.header.seq = 0;
        // TODO(mereweth) - convert frame ID
        msg.trajectory.header.frame_id = "base_link";

#define TRAJ_JNT_NAME_FROM_IDX(XXX) \
        msg.trajectory.joint_names.push_back(j ## XXX ## _name.toChar());

        for (U32 i = 0; i < nJoints; i++) {
            switch (i) {
                case 0: TRAJ_JNT_NAME_FROM_IDX(1); break;
                case 1: TRAJ_JNT_NAME_FROM_IDX(2); break;
                case 2: TRAJ_JNT_NAME_FROM_IDX(3); break;
                case 3: TRAJ_JNT_NAME_FROM_IDX(4); break;
                case 4: TRAJ_JNT_NAME_FROM_IDX(5); break;
                case 5: TRAJ_JNT_NAME_FROM_IDX(6); break;
                case 6: TRAJ_JNT_NAME_FROM_IDX(7); break;
                default: FW_ASSERT(0, i); break;
            }
        }
#undef TRAJ_JNT_NAME_FROM_IDX

        trajectory_msgs::JointTrajectoryPoint jPoint;

#define JNT_POS_FROM_IDX(XXX) \
        jPoint.positions.push_back(j ## XXX ## _pos);

        for (U32 i = 0; i < nJoints; i++) {
            switch (i) {
                case 0: JNT_POS_FROM_IDX(1); break;
                case 1: JNT_POS_FROM_IDX(2); break;
                case 2: JNT_POS_FROM_IDX(3); break;
                case 3: JNT_POS_FROM_IDX(4); break;
                case 4: JNT_POS_FROM_IDX(5); break;
                case 5: JNT_POS_FROM_IDX(6); break;
                case 6: JNT_POS_FROM_IDX(7); break;
                default: FW_ASSERT(0, i); break;
            }
        }
#undef JNT_POS_FROM_IDX
        jPoint.time_from_start.sec = (U32) goal_time;
        jPoint.time_from_start.nsec = (U32) ((goal_time
                                              - jPoint.time_from_start.sec)
                                             * 1000LU * 1000LU  * 1000LU);
        
        msg.trajectory.points.push_back(jPoint);

#define JNT_TOL_FROM_IDX(XXX) \
        jTol.name = j ## XXX ## _name.toChar();

        for (U32 i = 0; i < nJoints; i++) {
            control_msgs::JointTolerance jTol;
            switch (i) {
                case 0: JNT_TOL_FROM_IDX(1); break;
                case 1: JNT_TOL_FROM_IDX(2); break;
                case 2: JNT_TOL_FROM_IDX(3); break;
                case 3: JNT_TOL_FROM_IDX(4); break;
                case 4: JNT_TOL_FROM_IDX(5); break;
                case 5: JNT_TOL_FROM_IDX(6); break;
                case 6: JNT_TOL_FROM_IDX(7); break;
                default: FW_ASSERT(0, i); break;
            }
            jTol.position = tol;
            msg.path_tolerance.push_back(jTol);
        }
#undef JNT_TOL_FROM_IDX

#define GOAL_TOL_FROM_IDX(XXX) \
        jTol.name = j ## XXX ## _name.toChar();

        for (U32 i = 0; i < nJoints; i++) {
            control_msgs::JointTolerance jTol;
            switch (i) {
                case 0: GOAL_TOL_FROM_IDX(1); break;
                case 1: GOAL_TOL_FROM_IDX(2); break;
                case 2: GOAL_TOL_FROM_IDX(3); break;
                case 3: GOAL_TOL_FROM_IDX(4); break;
                case 4: GOAL_TOL_FROM_IDX(5); break;
                case 5: GOAL_TOL_FROM_IDX(6); break;
                case 6: GOAL_TOL_FROM_IDX(7); break;
                default: FW_ASSERT(0, i); break;
            }
            jTol.position = tol;
            msg.goal_tolerance.push_back(jTol);
        }
#undef GOAL_TOL_FROM_IDX

        msg.goal_time_tolerance.sec = (U32) goal_time_tol;
        msg.goal_time_tolerance.nsec = (U32) ((goal_time_tol - 
                                               msg.goal_time_tolerance.sec)
                                              * 1000LU * 1000LU  * 1000LU);

        m_client->sendGoal(msg,
                           boost::bind(&GazeboManipIfComponentImpl::doneCb, this, _1, _2));

        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
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
