// ====================================================================== 
// \title  RosSeqImpl.cpp
// \author genemerewether
// \brief  cpp file for RosSeq component implementation class
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

#include <ros/callback_queue.h>

#include <ROS/RosSeq/RosSeqComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace ROS {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  RosSeqComponentImpl ::
#if FW_OBJECT_NAMES == 1
    RosSeqComponentImpl(
        const char *const compName
    ) :
      RosSeqComponentBase(compName),
#else
      RosSeqImpl(void),
#endif
      m_rosInited(false),
      m_nodeHandle(NULL),
      m_actionServer(NULL)
  {

  }

  void RosSeqComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    RosSeqComponentBase::init(instance);
  }

  RosSeqComponentImpl ::
    ~RosSeqComponentImpl(void)
  {

  }

    void RosSeqComponentImpl ::
      startPub() {
        // TODO(mereweth) - prevent calling twice; free ActionServer
        FW_ASSERT(m_nodeHandle);
	m_actionServer = new actionlib::SimpleActionServer<fprime_msgs::RunSeqAction>(*m_nodeHandle,
										      "ROSSEQ",
										      false);
	m_actionServer->registerGoalCallback(boost::bind(&RosSeqComponentImpl::goalCB, this));
	m_actionServer->registerPreemptCallback(boost::bind(&RosSeqComponentImpl::preemptCB, this));
	
	m_actionServer->start();

        m_rosInited = true;
    }

    Os::Task::TaskStatus RosSeqComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
		   NATIVE_INT_TYPE stackSize,
		   NATIVE_INT_TYPE cpuAffinity) {
	Os::TaskString name("ROSSEQ");
	Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
	  stackSize, RosSeqComponentImpl::intTaskEntry, this, cpuAffinity);

	if (stat != Os::Task::TASK_OK) {
	  //DEBUG_PRINT("Task start error: %d\n",stat);
	}

	return stat;
    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void RosSeqComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void RosSeqComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    // TODO
  }

    void RosSeqComponentImpl ::
      seqDoneIn_handler(
	  const NATIVE_INT_TYPE portNum,
	  FwOpcodeType opCode,
	  U32 cmdSeq,
	  Fw::CommandResponse response
      )
    {
        FW_ASSERT(this->m_actionServer);
	if (Fw::COMMAND_OK == response) {
	    this->m_actionServer->setSucceeded();
	}
	else if (!this->m_actionServer->isActive()) {
	    this->m_actionServer->setAborted();
	}
    }

    // ----------------------------------------------------------------------
    // Member function definitions
    // ----------------------------------------------------------------------

    void RosSeqComponentImpl ::
      goalCB() {
        FW_ASSERT(this->m_actionServer);
	if (this->isConnected_seqRunOut_OutputPort(0)) {
	    const char* const pathToSeq = this->m_actionServer->acceptNewGoal()->pathToSeq.data.c_str();
	    if (strlen(pathToSeq) > 79) { // max size of string
	        // TODO(mereweth) - EVR or ros message
	        this->m_actionServer->setAborted();
	    }
	    Fw::EightyCharString seqName(pathToSeq);
	    this->seqRunOut_out(0, seqName);
	}
	else {
            // TODO(mereweth) - EVR or ros message
	    this->m_actionServer->acceptNewGoal();
	    this->m_actionServer->setAborted();
	}

	// NOTE(Mereweth) - from SimpleActionServer docs:
	/* Preempts received for the new goal between checking if 
	 * isNewGoalAvailable or invokation of a goal callback and
	 * the acceptNewGoal call will not trigger a preempt callback.
	 * This means, isPreemptReqauested should be called after
	 * accepting the goal even for callback-based implementations
	 * to make sure the new goal does not have a pending preempt 
	 * request.
	 */

	if (this->m_actionServer->isPreemptRequested() || !ros::ok()) {
	    this->m_actionServer->setPreempted();
	}
    }

    void RosSeqComponentImpl ::
      preemptCB() {
        FW_ASSERT(this->m_actionServer);
	if (this->isConnected_seqCancelOut_OutputPort(0)) {
  	    this->seqCancelOut_out(0);
	}
	this->m_actionServer->setPreempted();
    }

    //! Entry point for task waiting for messages
    void RosSeqComponentImpl ::
      intTaskEntry(void * ptr) {
      //DEBUG_PRINT("RosSeq task entry\n");

        FW_ASSERT(ptr);
        RosSeqComponentImpl* compPtr = (RosSeqComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_HLROSIFACE_IntTaskStarted();

	compPtr->m_nodeHandle = new ros::NodeHandle();
        ros::NodeHandle* const n = compPtr->m_nodeHandle;
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);
        
        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }

} // end namespace ROS
