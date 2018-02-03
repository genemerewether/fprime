// ======================================================================
// \title  RosCycleImpl.cpp
// \author mereweth
// \brief  cpp file for RosCycle component implementation class
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


#include <ROS/RosCycle/RosCycleComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace  {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  RosCycleComponentImpl ::
#if FW_OBJECT_NAMES == 1
    RosCycleComponentImpl(
        const char *const compName
    ) :
      RosCycleComponentBase(compName)
#else
    RosCycleImpl(void)
#endif
  {

  }

  void RosCycleComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    RosCycleComponentBase::init(queueDepth, instance);
  }

  RosCycleComponentImpl ::
    ~RosCycleComponentImpl(void)
  {

  }

  Os::Task::TaskStatus RosCycleComponentImpl ::
    startIntTask(NATIVE_INT_TYPE priority,
                 NATIVE_INT_TYPE stackSize,
                 NATIVE_INT_TYPE cpuAffinity) {
      Os::TaskString name("ROSCYCLE");
      Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
        stackSize, RosCycleComponentImpl::intTaskEntry, this, cpuAffinity);

      if (stat != Os::Task::TASK_OK) {
          DEBUG_PRINT("Task start error: %d\n",stat);
      }

      return stat;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void RosCycleComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void RosCycleComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
      this->pingOut_out(portNum, key);
  }

  // ----------------------------------------------------------------------
  // Member function definitions
  // ----------------------------------------------------------------------

  //! Entry point for task waiting for RTI
  void RosCycleComponentImpl ::
    intTaskEntry(void * ptr) {

      FW_ASSERT(ptr);
      RosCycleComponentImpl* compPtr = (RosCycleComponentImpl*) ptr;

      ros::NodeHandle n;
      ros::Subscriber sub = n.subscribe("clock", 10, compPtr->clockCallback);
  }

  void clockCallback(const std_msgs::String::ConstPtr& msg)
  {
      DEBUG_PRINT("Clock callback");//, msg->data.c_str());
  }

} // end namespace
