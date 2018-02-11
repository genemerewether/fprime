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
#include "Svc/Cycle/TimerVal.hpp"

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace ROS {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  RosCycleComponentImpl ::
#if FW_OBJECT_NAMES == 1
    RosCycleComponentImpl(
        const char *const compName,
        NATIVE_UINT_TYPE timeDivMS
    ) :
      RosCycleComponentBase(compName),
#else
    RosCycleImpl(NATIVE_UINT_TYPE timeDivMS) :
#endif
      m_callbacks(0),
      m_lastCallback(TB_ROS_TIME, 0, 0),
      m_timeDiv(TB_ROS_TIME, 0, timeDivMS * 1000), // 10 ms default interval
      m_quitThread(false)
  {

  }

  void RosCycleComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    RosCycleComponentBase::init(instance);

    m_lastCallback.setTimeBase(TB_ROS_TIME);
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
      //this->tlmWrite_ ... (m_callbacks);
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
      ros::Subscriber sub = n.subscribe("clock", 1000,
                                        &RosCycleComponentImpl::clockCallback,
                                        compPtr,
                                        ros::TransportHints().tcpNoDelay());

      ros::spin();
  }

  void RosCycleComponentImpl ::
    clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
  {
      if (this->m_callbacks%100 == 0)
      {
          DEBUG_PRINT("Clock callback %d, %f\n", this->m_callbacks,
                                               msg->clock.toSec());
      }
      this->m_callbacks++;

      // Make sure we were't at zero last time either
      if ((!msg->clock.isZero()) &&
          (m_lastCallback > Fw::Time::zero(TB_ROS_TIME))) {

          Fw::Time now(TB_ROS_TIME, msg->clock.sec, msg->clock.nsec / 1000L);

          //TODO(mereweth) - keep triggering cycle port and adding timeDiv to lastCallback

          while (now >= Fw::Time::add(m_lastCallback, m_timeDiv)) {
              DEBUG_PRINT("RosCycle enough time elapsed\n");
              Svc::TimerVal cycleStart;
              cycleStart.take();
              if (this->isConnected_cycle_OutputPort(0)) {
                  DEBUG_PRINT("RosCycle invoking cycle port\n");
                  this->cycle_out(0, cycleStart);
              }
              m_lastCallback = Fw::Time::add(m_lastCallback, m_timeDiv);
              //m_lastCallback.set(TB_ROS_TIME, msg->clock.sec, msg->clock.nsec / 1000);
          }
      }
      else {
          DEBUG_PRINT("RosCycle zero last time or clock msg is zero\n");
          m_lastCallback.set(TB_ROS_TIME, msg->clock.sec, msg->clock.nsec / 1000);
      }
  }

} // end namespace
