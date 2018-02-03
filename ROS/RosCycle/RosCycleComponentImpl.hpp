// ======================================================================
// \title  RosCycleImpl.hpp
// \author mereweth
// \brief  hpp file for RosCycle component implementation class
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

#ifndef RosCycle_HPP
#define RosCycle_HPP

#include "ros/ros.h"
#include <Os/Task.hpp>

#include "ROS/RosCycle/RosCycleComponentAc.hpp"

namespace  {

  class RosCycleComponentImpl :
    public RosCycleComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object RosCycle
      //!
      RosCycleComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object RosCycle
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object RosCycle
      //!
      ~RosCycleComponentImpl(void);

      //! Start interrupt task
      Os::Task::TaskStatus startIntTask(NATIVE_INT_TYPE priority,
                                        NATIVE_INT_TYPE stackSize,
                                        NATIVE_INT_TYPE cpuAffinity = -1);
    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

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

      void chatterCallback(const std_msgs::String::ConstPtr& msg);

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------

      //! Number of /clock callbacks received
      //!
      NATIVE_UINT_TYPE m_callbacks;

      //! Entry point for task waiting for interrupt
      static void intTaskEntry(void * ptr);

      //! Task object for RTI task
      //!
      Os::Task m_intTask;

      //! flag to quit thread
      //!
      bool m_quitThread;

    };

} // end namespace

#endif
