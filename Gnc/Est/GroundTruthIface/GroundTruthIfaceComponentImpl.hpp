// ======================================================================
// \title  GroundTruthIfaceImpl.hpp
// \author mereweth
// \brief  hpp file for GroundTruthIface component implementation class
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

#ifndef GroundTruthIface_HPP
#define GroundTruthIface_HPP

#include "Gnc/Est/GroundTruthIface/GroundTruthIfaceComponentAc.hpp"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"

#include "Os/Task.hpp"
#include "Os/Mutex.hpp"

namespace Gnc {

  class GroundTruthIfaceComponentImpl :
    public GroundTruthIfaceComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object GroundTruthIface
      //!
      GroundTruthIfaceComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object GroundTruthIface
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object GroundTruthIface
      //!
      ~GroundTruthIfaceComponentImpl(void);

      void setTBDes(TimeBase tbDes);

      //! Start interrupt task
      Os::Task::TaskStatus startIntTask(NATIVE_INT_TYPE priority,
                                        NATIVE_INT_TYPE stackSize,
                                        NATIVE_INT_TYPE cpuAffinity = -1);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Utility classes for enumerating callbacks
      // ----------------------------------------------------------------------

        class TimeBaseHolder
        {
          public:
              TimeBaseHolder();
              TimeBase tbDes;
        };
    
        class OdometryHandler : public TimeBaseHolder
        {
          public:
              OdometryHandler(GroundTruthIfaceComponentImpl* compPtr,
                              int portNum);

              ~OdometryHandler();

              void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

          PRIVATE:

              GroundTruthIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class OdometryHandler

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

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------

        bool m_rosInited;

        TimeBase m_tbDes;

        ros::NodeHandle* m_nodeHandle;

        //! Entry point for task waiting for interrupt
        static void intTaskEntry(void * ptr);

        //! Task object for RTI task
        //!
        Os::Task m_intTask;

        struct OdometrySet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::nav_msgs::OdometryAccel odometry; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_odometrySet[NUM_ODOMETRY_OUTPUT_PORTS];
    
    };

} // end namespace

#endif
