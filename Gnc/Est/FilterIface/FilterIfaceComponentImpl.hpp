// ======================================================================
// \title  FilterIfaceImpl.hpp
// \author mereweth
// \brief  hpp file for FilterIface component implementation class
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

#ifndef FilterIface_HPP
#define FilterIface_HPP

#include "Gnc/Est/FilterIface/FilterIfaceComponentAc.hpp"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "mav_msgs/ImuStateUpdate.h"
#include "tf/transform_broadcaster.h"

#include "Os/Task.hpp"
#include "Os/Mutex.hpp"

namespace Gnc {

  class FilterIfaceComponentImpl :
    public FilterIfaceComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object FilterIface
      //!
      FilterIfaceComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object FilterIface
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object FilterIface
      //!
      ~FilterIfaceComponentImpl(void);

      void startPub();

      //! Start interrupt task
      Os::Task::TaskStatus startIntTask(NATIVE_INT_TYPE priority,
                                        NATIVE_INT_TYPE stackSize,
                                        NATIVE_INT_TYPE cpuAffinity = -1);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Utility classes for enumerating callbacks
      // ----------------------------------------------------------------------

        class ImuStateUpdateHandler
        {
          public:
              ImuStateUpdateHandler(FilterIfaceComponentImpl* compPtr,
                              int portNum);

              ~ImuStateUpdateHandler();

              void imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg);

          PRIVATE:

              FilterIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class ImuStateUpdateHandler

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for Odometry
      //!
      void Odometry_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::nav_msgs::OdometryNoCov &Odometry
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

        //! Publishers for Odometry data
        //!
        ros::Publisher m_odomPub[NUM_ODOMETRY_INPUT_PORTS];

        ros::NodeHandle* m_nodeHandle;

        tf::TransformBroadcaster* m_trBroad;

        //! Entry point for task waiting for interrupt
        static void intTaskEntry(void * ptr);

        //! Task object for RTI task
        //!
        Os::Task m_intTask;

        struct ImuStateUpdateSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::mav_msgs::ImuStateUpdateNoCov imuStateUpdate; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_imuStateUpdateSet[NUM_IMUSTATEUPDATE_OUTPUT_PORTS];
    
    };

} // end namespace

#endif
