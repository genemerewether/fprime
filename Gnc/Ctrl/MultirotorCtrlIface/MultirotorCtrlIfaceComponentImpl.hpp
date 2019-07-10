// ======================================================================
// \title  MultirotorCtrlIfaceImpl.hpp
// \author mereweth
// \brief  hpp file for MultirotorCtrlIface component implementation class
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

#ifndef MultirotorCtrlIface_HPP
#define MultirotorCtrlIface_HPP

#include "Gnc/Ctrl/MultirotorCtrlIface/MultirotorCtrlIfaceComponentAc.hpp"

#include "ros/ros.h"
#include "mav_msgs/AttitudeRateThrust.h"
#include "mav_msgs/BoolStamped.h"
#include "mav_msgs/FlatOutput.h"

#include "Os/Task.hpp"
#include "Os/Mutex.hpp"

namespace Gnc {

  class MultirotorCtrlIfaceComponentImpl :
    public MultirotorCtrlIfaceComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MultirotorCtrlIface
      //!
      MultirotorCtrlIfaceComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object MultirotorCtrlIface
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object MultirotorCtrlIface
      //!
      ~MultirotorCtrlIfaceComponentImpl(void);

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
    
        class BoolStampedHandler : public TimeBaseHolder
        {
          public:
              BoolStampedHandler(MultirotorCtrlIfaceComponentImpl* compPtr,
				 int portNum);

              ~BoolStampedHandler();

              void boolStampedCallback(const mav_msgs::BoolStamped::ConstPtr& msg);

          PRIVATE:

              MultirotorCtrlIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class BoolStampedHandler
    
        class FlatOutputHandler : public TimeBaseHolder
        {
          public:
              FlatOutputHandler(MultirotorCtrlIfaceComponentImpl* compPtr,
                              int portNum);

              ~FlatOutputHandler();

              void flatOutputCallback(const mav_msgs::FlatOutput::ConstPtr& msg);

          PRIVATE:

              MultirotorCtrlIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class FlatOutputHandler

        class AttitudeRateThrustHandler : public TimeBaseHolder
        {
          public:
              AttitudeRateThrustHandler(MultirotorCtrlIfaceComponentImpl* compPtr,
                                        int portNum);

              ~AttitudeRateThrustHandler();

              void attitudeRateThrustCallback(const mav_msgs::AttitudeRateThrust::ConstPtr& msg);

          PRIVATE:

              MultirotorCtrlIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class AttitudeRateThrustHandler

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for AccelCommand
      //!
      void AccelCommand_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::geometry_msgs::AccelStamped &AccelStamped
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

        struct BoolStampedSet {
            Os::Mutex mutex; //! Mutex lock to guard flat output object
            ROS::mav_msgs::BoolStamped boolStamped; //! bool stamped object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_boolStampedSet[NUM_BOOLSTAMPED_OUTPUT_PORTS];

        struct FlatOutSet {
            Os::Mutex mutex; //! Mutex lock to guard flat output object
            ROS::mav_msgs::FlatOutput flatOutput; //! flat output object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_flatOutSet[NUM_FLATOUTPUT_OUTPUT_PORTS];

        struct AttRateThrustSet {
            Os::Mutex mutex; //! Mutex lock to guard object
            ROS::mav_msgs::AttitudeRateThrust attRateThrust; //! Attitude, rate, and thrust object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_attRateThrustSet[NUM_ATTRATETHRUST_OUTPUT_PORTS];
    
    };

} // end namespace

#endif
