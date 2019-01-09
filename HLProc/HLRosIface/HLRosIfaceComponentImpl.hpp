// ======================================================================
// \title  HLRosIfaceImpl.hpp
// \author mereweth
// \brief  hpp file for HLRosIface component implementation class
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

#ifndef HLRosIface_HPP
#define HLRosIface_HPP

#include "HLProc/HLRosIface/HLRosIfaceComponentAc.hpp"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "mav_msgs/Actuators.h"
#include "mav_msgs/AttitudeRateThrust.h"
#include "mav_msgs/FlatOutput.h"
#include "mav_msgs/ImuStateUpdate.h"

#include "Os/Mutex.hpp"

namespace HLProc {

  class HLRosIfaceComponentImpl :
    public HLRosIfaceComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object HLRosIface
      //!
      HLRosIfaceComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object HLRosIface
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object HLRosIface
      //!
      ~HLRosIfaceComponentImpl(void);

      void startPub();

      //! Start interrupt task
      Os::Task::TaskStatus startIntTask(NATIVE_INT_TYPE priority,
                                        NATIVE_INT_TYPE stackSize,
                                        NATIVE_INT_TYPE cpuAffinity = -1);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Utility classes for enumerating callbacks
      // ----------------------------------------------------------------------

        class ActuatorsHandler
        {
          public:
              ActuatorsHandler(HLRosIfaceComponentImpl* compPtr,
                             int portNum);

              ~ActuatorsHandler();

              void actuatorsCallback(const mav_msgs::Actuators::ConstPtr& msg);

          PRIVATE:

              HLRosIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class ActuatorsHandler

        class ImuStateUpdateHandler
        {
          public:
              ImuStateUpdateHandler(HLRosIfaceComponentImpl* compPtr,
                              int portNum);

              ~ImuStateUpdateHandler();

              void imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg);

          PRIVATE:

              HLRosIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class ImuStateUpdateHandler

    
        class FlatOutputHandler
        {
          public:
              FlatOutputHandler(HLRosIfaceComponentImpl* compPtr,
                              int portNum);

              ~FlatOutputHandler();

              void flatOutputCallback(const mav_msgs::FlatOutput::ConstPtr& msg);

          PRIVATE:

              HLRosIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class FlatOutputHandler

        class AttitudeRateThrustHandler
        {
          public:
              AttitudeRateThrustHandler(HLRosIfaceComponentImpl* compPtr,
                                        int portNum);

              ~AttitudeRateThrustHandler();

              void attitudeRateThrustCallback(const mav_msgs::AttitudeRateThrust::ConstPtr& msg);

          PRIVATE:

              HLRosIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class AttitudeRateThrustHandler

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Imu
      //!
      void Imu_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &Imu
      );

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

        //! NodeHandle pointer for use in RateGroup context
        //!
        ros::NodeHandle* m_rgNH;

        //! Publishers for IMU data
        //!
        ros::Publisher m_imuPub[NUM_IMU_INPUT_PORTS];

        //! Publishers for Odometry data
        //!
        ros::Publisher m_odomPub[NUM_ODOMETRY_INPUT_PORTS];

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

        struct ActuatorsSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::mav_msgs::Actuators actuators; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_actuatorsSet[NUM_ACTUATORSDATA_OUTPUT_PORTS];
    
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
