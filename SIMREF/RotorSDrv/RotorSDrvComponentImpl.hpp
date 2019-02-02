// ======================================================================
// \title  RotorSDrvImpl.hpp
// \author mereweth
// \brief  hpp file for RotorSDrv component implementation class
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

#ifndef RotorSDrv_HPP
#define RotorSDrv_HPP

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "mav_msgs/Actuators.h"
#include "mav_msgs/AttitudeRateThrust.h"
#include "mav_msgs/FlatOutput.h"
#include "mav_msgs/ImuStateUpdate.h"

#include "Os/Mutex.hpp"

#include "SIMREF/RotorSDrv/RotorSDrvComponentAc.hpp"
#include "SIMREF/RotorSDrv/RotorSDrvComponentImplCfg.hpp"

namespace SIMREF {

  class RotorSDrvComponentImpl :
    public RotorSDrvComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

        //! Construct object RotorSDrv
        //!
        RotorSDrvComponentImpl(
  #if FW_OBJECT_NAMES == 1
            const char *const compName /*!< The component name*/
  #else
            void
  #endif
        );

        //! Initialize object RotorSDrv
        //!
        void init(
            const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
        );

        //! Destroy object RotorSDrv
        //!
        ~RotorSDrvComponentImpl(void);

        //! Initialize publishers
        //!
        void startPub(void);

        //! Start interrupt task
        Os::Task::TaskStatus startIntTask(NATIVE_INT_TYPE priority,
                                          NATIVE_INT_TYPE stackSize,
                                          NATIVE_INT_TYPE cpuAffinity = -1);

      // ----------------------------------------------------------------------
      // Utility classes for enumerating callbacks
      // ----------------------------------------------------------------------

        class ImuHandler
        {
          public:
              ImuHandler(RotorSDrvComponentImpl* compPtr,
                         int portNum);

              ~ImuHandler();

              void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

          PRIVATE:

              RotorSDrvComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class ImuHandler

        class ImuStateUpdateHandler
        {
          public:
              ImuStateUpdateHandler(RotorSDrvComponentImpl* compPtr,
                              int portNum);

              ~ImuStateUpdateHandler();

              void imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg);

          PRIVATE:

              RotorSDrvComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class ImuStateUpdateHandler

        class OdometryHandler
        {
          public:
              OdometryHandler(RotorSDrvComponentImpl* compPtr,
                              int portNum);

              ~OdometryHandler();

              void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);

          PRIVATE:

              RotorSDrvComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class OdometryHandler

        class FlatOutputHandler
        {
          public:
              FlatOutputHandler(RotorSDrvComponentImpl* compPtr,
                              int portNum);

              ~FlatOutputHandler();

              void flatOutputCallback(const mav_msgs::FlatOutput::ConstPtr& msg);

          PRIVATE:

              RotorSDrvComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class FlatOutputHandler


        class AttitudeRateThrustHandler
        {
          public:
              AttitudeRateThrustHandler(RotorSDrvComponentImpl* compPtr,
                                        int portNum);

              ~AttitudeRateThrustHandler();

              void attitudeRateThrustCallback(const mav_msgs::AttitudeRateThrust::ConstPtr& msg);

          PRIVATE:

              RotorSDrvComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class AttitudeRateThrustHandler

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------


        //! Handler implementation for Odometry
        //!
        void OdomLog_handler(
            const NATIVE_INT_TYPE portNum, /*!< The port number*/
            ROS::nav_msgs::OdometryNoCov &Odometry
        );
    
        //! Handler implementation for motor
        //!
        void motor_handler(
            const NATIVE_INT_TYPE portNum, /*!< The port number*/
            ROS::mav_msgs::Actuators &actuator
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
      // Member variables
      // ----------------------------------------------------------------------

        //! Entry point for task waiting for interrupt
        static void intTaskEntry(void * ptr);

        //! Task object for RTI task
        //!
        Os::Task m_intTask;

        bool m_rosInited;

        //! Publisher for motor speeds
        //!
        ros::Publisher m_motorPub;

        //! Publishers for Odometry data
        //!
        ros::Publisher m_odomPub[NUM_ODOMLOG_INPUT_PORTS];

        struct ImuSet {
            Os::Mutex mutex; //! Mutex lock to guard imu object
            ROS::sensor_msgs::ImuNoCov imu; //! imu object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_imuSet[NUM_SIMIMU_OUTPUT_PORTS];

        struct ImuStateUpdateSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::mav_msgs::ImuStateUpdateNoCov imuStateUpdate; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_imuStateUpdateSet[NUM_IMUSTATEUPDATE_OUTPUT_PORTS];

        struct OdomSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::nav_msgs::Odometry odom; //! odometry object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_odomSet[NUM_ODOMETRY_OUTPUT_PORTS];

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

} // end namespace SIMREF

#endif
