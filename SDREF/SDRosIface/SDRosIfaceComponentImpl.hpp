// ======================================================================
// \title  SDRosIfaceImpl.hpp
// \author mereweth
// \brief  hpp file for SDRosIface component implementation class
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

#ifndef SDRosIface_HPP
#define SDRosIface_HPP

#include "SDREF/SDRosIface/SDRosIfaceComponentAc.hpp"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "mav_msgs/Actuators.h"

// TODO - ImuStateUpdate.h

#include "Os/Mutex.hpp"

namespace SDREF {

  class SDRosIfaceComponentImpl :
    public SDRosIfaceComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object SDRosIface
      //!
      SDRosIfaceComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object SDRosIface
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object SDRosIface
      //!
      ~SDRosIfaceComponentImpl(void);

      void startPub();

    PRIVATE:

        //! Start interrupt task
        Os::Task::TaskStatus startIntTask(NATIVE_INT_TYPE priority,
                                          NATIVE_INT_TYPE stackSize,
                                          NATIVE_INT_TYPE cpuAffinity = -1);

      // ----------------------------------------------------------------------
      // Utility classes for enumerating callbacks
      // ----------------------------------------------------------------------

        class ActuatorsHandler
        {
          public:
              ActuatorsHandler(SDRosIfaceComponentImpl* compPtr,
                             int portNum);

              ~ActuatorsHandler();

              void actuatorsCallback(const mav_msgs::Actuators::ConstPtr& msg);

          PRIVATE:

              SDRosIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class ActuatorsHandler

        class ImuStateUpdateHandler
        {
          public:
              ImuStateUpdateHandler(SDRosIfaceComponentImpl* compPtr,
                              int portNum);

              ~ImuStateUpdateHandler();

              //void imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg);

          PRIVATE:

              SDRosIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class ImuStateUpdateHandler

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
            ROS::mav_msgs::ImuStateUpdate imuStateUpdate; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_imuStateUpdateSet[NUM_IMUSTATEUPDATE_OUTPUT_PORTS];

        struct ActuatorsSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::mav_msgs::Actuators actuators; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_actuatorsSet[NUM_ACTUATORSDATA_OUTPUT_PORTS];
    };

} // end namespace

#endif
