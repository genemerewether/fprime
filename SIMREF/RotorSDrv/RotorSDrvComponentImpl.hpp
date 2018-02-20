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
#include "std_msgs/Float32.h"

#include "Os/Mutex.hpp"

#include "SIMREF/RotorSDrv/RotorSDrvComponentAc.hpp"

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

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

        //! Handler implementation for motor
        //!
        void motor_handler(
            const NATIVE_INT_TYPE portNum, /*!< The port number*/
            ROS::std_msgs::Float32 &Float32
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

        //! NodeHandle pointer for use in RateGroup context
        //!
        ros::NodeHandle* m_rgNH;

        //! Publishers for motor speeds
        //!
        ros::Publisher m_motorPub[NUM_MOTOR_INPUT_PORTS];

        struct OdomSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::nav_msgs::Odometry odom; //! odometry object
            bool fresh; //! Whether object has been updated
        } m_odomSet[NUM_ODOMETRY_OUTPUT_PORTS];
    };

} // end namespace SIMREF

#endif
