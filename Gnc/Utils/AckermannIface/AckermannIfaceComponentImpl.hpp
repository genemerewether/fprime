// ======================================================================
// \title  AckermannIfaceImpl.hpp
// \author mereweth
// \brief  hpp file for AckermannIface component implementation class
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

#ifndef AckermannIface_HPP
#define AckermannIface_HPP

#include "Gnc/Utils/AckermannIface/AckermannIfaceComponentAc.hpp"

#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"

#include "Os/Task.hpp"
#include "Os/Mutex.hpp"

namespace Gnc {

  class AckermannIfaceComponentImpl :
    public AckermannIfaceComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object AckermannIface
      //!
      AckermannIfaceComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object AckermannIface
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object AckermannIface
      //!
      ~AckermannIfaceComponentImpl(void);

      void setTBDes(TimeBase tbDes);
    
      void startPub();

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
    
        class AckermannDriveStampedHandler : public TimeBaseHolder
        {
          public:
              AckermannDriveStampedHandler(AckermannIfaceComponentImpl* compPtr,
                              int portNum);

              ~AckermannDriveStampedHandler();

              void ackermannDriveStampedCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);

          PRIVATE:

              AckermannIfaceComponentImpl* compPtr;

              const unsigned int portNum;

        }; // end class AckermannDriveStampedHandler

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

        struct AckermannDriveStampedSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::ackermann_msgs::AckermannDriveStamped ackermannDriveStamped; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_ackermannDriveStampedSet[NUM_ACKERMANNDRIVESTAMPED_OUTPUT_PORTS];
    
    };

} // end namespace

#endif
