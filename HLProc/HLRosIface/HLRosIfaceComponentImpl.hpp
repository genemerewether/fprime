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
#include "sensor_msgs/Imu.h"
#include "mav_msgs/Actuators.h"
#include <image_transport/image_transport.h>

#include "Os/Task.hpp"
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
    
        class ActuatorsHandler : public TimeBaseHolder
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

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Imu
      //!
      void Imu_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &Imu
      );

      //! Handler implementation for PointCloud
      //!
      void PointCloud_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::PointCloud2 &PointCloud2 
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

      //! Handler implementation for ImageRecv
      //!
      void ImageRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::Image &Image
      );

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------

        bool m_rosInited;

        TimeBase m_tbDes;

        //! Publishers for IMU data
        //!
        ros::Publisher m_imuPub[NUM_IMU_INPUT_PORTS];

        ros::NodeHandle* m_nodeHandle;
        image_transport::ImageTransport* m_imageXport;
        image_transport::Publisher m_imagePub[NUM_IMAGERECV_INPUT_PORTS];
        ros::Publisher m_exposurePub[NUM_IMAGERECV_INPUT_PORTS];
        ros::Publisher m_gainPub[NUM_IMAGERECV_INPUT_PORTS];
    
        ros::Publisher m_pointCloudPub[NUM_POINTCLOUD_INPUT_PORTS];

        //! Entry point for task waiting for interrupt
        static void intTaskEntry(void * ptr);

        //! Task object for RTI task
        //!
        Os::Task m_intTask;

        struct ActuatorsSet {
            Os::Mutex mutex; //! Mutex lock to guard odometry object
            ROS::mav_msgs::Actuators actuators; //! message object
            bool fresh; //! Whether object has been updated
            NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
        } m_actuatorsSet[NUM_ACTUATORSDATA_OUTPUT_PORTS];
    
    };

} // end namespace

#endif
