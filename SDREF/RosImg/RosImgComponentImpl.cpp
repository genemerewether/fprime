/**
 * \file  RosImgComponentImpl.cpp
 * \author Gene Merewether
 * \brief  Component that act as a wrapper for GNC NAV image libraries
 *
 * \copyright
 * Copyright 2009-2016, by the California Institute of Technology.
 * ALL RIGHTS RESERVED.  United States Government Sponsorship
 * acknowledged. Any commercial use must be negotiated with the Office
 * of Technology Transfer at the California Institute of Technology.
 * <br /><br />
 * This software may be subject to U.S. export control laws and
 * regulations.  By accepting this document, the user agrees to comply
 * with all U.S. export laws and regulations.  User has the
 * responsibility to obtain export licenses, or other export authority
 * as may be required before exporting such information to foreign
 * countries or providing access to foreign persons.
 */


#include <NAVOUTDOOR/RosImg/RosImgComponentImpl.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <NAV/NavCam/camera.h>

#include <Fw/Types/Assert.hpp>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Navoutdoor {

    // ----------------------------------------------------------------------
    // Initialization
    // ----------------------------------------------------------------------

    RosImgComponentImpl::
#if FW_OBJECT_NAMES == 1
    RosImgComponentImpl(const char* name) :
        RosImgComponentBase(name),
#else
        RosImgComponentImpl():
#endif
        m_nodeHandlePtr(NULL),
        m_imageTransportPtr(NULL)
    {
    }

    void RosImgComponentImpl::init(NATIVE_INT_TYPE queueDepth, NATIVE_INT_TYPE instance)
    {
      RosImgComponentBase::init(queueDepth, instance);
      m_nodeHandlePtr = new ros::NodeHandle();
      m_imageTransportPtr = new image_transport::ImageTransport(*m_nodeHandlePtr);
      m_pubImage = m_imageTransportPtr->advertise("optic", 1);
    }

    RosImgComponentImpl ::
      ~RosImgComponentImpl(void)
    {
      free(m_nodeHandlePtr);
      free(m_imageTransportPtr);
    }

    // ----------------------------------------------------------------------
    // Helper Methods
    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    // Handlers to implement for typed input ports
    // ----------------------------------------------------------------------

    void RosImgComponentImpl::ImageRecv_handler(NATIVE_INT_TYPE portNum, Fw::Buffer &fwBuffer)
    {
      static NATIVE_INT_TYPE frame_id = 0;

      DEBUG_PRINT("RosImg received buffer from NavCam\n");
      sensor_msgs::Image msg;

      camera::ICameraFrame* framePtr = (camera::ICameraFrame*) fwBuffer.getdata();

      /*TODO(mereweth@jpl.nasa.gov) - create a structure for this port
       * Use ICameraFrame as one of the fields?
       */

      sensor_msgs::fillImage( msg,
                              sensor_msgs::image_encodings::MONO8,
                              480, // rows
                              640, // cols
                              640, // step (size = rows * step)
                              reinterpret_cast<uint8_t*>(framePtr->data) );

      struct timespec monotonic_now;
      clock_gettime(CLOCK_MONOTONIC, &monotonic_now);

      struct timespec realtime_now;
      clock_gettime(CLOCK_REALTIME, &realtime_now);

      struct timespec timestamp_realtime_ns;

      if ((realtime_now.tv_nsec - monotonic_now.tv_nsec) < 0) {
      timestamp_realtime_ns.tv_sec =
                              realtime_now.tv_sec - monotonic_now.tv_sec - 1;
      timestamp_realtime_ns.tv_nsec =
                              realtime_now.tv_nsec - monotonic_now.tv_nsec +
                              (1000L * 1000L * 1000L);
      } else {
      timestamp_realtime_ns.tv_sec =
                              realtime_now.tv_sec - monotonic_now.tv_sec;
      timestamp_realtime_ns.tv_nsec =
                              realtime_now.tv_nsec - monotonic_now.tv_nsec;
      }

      timestamp_realtime_ns.tv_sec +=
                    (int32_t)(framePtr->timeStamp / (1000L * 1000L * 1000L));

      timestamp_realtime_ns.tv_nsec +=
                    (int32_t)(framePtr->timeStamp % (1000L * 1000L * 1000L));

      ros::Time frame_time;
      frame_time.sec = timestamp_realtime_ns.tv_sec;
      frame_time.nsec = timestamp_realtime_ns.tv_nsec;
      msg.header.frame_id = "optic_flow";
      msg.header.stamp = frame_time;
      msg.header.seq = frame_id;

      frame_id++; // static variable until we can use image ID from port

      m_pubImage.publish(msg);
      framePtr->releaseRef();
    }


} // end namespace
