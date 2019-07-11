// ======================================================================
// \title  HLRosIfaceImpl.cpp
// \author mereweth
// \brief  cpp file for HLRosIface component implementation class
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


#include <HLProc/HLRosIface/HLRosIfaceComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

#include <Os/File.hpp>

#include <math.h>
#include <stdio.h>

#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/fill_image.h>
#include "std_msgs/Float32.h"

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace HLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  HLRosIfaceComponentImpl ::
#if FW_OBJECT_NAMES == 1
    HLRosIfaceComponentImpl(
        const char *const compName
    ) :
      HLRosIfaceComponentBase(compName),
#else
    HLRosIfaceImpl(void),
#endif
    m_rosInited(false),
    m_tbDes(TB_NONE),
    m_nodeHandle(NULL),
    m_imageXport(NULL),
    m_actuatorsSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_actuatorsSet); i++) {
          m_actuatorsSet[i].fresh = false;
          m_actuatorsSet[i].overflows = 0u;
      }
  }

    void HLRosIfaceComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      HLRosIfaceComponentBase::init(instance);
    }

    HLRosIfaceComponentImpl ::
      ~HLRosIfaceComponentImpl(void)
    {

    }

    void HLRosIfaceComponentImpl ::
      setTBDes(TimeBase tbDes) {
        this->m_tbDes = tbDes;
    }
  
    Os::Task::TaskStatus HLRosIfaceComponentImpl ::
      startIntTask(NATIVE_INT_TYPE priority,
                   NATIVE_INT_TYPE stackSize,
                   NATIVE_INT_TYPE cpuAffinity) {
        Os::TaskString name("HLROSIFACE");
        Os::Task::TaskStatus stat = this->m_intTask.start(name, 0, priority,
          stackSize, HLRosIfaceComponentImpl::intTaskEntry, this, cpuAffinity);

        if (stat != Os::Task::TASK_OK) {
            DEBUG_PRINT("Task start error: %d\n",stat);
        }

        return stat;
    }
  
    void HLRosIfaceComponentImpl ::
      disableRos() {
        this->m_rosInited = false;
    }
  
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

    void HLRosIfaceComponentImpl ::
      Imu_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::sensor_msgs::ImuNoCov &Imu
      )
    {

        ROS::std_msgs::Header header = Imu.getheader();
        Fw::Time stamp = header.getstamp();

        // if port is not connected, default to no conversion
        Fw::Time convTime = stamp;

        if (this->isConnected_convertTime_OutputPort(0)) {
            bool success = false;
            convTime = this->convertTime_out(0, stamp, TB_ROS_TIME, 0, success);
            if (!success) {
                // TODO(Mereweth) - EVR
                return;
            }
        }

        header.setstamp(convTime);
        Imu.setheader(header);
        
        if (this->isConnected_FileLogger_OutputPort(0)) {
            Svc::ActiveFileLoggerPacket fileBuff;
            Fw::SerializeStatus stat;
            Fw::Time recvTime = this->getTime();
            fileBuff.resetSer();
            stat = fileBuff.serialize((U8)AFL_HLROSIFACE_IMUNOCOV);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getUSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = Imu.serialize(fileBuff);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);

            this->FileLogger_out(0,fileBuff);
        }

        if (!m_rosInited) {
            return;
        }

        sensor_msgs::Imu msg;
        msg.header.stamp.sec = header.getstamp().getSeconds();
        msg.header.stamp.nsec = header.getstamp().getUSeconds() * 1000L;

        msg.header.seq = header.getseq();

        // TODO(mereweth) - convert frame ID
        U32 frame_id = header.getframe_id();
        msg.header.frame_id = "mpu9250";

        msg.orientation_covariance[0] = -1;

        ROS::geometry_msgs::Vector3 vec = Imu.getlinear_acceleration();
        msg.linear_acceleration.x = vec.getx();
        msg.linear_acceleration.y = vec.gety();
        msg.linear_acceleration.z = vec.getz();
        vec = Imu.getangular_velocity();
        msg.angular_velocity.x = vec.getx();
        msg.angular_velocity.y = vec.gety();
        msg.angular_velocity.z = vec.getz();
        m_imuPub[portNum].publish(msg);
    }

    void HLRosIfaceComponentImpl ::
      sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_actuatorsSet); i++) {
            m_actuatorsSet[i].mutex.lock();
            if (m_actuatorsSet[i].fresh) {
                if (this->isConnected_ActuatorsData_OutputPort(i)) {
                    // mimics driver hardware getting and sending sensor data
                    this->ActuatorsData_out(i, m_actuatorsSet[i].actuators);
                }
                else {
                    DEBUG_PRINT("Actuators port %d not connected\n", i);
                }
                m_actuatorsSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new actuators received?
            m_actuatorsSet[i].mutex.unLock();
        }
    }

    void HLRosIfaceComponentImpl ::
      pingIn_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
        // TODO(mereweth) - check that message-wait task is OK if we add one
        this->pingOut_out(portNum, key);
    }

    void HLRosIfaceComponentImpl ::
      ImageRecv_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::sensor_msgs::Image &Image
      )
    {
        if (m_rosInited) {
            sensor_msgs::Image msg;
            const U8* ptr = (const U8*) Image.getdata().getdata();
            sensor_msgs::fillImage(msg,
                                   sensor_msgs::image_encodings::MONO8,
                                   Image.getheight(),
                                   Image.getwidth(),
                                   Image.getstep(),
                                   ptr);
            if (0 == portNum) {
                msg.header.frame_id = "dfc";
            }
            else {
                msg.header.frame_id = "stereo";
            }
            msg.header.stamp.sec = Image.getheader().getstamp().getSeconds();
            msg.header.stamp.nsec = Image.getheader().getstamp().getUSeconds() * 1000L;
            msg.is_bigendian = Image.getis_bigendian();
            std_msgs::Float32 gain;
            gain.data = Image.getgain();
            m_gainPub[portNum].publish(gain);
            std_msgs::Float32 exposure;
            exposure.data = Image.getexposure();
            m_exposurePub[portNum].publish(exposure);
            
            m_imagePub[portNum].publish(msg);
        }
        Fw::Buffer temp = Image.getdata();
        this->ImageForward_out(portNum, temp);
    }

    void HLRosIfaceComponentImpl ::
      PointCloud_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::sensor_msgs::PointCloud2 &PointCloud2
      )
    {
        if (m_rosInited) {
            using namespace ROS::sensor_msgs;
            
            sensor_msgs::PointCloud2 msg;
            const U8* ptr = (const U8*) PointCloud2.getdata().getdata();
            U32 uSize = PointCloud2.getdata().getsize();
            msg.data.resize(uSize);
            memcpy(&msg.data[0], ptr, uSize);

            int size = 0;
            const PointField *fields = PointCloud2.getfields(size);
            size = FW_MIN(size, PointCloud2.getfields_count());
            msg.fields.resize(size);
            for (int i = 0; i < size; i++) {
                msg.fields[i].name = fields[i].getname().toChar();
                msg.fields[i].offset = fields[i].getoffset();
                msg.fields[i].datatype = fields[i].getdatatype();
                msg.fields[i].count = fields[i].getcount();
            }
            
            if (0 == portNum) {
                msg.header.frame_id = "stereo";
            }
            else if (1 == portNum) { // from vislam
                msg.header.frame_id = "world";
            }
            else {
                msg.header.frame_id = "unknown";
            }
            msg.header.stamp.sec = PointCloud2.getheader().getstamp().getSeconds();
            msg.header.stamp.nsec = PointCloud2.getheader().getstamp().getUSeconds() * 1000L;
            msg.height = PointCloud2.getheight();
            msg.width = PointCloud2.getwidth();
            msg.is_bigendian = PointCloud2.getis_bigendian();
            msg.point_step = PointCloud2.getpoint_step();
            msg.row_step = PointCloud2.getrow_step();
            msg.is_dense = PointCloud2.getis_dense();
            
            m_pointCloudPub[portNum].publish(msg);
        }
        Fw::Buffer temp = PointCloud2.getdata();
        // NOTE(Mereweth) - when we change to cycling pointcloud buffers, remove isConnected
        if (this->isConnected_PointCloudForward_OutputPort(0)) {
            this->PointCloudForward_out(portNum, temp);
        }
    }
  
    // ----------------------------------------------------------------------
    // Member function definitions
    // ----------------------------------------------------------------------

    //! Entry point for task waiting for messages
    void HLRosIfaceComponentImpl ::
      intTaskEntry(void * ptr) {
        // TODO(mereweth) - prevent calling twice; free m_nodeHandle
        DEBUG_PRINT("HLRosIface task entry\n");

        FW_ASSERT(ptr);
        HLRosIfaceComponentImpl* compPtr = (HLRosIfaceComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_HLROSIFACE_IntTaskStarted();

        compPtr->m_nodeHandle = new ros::NodeHandle();
        ros::NodeHandle* n = compPtr->m_nodeHandle;
        FW_ASSERT(n);
        ros::CallbackQueue localCallbacks;
        n->setCallbackQueue(&localCallbacks);

        if (ros::isShuttingDown()) {
            return;
        }
        
        compPtr->m_imageXport = new image_transport::ImageTransport(*n);

        char buf[512];
        for (int i = 0; i < NUM_IMU_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "imu_%d", i);
            compPtr->m_imuPub[i] = n->advertise<sensor_msgs::Imu>(buf, 10000);
        }

        for (int i = 0; i < NUM_IMAGERECV_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "image_%d", i);
            compPtr->m_imagePub[i] = compPtr->m_imageXport->advertise(buf, 1);
            
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "image_exposure_%d", i);
            compPtr->m_exposurePub[i] = n->advertise<std_msgs::Float32>(buf, 1);
            
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "image_gain_%d", i);
            compPtr->m_gainPub[i] = n->advertise<std_msgs::Float32>(buf, 1);
        }

        for (int i = 0; i < NUM_POINTCLOUD_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "pointcloud_%d", i);
            compPtr->m_pointCloudPub[i] = n->advertise<sensor_msgs::PointCloud2>(buf, 1);
        }

        ActuatorsHandler actuatorsHandler0(compPtr, 0);
        actuatorsHandler0.tbDes = compPtr->m_tbDes;
        ros::Subscriber actuatorsSub0 = n->subscribe("flight_actuators_command", 10,
                                            &ActuatorsHandler::actuatorsCallback,
                                            &actuatorsHandler0,
                                            ros::TransportHints().tcpNoDelay());

        ActuatorsHandler actuatorsHandler1(compPtr, 1);
        actuatorsHandler1.tbDes = compPtr->m_tbDes;
        ros::Subscriber actuatorsSub1 = n->subscribe("aux_actuators_command", 10,
                                            &ActuatorsHandler::actuatorsCallback,
                                            &actuatorsHandler1,
                                            ros::TransportHints().tcpNoDelay());

        compPtr->m_rosInited = true;
	
        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
    }

    HLRosIfaceComponentImpl :: TimeBaseHolder ::
      TimeBaseHolder() :
      tbDes(TB_NONE)
    {
    }

    HLRosIfaceComponentImpl :: ActuatorsHandler ::
      ActuatorsHandler(HLRosIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_ACTUATORSDATA_OUTPUT_PORTS); //compPtr->getNum_ImuStateUpdate_OutputPorts());
    }

    HLRosIfaceComponentImpl :: ActuatorsHandler :: ~ActuatorsHandler()
    {

    }

    void HLRosIfaceComponentImpl :: ActuatorsHandler ::
      actuatorsCallback(const mav_msgs::Actuators::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_ACTUATORSDATA_OUTPUT_PORTS);

        DEBUG_PRINT("ActuatorsData port handler %d\n", this->portNum);

        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;

            if (!std::isfinite(msg->header.stamp.sec) ||
                !std::isfinite(msg->header.stamp.nsec)) {
                //TODO(mereweth) - EVR
                return;
            }

            Fw::Time rosTime(TB_ROS_TIME, 0,
                             msg->header.stamp.sec,
                             msg->header.stamp.nsec / 1000);

            // if port is not connected, default to no conversion
            Fw::Time convTime = rosTime;

            if (this->compPtr->isConnected_convertTime_OutputPort(0)) {
                bool success = false;
                convTime = this->compPtr->convertTime_out(0, rosTime, this->tbDes, 0, success);
                if (!success) {
                    // TODO(Mereweth) - EVR
                    return;
                }
            }
            
            Header head(msg->header.seq,
                        convTime,
                        // TODO(mereweth) - convert frame id
                        0/*Fw::EightyCharString(msg->header.frame_id.data())*/);

            Actuators actuators;
            actuators.setheader(head);
            NATIVE_INT_TYPE size;
            /* TODO(mereweth) - convention on whether to make count fields the original
             * ROS value or the min of that and the allocated size
             */
            size = msg->angles.size();
            for (NATIVE_INT_TYPE i = 0; i < size; i++) {
                if (!std::isfinite(msg->angles[i])) {
                    //TODO(mereweth) - EVR
                    return;
                }
            }
            actuators.setangles(msg->angles.data(), size);
            actuators.setangles_count(msg->angles.size());

            size = msg->angular_velocities.size();
            for (NATIVE_INT_TYPE i = 0; i < size; i++) {
                if (!std::isfinite(msg->angular_velocities[i])) {
                    //TODO(mereweth) - EVR
                    return;
                }
            }
            actuators.setangular_velocities(msg->angular_velocities.data(), size);
            actuators.setangular_velocities_count(msg->angular_velocities.size());

            size = msg->normalized.size();
            for (NATIVE_INT_TYPE i = 0; i < size; i++) {
                if (!std::isfinite(msg->normalized[i])) {
                    //TODO(mereweth) - EVR
                    return;
                }
            }
            actuators.setnormalized(msg->normalized.data(), size);
            actuators.setnormalized_count(msg->normalized.size());

            DEBUG_PRINT("Actuators port %d sizes %u, %u, %u\n",
                        this->portNum,
                        msg->angles.size(),
                        msg->angular_velocities.size(),
                        msg->normalized.size());

            this->compPtr->m_actuatorsSet[this->portNum].mutex.lock();
            if (this->compPtr->m_actuatorsSet[this->portNum].fresh) {
                this->compPtr->m_actuatorsSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting Actuators port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_actuatorsSet[this->portNum].actuators = actuators;
            this->compPtr->m_actuatorsSet[this->portNum].fresh = true;
            this->compPtr->m_actuatorsSet[this->portNum].mutex.unLock();
        }
    }

} // end namespace
