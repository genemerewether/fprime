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
#include <HLProc/HLRosIface/HLRosIfaceComponentImplCfg.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>

#include <Os/File.hpp>

#include <math.h>
#include <stdio.h>

#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h> 

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
    m_imuStateUpdateSet(), // zero-initialize instead of default-initializing
    m_actuatorsSet(), // zero-initialize instead of default-initializing
    m_flatOutSet(), // zero-initialize instead of default-initializing
    m_attRateThrustSet() // zero-initialize instead of default-initializing
  {
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
          m_imuStateUpdateSet[i].fresh = false;
          m_imuStateUpdateSet[i].overflows = 0u;
      }
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_actuatorsSet); i++) {
          m_actuatorsSet[i].fresh = false;
          m_actuatorsSet[i].overflows = 0u;
      }
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_flatOutSet); i++) {
          m_flatOutSet[i].fresh = false;
          m_flatOutSet[i].overflows = 0u;
      }
      for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_attRateThrustSet); i++) {
          m_attRateThrustSet[i].fresh = false;
          m_attRateThrustSet[i].overflows = 0u;
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
      startPub() {
        // TODO(mereweth) - prevent calling twice; free imageXport
        ros::NodeHandle n;
	m_imageXport = new image_transport::ImageTransport(n);

        char buf[32];
        for (int i = 0; i < NUM_IMU_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "imu_%d", i);
            m_imuPub[i] = n.advertise<sensor_msgs::Imu>(buf, 10000);
        }

        for (int i = 0; i < NUM_ODOMETRY_INPUT_PORTS; i++) {
            snprintf(buf, FW_NUM_ARRAY_ELEMENTS(buf), "odometry_%d", i);
            m_odomPub[i] = n.advertise<nav_msgs::Odometry>(buf, 5);
        }

	m_imagePub = m_imageXport->advertise("image_raw", 1);

        m_rosInited = true;
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
        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_imuStateUpdateSet); i++) {
            m_imuStateUpdateSet[i].mutex.lock();
            if (m_imuStateUpdateSet[i].fresh) {
                if (this->isConnected_ImuStateUpdate_OutputPort(i)) {
                    // mimics driver hardware getting and sending sensor data
                    this->ImuStateUpdate_out(i, m_imuStateUpdateSet[i].imuStateUpdate);
                }
                else {
                    DEBUG_PRINT("IMU state update port %d not connected\n", i);
                }
                m_imuStateUpdateSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new odometry received?
            m_imuStateUpdateSet[i].mutex.unLock();
        }

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

        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_flatOutSet); i++) {
            m_flatOutSet[i].mutex.lock();
            if (m_flatOutSet[i].fresh) {
                if (this->isConnected_flatOutput_OutputPort(i)) {
                    // mimics driver hardware getting and sending sensor data
                    this->flatOutput_out(i, m_flatOutSet[i].flatOutput);
                }
                else {
                    DEBUG_PRINT("Flat output port %d not connected\n", i);
                }
                m_flatOutSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new odometry received?
            m_flatOutSet[i].mutex.unLock();
        }

        for (int i = 0; i < FW_NUM_ARRAY_ELEMENTS(m_attRateThrustSet); i++) {
            m_attRateThrustSet[i].mutex.lock();
            if (m_attRateThrustSet[i].fresh) {
                if (this->isConnected_attRateThrust_OutputPort(i)) {
                    // mimics driver hardware getting and sending sensor data
                    this->attRateThrust_out(i, m_attRateThrustSet[i].attRateThrust);
                }
                else {
                    DEBUG_PRINT("Attitude rate thrust port %d not connected\n", i);
                }
                m_attRateThrustSet[i].fresh = false;
            }
            // TODO(mereweth) - notify that no new odometry received?
            m_attRateThrustSet[i].mutex.unLock();
        }
    }

    void HLRosIfaceComponentImpl ::
      Odometry_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::nav_msgs::OdometryNoCov &Odometry
      )
    {
        if (this->isConnected_FileLogger_OutputPort(0)) {
            Svc::ActiveFileLoggerPacket fileBuff;
            Fw::SerializeStatus stat;
            Fw::Time recvTime = this->getTime();
            fileBuff.resetSer();
            stat = fileBuff.serialize((U8)AFL_HLROSIFACE_ODOMNOCOV);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getUSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = Odometry.serialize(fileBuff);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);

            this->FileLogger_out(0,fileBuff);
        }
        
        if (!m_rosInited) {
            return;
        }
        
        nav_msgs::Odometry msg;

        ROS::std_msgs::Header header = Odometry.getheader();
        Fw::Time stamp = header.getstamp();

        //TODO(mereweth) - BEGIN convert time instead using HLTimeConv

        I64 usecDsp = (I64) stamp.getSeconds() * 1000LL * 1000LL + (I64) stamp.getUSeconds();
        Os::File::Status stat = Os::File::OTHER_ERROR;
        Os::File file;
        stat = file.open("/sys/kernel/dsp_offset/walltime_dsp_diff", Os::File::OPEN_READ);
        if (stat != Os::File::OP_OK) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        char buff[255];
        NATIVE_INT_TYPE size = sizeof(buff);
        stat = file.read(buff, size, false);
        file.close();
        if ((stat != Os::File::OP_OK) ||
            !size) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        // Make sure buffer is null-terminated:
        buff[sizeof(buff)-1] = 0;
        I64 walltimeDspLeadUs = strtoll(buff, NULL, 10);

        if (-walltimeDspLeadUs > usecDsp) {
            // TODO(mereweth) - EVR; can't have difference greater than time
            printf("linux-dsp diff %lld negative; greater than message time %lu\n",
                   walltimeDspLeadUs, usecDsp);
            return;
        }
        I64 usecRos = usecDsp + walltimeDspLeadUs;
        msg.header.stamp.sec = (U32) (usecRos / 1000LL / 1000LL);
        msg.header.stamp.nsec = (usecRos % (1000LL * 1000LL)) * 1000LU;

        //TODO(mereweth) - END convert time instead using HLTimeConv

        msg.header.seq = header.getseq();

        // TODO(mereweth) - convert frame ID
        U32 frame_id = header.getframe_id();
        msg.header.frame_id = "odom";

        msg.child_frame_id = "body";

        ROS::geometry_msgs::Point p = Odometry.getpose().getposition();
        msg.pose.pose.position.x = p.getx();
        msg.pose.pose.position.y = p.gety();
        msg.pose.pose.position.z = p.getz();
        ROS::geometry_msgs::Quaternion q = Odometry.getpose().getorientation();
        msg.pose.pose.orientation.w = q.getw();
        msg.pose.pose.orientation.x = q.getx();
        msg.pose.pose.orientation.y = q.gety();
        msg.pose.pose.orientation.z = q.getz();
        ROS::geometry_msgs::Vector3 vec = Odometry.gettwist().getlinear();
        msg.twist.twist.linear.x = vec.getx();
        msg.twist.twist.linear.y = vec.gety();
        msg.twist.twist.linear.z = vec.getz();
        vec = Odometry.gettwist().getangular();
        msg.twist.twist.angular.x = vec.getx();
        msg.twist.twist.angular.y = vec.gety();
        msg.twist.twist.angular.z = vec.getz();

        m_odomPub[portNum].publish(msg);
    }

    void HLRosIfaceComponentImpl ::
      AccelCommand_handler(
          const NATIVE_INT_TYPE portNum,
          ROS::geometry_msgs::AccelStamped &AccelStamped
      )
    {
        if (this->isConnected_FileLogger_OutputPort(0)) {
            Svc::ActiveFileLoggerPacket fileBuff;
            Fw::SerializeStatus stat;
            Fw::Time recvTime = this->getTime();
            fileBuff.resetSer();
            stat = fileBuff.serialize((U8)AFL_HLROSIFACE_ACCEL_CMD);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = fileBuff.serialize(recvTime.getUSeconds());
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);
            stat = AccelStamped.serialize(fileBuff);
            FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, stat);

            this->FileLogger_out(0,fileBuff);
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
        sensor_msgs::Image msg;
	const U8* ptr = (const U8*) Image.getdata().getdata();
	sensor_msgs::fillImage(
	    msg,
	    sensor_msgs::image_encodings::MONO8,
	    Image.getheight(),
            Image.getwidth(),
	    Image.getstep(),
	    ptr
	);
	msg.header.frame_id = "dfc";
	msg.header.stamp.sec = Image.getheader().getstamp().getSeconds();
	msg.header.stamp.nsec = Image.getheader().getstamp().getUSeconds() * 1000L;
	msg.is_bigendian = 0;
	m_imagePub.publish(msg);

	Fw::Buffer temp = Image.getdata();
        this->ImageForward_out(0, temp);
    }
  

    // ----------------------------------------------------------------------
    // Member function definitions
    // ----------------------------------------------------------------------

    //! Entry point for task waiting for messages
    void HLRosIfaceComponentImpl ::
      intTaskEntry(void * ptr) {
        DEBUG_PRINT("HLRosIface task entry\n");

        FW_ASSERT(ptr);
        HLRosIfaceComponentImpl* compPtr = (HLRosIfaceComponentImpl*) ptr;
        //compPtr->log_ACTIVITY_LO_HLROSIFACE_IntTaskStarted();

        ros::NodeHandle n;
        ros::CallbackQueue localCallbacks;
        n.setCallbackQueue(&localCallbacks);

        ImuStateUpdateHandler updateHandler(compPtr, 0);

        ros::Subscriber updateSub = n.subscribe("imu_state_update", 10,
                                                &ImuStateUpdateHandler::imuStateUpdateCallback,
                                                &updateHandler,
                                                ros::TransportHints().tcpNoDelay());


        ActuatorsHandler actuatorsHandler0(compPtr, 0);
        ros::Subscriber actuatorsSub0 = n.subscribe("flight_actuators_command", 10,
                                            &ActuatorsHandler::actuatorsCallback,
                                            &actuatorsHandler0,
                                            ros::TransportHints().tcpNoDelay());
        
        ActuatorsHandler actuatorsHandler1(compPtr, 1);
        ros::Subscriber actuatorsSub1 = n.subscribe("aux_actuators_command", 10,
                                            &ActuatorsHandler::actuatorsCallback,
                                            &actuatorsHandler1,
                                            ros::TransportHints().tcpNoDelay());

        FlatOutputHandler flatoutHandler(compPtr, 0);
        AttitudeRateThrustHandler attRateThrustHandler(compPtr, 0);
        ros::Subscriber flatoutSub = n.subscribe("flat_output_setpoint", 1,
                                                 &FlatOutputHandler::flatOutputCallback,
                                                 &flatoutHandler,
                                                 ros::TransportHints().tcpNoDelay());

        ros::Subscriber attRateThrustSub = n.subscribe("attitude_rate_thrust_setpoint", 10,
                                                       &AttitudeRateThrustHandler::attitudeRateThrustCallback,
                                                       &attRateThrustHandler,
                                                       ros::TransportHints().tcpNoDelay());
        
        while (1) {
            // TODO(mereweth) - check for and respond to ping
            localCallbacks.callAvailable(ros::WallDuration(0, 10 * 1000 * 1000));
        }
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
            Header head(msg->header.seq,
                        Fw::Time(TB_ROS_TIME, 0,
                                 msg->header.stamp.sec,
                                 msg->header.stamp.nsec / 1000),
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

    HLRosIfaceComponentImpl :: ImuStateUpdateHandler ::
      ImuStateUpdateHandler(HLRosIfaceComponentImpl* compPtr,
                      int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS); //compPtr->getNum_ImuStateUpdate_OutputPorts());
    }

    HLRosIfaceComponentImpl :: ImuStateUpdateHandler :: ~ImuStateUpdateHandler()
    {

    }

    void HLRosIfaceComponentImpl :: ImuStateUpdateHandler ::
      imuStateUpdateCallback(const mav_msgs::ImuStateUpdate::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_IMUSTATEUPDATE_OUTPUT_PORTS);

        DEBUG_PRINT("imuStateUpdate port handler %d\n", this->portNum);

	if (!std::isfinite(msg->header.stamp.sec) ||
	    !std::isfinite(msg->header.stamp.nsec)) {
	    //TODO(mereweth) - EVR
	    return;
	}
	    
        //TODO(mereweth) - BEGIN convert time instead using HLTimeConv

        I64 usecRos = (I64) msg->header.stamp.sec * 1000LL * 1000LL
                      + (I64) msg->header.stamp.nsec / 1000LL;
        Os::File::Status stat = Os::File::OTHER_ERROR;
        Os::File file;
        stat = file.open("/sys/kernel/dsp_offset/walltime_dsp_diff", Os::File::OPEN_READ);
        if (stat != Os::File::OP_OK) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        char buff[255];
        NATIVE_INT_TYPE size = sizeof(buff);
        stat = file.read(buff, size, false);
        file.close();
        if ((stat != Os::File::OP_OK) ||
            !size) {
            // TODO(mereweth) - EVR
            printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
            return;
        }
        // Make sure buffer is null-terminated:
        buff[sizeof(buff)-1] = 0;
        I64 walltimeDspLeadUs = strtoll(buff, NULL, 10);

        if (walltimeDspLeadUs > usecRos) {
            // TODO(mereweth) - EVR; can't have difference greater than time
            printf("linux-dsp diff %lld greater than message time %lu\n",
                   walltimeDspLeadUs, usecRos);
            return;
        }
        I64 usecDsp = usecRos - walltimeDspLeadUs;
        Fw::Time convTime(TB_WORKSTATION_TIME,
                          0,
                          (U32) (usecDsp / 1000 / 1000),
                          (U32) (usecDsp % (1000 * 1000)));

        //TODO(mereweth) - END convert time instead using HLTimeConv

	if (!std::isfinite(msg->pose.pose.position.x) ||
	    !std::isfinite(msg->pose.pose.position.y) ||
	    !std::isfinite(msg->pose.pose.position.z) ||
	    
	    !std::isfinite(msg->pose.pose.orientation.x) ||
            !std::isfinite(msg->pose.pose.orientation.y) ||
	    !std::isfinite(msg->pose.pose.orientation.z) ||
	    !std::isfinite(msg->pose.pose.orientation.w) ||
	    
            !std::isfinite(msg->twist.twist.linear.x) ||
	    !std::isfinite(msg->twist.twist.linear.y) ||
	    !std::isfinite(msg->twist.twist.linear.z) ||
	    
	    !std::isfinite(msg->twist.twist.angular.x) ||
	    !std::isfinite(msg->twist.twist.angular.y) ||
	    !std::isfinite(msg->twist.twist.angular.z) ||
	    
	    !std::isfinite(msg->angular_velocity_bias.x) ||
	    !std::isfinite(msg->angular_velocity_bias.y) ||
	    !std::isfinite(msg->angular_velocity_bias.z) ||
	    
	    !std::isfinite(msg->linear_acceleration_bias.x) ||
	    !std::isfinite(msg->linear_acceleration_bias.y) ||
	    !std::isfinite(msg->linear_acceleration_bias.z)) {
	  //TODO(mereweth) - EVR
	  return;
	}
	
        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;

            ImuStateUpdateNoCov imuStateUpdate(
              Header(msg->header.seq,
                     convTime,
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),
              0/*Fw::EightyCharString(msg->child_frame_id.data())*/,
              Pose(Point(msg->pose.pose.position.x,
                         msg->pose.pose.position.y,
                         msg->pose.pose.position.z),
                   Quaternion(msg->pose.pose.orientation.x,
                              msg->pose.pose.orientation.y,
                              msg->pose.pose.orientation.z,
                              msg->pose.pose.orientation.w)),
              Twist(Vector3(msg->twist.twist.linear.x,
                            msg->twist.twist.linear.y,
                            msg->twist.twist.linear.z),
                    Vector3(msg->twist.twist.angular.x,
                            msg->twist.twist.angular.y,
                            msg->twist.twist.angular.z)),
              Vector3(msg->angular_velocity_bias.x,
                      msg->angular_velocity_bias.y,
                      msg->angular_velocity_bias.z),
              Vector3(msg->linear_acceleration_bias.x,
                      msg->linear_acceleration_bias.y,
                      msg->linear_acceleration_bias.z)
            ); // end ImuStateUpdate constructor

            this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.lock();
	    if (this->compPtr->m_imuStateUpdateSet[this->portNum].fresh) {
		this->compPtr->m_imuStateUpdateSet[this->portNum].overflows++;
		DEBUG_PRINT("Overwriting imuStateUpdate port %d before Sched\n", this->portNum);
	    }
	    this->compPtr->m_imuStateUpdateSet[this->portNum].imuStateUpdate = imuStateUpdate;
	    this->compPtr->m_imuStateUpdateSet[this->portNum].fresh = true;
	}
	this->compPtr->m_imuStateUpdateSet[this->portNum].mutex.unLock();
    }

    // Flat output constructor/destructor/callback
    HLRosIfaceComponentImpl :: FlatOutputHandler ::
      FlatOutputHandler(HLRosIfaceComponentImpl* compPtr,
                        int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_FLATOUTPUT_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    HLRosIfaceComponentImpl :: FlatOutputHandler :: ~FlatOutputHandler()
    {

    }

    void HLRosIfaceComponentImpl :: FlatOutputHandler ::
      flatOutputCallback(const mav_msgs::FlatOutput::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_FLATOUTPUT_OUTPUT_PORTS);

        DEBUG_PRINT("Flat output port handler %d\n", this->portNum);

	if (!std::isfinite(msg->header.stamp.sec) ||
	    !std::isfinite(msg->header.stamp.nsec)) {
	    //TODO(mereweth) - EVR
	    return;
	}
	
	if (!std::isfinite(msg->position.x) ||
	    !std::isfinite(msg->position.y) ||
	    !std::isfinite(msg->position.z) ||
	    
	    !std::isfinite(msg->velocity.x) ||
            !std::isfinite(msg->velocity.y) ||
	    !std::isfinite(msg->velocity.z) ||
	    
            !std::isfinite(msg->acceleration.x) ||
	    !std::isfinite(msg->acceleration.y) ||
	    !std::isfinite(msg->acceleration.z) ||
	    
	    !std::isfinite(msg->yaw)) {
	  //TODO(mereweth) - EVR
	  return;
	}
	
        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            FlatOutput flatOutput(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),

              Point(msg->position.x, msg->position.y, msg->position.z),
              Vector3(msg->velocity.x, msg->velocity.y, msg->velocity.z),
              Vector3(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z),
              F64(msg->yaw)

            ); // end FlatOutput constructor

            this->compPtr->m_flatOutSet[this->portNum].mutex.lock();
            if (this->compPtr->m_flatOutSet[this->portNum].fresh) {
                this->compPtr->m_flatOutSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting flatout port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_flatOutSet[this->portNum].flatOutput = flatOutput;
            this->compPtr->m_flatOutSet[this->portNum].fresh = true;
            this->compPtr->m_flatOutSet[this->portNum].mutex.unLock();
        }
    }

    // AttitudeRateThrust constructor/destructor/callback
    HLRosIfaceComponentImpl :: AttitudeRateThrustHandler ::
      AttitudeRateThrustHandler(HLRosIfaceComponentImpl* compPtr,
                                int portNum) :
      compPtr(compPtr),
      portNum(portNum)
    {
        FW_ASSERT(compPtr);
        FW_ASSERT(portNum < NUM_ATTRATETHRUST_OUTPUT_PORTS); //compPtr->getNum_odometry_OutputPorts());
    }

    HLRosIfaceComponentImpl :: AttitudeRateThrustHandler :: ~AttitudeRateThrustHandler()
    {

    }

    void HLRosIfaceComponentImpl :: AttitudeRateThrustHandler ::
      attitudeRateThrustCallback(const mav_msgs::AttitudeRateThrust::ConstPtr& msg)
    {
        FW_ASSERT(this->compPtr);
        FW_ASSERT(this->portNum < NUM_ATTRATETHRUST_OUTPUT_PORTS);

        DEBUG_PRINT("Attitude rate thrust output port handler %d\n", this->portNum);

	if (!std::isfinite(msg->header.stamp.sec) ||
	    !std::isfinite(msg->header.stamp.nsec)) {
	    //TODO(mereweth) - EVR
	    return;
	}

	if (!std::isfinite(msg->attitude.x) ||
	    !std::isfinite(msg->attitude.y) ||
	    !std::isfinite(msg->attitude.z) ||
	    !std::isfinite(msg->attitude.w) ||
	    
	    !std::isfinite(msg->angular_rates.x) ||
            !std::isfinite(msg->angular_rates.y) ||
	    !std::isfinite(msg->angular_rates.z) ||
	    
            !std::isfinite(msg->thrust.x) ||
	    !std::isfinite(msg->thrust.y) ||
	    !std::isfinite(msg->thrust.z)) {
	  //TODO(mereweth) - EVR
	  return;
	}
	
        {
            using namespace ROS::std_msgs;
            using namespace ROS::mav_msgs;
            using namespace ROS::geometry_msgs;
            AttitudeRateThrust attRateThrust(
              Header(msg->header.seq,
                     Fw::Time(TB_ROS_TIME, 0,
                              msg->header.stamp.sec,
                              msg->header.stamp.nsec / 1000),
                     // TODO(mereweth) - convert frame id
                     0/*Fw::EightyCharString(msg->header.frame_id.data())*/),

              Quaternion(msg->attitude.x, msg->attitude.y, msg->attitude.z, msg->attitude.w),
              Vector3(msg->angular_rates.x, msg->angular_rates.y, msg->angular_rates.z),
              Vector3(msg->thrust.x, msg->thrust.y, msg->thrust.z)

            ); // end AttitudeRateThrust constructor

            this->compPtr->m_attRateThrustSet[this->portNum].mutex.lock();
            if (this->compPtr->m_attRateThrustSet[this->portNum].fresh) {
                this->compPtr->m_attRateThrustSet[this->portNum].overflows++;
                DEBUG_PRINT("Overwriting attitude rate thrust port %d before Sched\n", this->portNum);
            }
            this->compPtr->m_attRateThrustSet[this->portNum].attRateThrust = attRateThrust;
            this->compPtr->m_attRateThrustSet[this->portNum].fresh = true;
            this->compPtr->m_attRateThrustSet[this->portNum].mutex.unLock();
        }
    }
  
} // end namespace
