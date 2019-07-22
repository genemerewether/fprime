// ====================================================================== 
// \title  MVVislamImpl.cpp
// \author mereweth
// \brief  cpp file for MVVislam component implementation class
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


#include <SnapdragonFlight/MVVislam/MVVislamComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Eigen/Geometry>
#include <math.h>

#include <Os/File.hpp>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  MVVislamComponentImpl ::
#if FW_OBJECT_NAMES == 1
    MVVislamComponentImpl(
        const char *const compName
    ) :
      MVVislamComponentBase(compName)
#else
      MVVislamImpl(void)
#endif
#ifdef BUILD_SDFLIGHT
      ,m_mvVISLAMPtr(NULL)
#endif
      ,m_initialized(false)
      ,m_activated(false)
      ,m_errorCode(0u)
      ,w_q_b(0.0, 0.0, 0.0, 1.0)
      ,x_b(0.0, 0.0, 0.0)
      ,m_tbDes(TB_NONE)
  {

  }

  void MVVislamComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    ) 
  {
    MVVislamComponentBase::init(queueDepth, instance);
  }

  MVVislamComponentImpl ::
    ~MVVislamComponentImpl(void)
  {

  }

  void MVVislamComponentImpl ::
    setTBDes(TimeBase tbDes) {
      this->m_tbDes = tbDes;
  }
  
  void MVVislamComponentImpl ::
    preamble(void)
  {
      initHelper();
  }


  void MVVislamComponentImpl ::
    initHelper(void)
  {
#ifdef SOC_8074
      // NOTE(mereweth) - x,y,z, offsets in meters
      F32 tbc[] = { 0.005, 0.0150, 0.0 };
      // NOTE(mereweth) - axis-angle rep; rotation of 90 deg about Z
      F32 ombc[] = { 0.0, 0.0, 1.57 };
#else
      // NOTE(mereweth) - x,y,z, offsets in meters
      F32 tbc[] = { -0.051, 0.015, 0.011 };
      // NOTE(mereweth) - axis-angle rep; 
      F32 ombc[] = { 0.6149, -0.6149, -1.4817 };
#endif

      F32 std0Tbc[] = { 0.005, 0.005, 0.005 };
      F32 std0Ombc[] = { 0.04, 0.04, 0.04 };

      F32 tba[] = { 0.0, 0.0, 0.0 };
    
#ifdef BUILD_SDFLIGHT
      mvCameraConfiguration camCfg;
      camCfg.pixelWidth = 640;
      camCfg.pixelHeight = 480;
      camCfg.memoryStride = 640;

      camCfg.principalPoint[0] = 320.0;
      camCfg.principalPoint[1] = 240.0;
      camCfg.focalLength[0] = 275.0;
      camCfg.focalLength[1] = 275.0;
      camCfg.uvOffset = 0;
      camCfg.distortionModel = 10;
      camCfg.distortion[0] = .003908;
      camCfg.distortion[1] = -0.009574;
      camCfg.distortion[2] = 0.010173;
      camCfg.distortion[3] = -0.003329;

      this->m_mvVISLAMPtr = 
        mvVISLAM_Initialize(&camCfg,
                            0.0f, //readoutTime
                            tbc,
                            ombc,
#ifdef SOC_8074
                            -0.0068, //delta
#else
                            0.002f, //delta
#endif
                            std0Tbc,
                            std0Ombc,
                            0.001f, //std0Delta
                            156.0f, //accelMeasRange
                            34.0f, //gyroMeasRange
                            0.316f, //stdAccelMeasNoise
                            1e-2f, //stdGyroMeasNoise
                            100.0f, //stdCamNoise
                            0.5f, //minStdPixelNoise
                            1.6651f, //failHighPixelNoiseScaleFactor
                            0.0f, //logDepthBootstrap
                            false, //useLogCameraHeight
                            -3.22f, //logCameraHeightBootstrap
                            false, //noInitWhenMoving
                            200.0f, //limitedIMUbWtrigger
                            "na", //staticMaskFilename
                            0.0f, //gpsImuTimeAlignment
                            tba,
                            false //mapping
      );
      if (NULL != m_mvVISLAMPtr) {
          m_initialized = true;
      }
#endif //BUILD_SDFLIGHT
  }
  
  void MVVislamComponentImpl ::
    finalizer(void)
  {
      m_initialized = false;
#ifdef BUILD_SDFLIGHT
      if (NULL != m_mvVISLAMPtr) {
          mvVISLAM_Deinitialize(m_mvVISLAMPtr);
      }
#endif //BUILD_SDFLIGHT
  }
  
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void MVVislamComponentImpl ::
    Imu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &imu
    )
  {
      ROS::std_msgs::Header h = imu.getheader();
      Fw::Time stamp = h.getstamp();

      // if port is not connected, default to no conversion
      Fw::Time convTime = stamp;

      if (this->isConnected_convertTime_OutputPort(0)) {
          bool success = false;
          convTime = this->convertTime_out(0, stamp, TB_WORKSTATION_TIME, 0, success);
          if (!success) {
              // TODO(Mereweth) - EVR
              return;
          }
      }
      
      h.setstamp(convTime);
      imu.setheader(h);
      
      const I64 usecHLOS = (I64) convTime.getSeconds() * 1000LL * 1000LL + (I64) convTime.getUSeconds();
      if (m_initialized && m_activated) {
#ifdef BUILD_SDFLIGHT
          mvVISLAM_AddAccel(m_mvVISLAMPtr, usecHLOS * 1000LL,
                            imu.getlinear_acceleration().getx(),
                            imu.getlinear_acceleration().gety(),
                            imu.getlinear_acceleration().getz());
          mvVISLAM_AddGyro(m_mvVISLAMPtr, usecHLOS * 1000LL,
                           imu.getangular_velocity().getx(),
                           imu.getangular_velocity().gety(),
                           imu.getangular_velocity().getz());
#endif //BUILD_SDFLIGHT
      }

      if (isConnected_ImuFwd_OutputPort(0)) {
          ImuFwd_out(0, imu);
      }
  }

  void MVVislamComponentImpl ::
    ImageIn_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::Image &Image
    )
  {
      Fw::Buffer data = Image.getdata();
      if (m_initialized && m_activated) {
#ifdef BUILD_SDFLIGHT
          Fw::Time hlosTime = Image.getheader().getstamp();
          I64 usecHLOS = hlosTime.getSeconds() * 1000LL * 1000LL + hlosTime.getUSeconds();
          mvVISLAM_AddImage(m_mvVISLAMPtr, usecHLOS * 1000LL, (U8*) (data.getdata()));
          mvVISLAMPose vio_pose = mvVISLAM_GetPose(m_mvVISLAMPtr);
	  
          /*
            - \b 0:  Reset: cov not pos definite
            - \b 1:  Reset: IMU exceeded range
            - \b 2:  Reset: IMU bandwidth too low
            - \b 3:  Reset: not stationary at initialization
            - \b 4:  Reset: no features for x seconds
            - \b 5:  Reset: insufficient constraints from features
            - \b 6:  Reset: failed to add new features
            - \b 7:  Reset: exceeded instant velocity uncertainty
            - \b 8:  Reset: exceeded velocity uncertainty over window
            - \b 10:  Dropped IMU samples
            - \b 11:  Intrinsic camera cal questionable
            - \b 12:  Insufficient number of good features to initialize
            - \b 13:  Dropped camera frame
            - \b 14:  Dropped GPS velocity sample
            - \b 15:  Sensor measurements with uninitialized time stamps or uninitialized uncertainty (set to 0)
          */

          //TODO(mereweth) - move these after transform
          this->x_b.setx(vio_pose.bodyPose.matrix[0][3]);
          this->x_b.sety(vio_pose.bodyPose.matrix[1][3]);
          this->x_b.setz(vio_pose.bodyPose.matrix[2][3]);

          this->m_errorCode = vio_pose.errorCode;
          if ((!this->m_errorCode) &&
              (MV_TRACKING_STATE_HIGH_QUALITY == vio_pose.poseQuality)) {
              using namespace Eigen;
              using namespace ROS::std_msgs;
              using namespace ROS::mav_msgs;
              using namespace ROS::geometry_msgs;
              Vector3f grav(vio_pose.gravity[0], vio_pose.gravity[1], vio_pose.gravity[2]);
              grav.normalize();
              Transform<float,3,Affine> odom_to_imu(AngleAxisf(3.14159, Vector3f::UnitY())
                                                    * AngleAxisf(acos(grav.dot(Vector3f::UnitZ())),
                                                                 grav.cross(Vector3f::UnitZ())));
              // NOTE(Mereweth) - odom_to_imu is odom_to_imu-start (MV spatial) here,
              // and velocity is expressed in MV spatial, so this is correct to get
              // to odom frame
              Vector3f vel(vio_pose.velocity[0], vio_pose.velocity[1], vio_pose.velocity[2]);
              vel = odom_to_imu * vel;
              
              Transform<float,3,Affine> imu_start_to_imu;
              imu_start_to_imu.matrix() << vio_pose.bodyPose.matrix[0][0],
                vio_pose.bodyPose.matrix[0][1],
                vio_pose.bodyPose.matrix[0][2],
                vio_pose.bodyPose.matrix[0][3],
                vio_pose.bodyPose.matrix[1][0],
                vio_pose.bodyPose.matrix[1][1],
                vio_pose.bodyPose.matrix[1][2],
                vio_pose.bodyPose.matrix[1][3],
                vio_pose.bodyPose.matrix[2][0],
                vio_pose.bodyPose.matrix[2][1],
                vio_pose.bodyPose.matrix[2][2],
                vio_pose.bodyPose.matrix[2][3],
                0.0, 0.0, 0.0, 1.0;
              odom_to_imu = odom_to_imu * imu_start_to_imu;
              
              const Quaternionf odom_to_imu_q(odom_to_imu.rotation());
              const Vector3f odom_to_imu_v(odom_to_imu.translation());

              // if port is not connected, default to no conversion
              Fw::Time convTime = hlosTime;

              if (this->isConnected_convertTime_OutputPort(0)) {
                  bool success = false;
                  convTime = this->convertTime_out(0, hlosTime, this->m_tbDes, 0, success);
                  if (!success) {
                      // TODO(Mereweth) - EVR
                      return;
                  }
              }
	      
              ImuStateUpdateNoCov update(
                  Header(Image.getheader().getseq(),
                         convTime,
                         0),
                  0, // TODO(mereweth) - child id
                  Pose(Point(odom_to_imu_v(0),
                             odom_to_imu_v(1),
                             odom_to_imu_v(2)),
                       ROS::geometry_msgs::Quaternion(odom_to_imu_q.x(),
                                                      odom_to_imu_q.y(),
                                                      odom_to_imu_q.z(),
                                                      odom_to_imu_q.w())),
                  Twist(Vector3(vel(0), vel(1), vel(2)),
                        Vector3(vio_pose.angularVelocity[0],
                                vio_pose.angularVelocity[1],
                                vio_pose.angularVelocity[2])),
                  Vector3(vio_pose.wBias[0],
                          vio_pose.wBias[1],
                          vio_pose.wBias[2]),
                  Vector3(vio_pose.aBias[0],
                          vio_pose.aBias[1],
                          vio_pose.aBias[2])
              ); // end ImuStateUpdate constructor

              for (int i = 0; i < NUM_IMUSTATEUPDATE_OUTPUT_PORTS; i++) {
                  if (isConnected_ImuStateUpdate_OutputPort(i)) {
                      ImuStateUpdate_out(i, update);
                  }
              }
          } // if !errorCode

          int num_points = mvVISLAM_HasUpdatedPointCloud(m_mvVISLAMPtr);
          //std::vector<mvVISLAMMapPoint> map_points(num_points, {0});
          //int num_received = mvVISLAM_GetPointCloud(m_mvVISLAMPtr, map_points.data(), num_points);
          //PublishMapPoints(map_points, msg->header.seq, msg->header.stamp);

#endif //BUILD_SDFLIGHT
      }

      ImageBufferReturn_out(0, data);
  }

  void MVVislamComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      tlmWrite_MVVISLAM_ErrorCode(m_errorCode);
      tlmWrite_MVVISLAM_x_b(x_b);
  }

  void MVVislamComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
      pingOut_out(0, key);
  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void MVVislamComponentImpl ::
    MVVISLAM_Reinit_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
#ifdef BUILD_SDFLIGHT
      if (NULL != m_mvVISLAMPtr) {
          mvVISLAM_Deinitialize(m_mvVISLAMPtr);
      }
#endif //BUILD_SDFLIGHT
      m_initialized = false;
      initHelper();
      if (m_initialized) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
      else {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
      }
  }

  void MVVislamComponentImpl ::
    MVVISLAM_Activate_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        bool enable
    )
  {
      if (enable) {
          if (m_initialized) {
              m_activated = true;
              this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
          }
          else {
              this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          }
      }
      else {
          m_activated = false;
    	  this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
  }
  
  void MVVislamComponentImpl ::
    MVVISLAM_Reset_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
      if (m_initialized) {
#ifdef BUILD_SDFLIGHT
      	  mvVISLAM_Reset(m_mvVISLAMPtr, true);
#endif //BUILD_SDFLIGHT
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
      else {
	      this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
      }
  }

} // end namespace SnapdragonFlight
