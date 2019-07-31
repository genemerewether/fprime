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
      ,m_camCfg()
#endif
      ,m_tbc()
      ,m_ombc()
      ,m_std0tbc()
      ,m_std0ombc()
      ,m_tba()

      ,m_readoutTime(0.0)
      ,m_camDelta(0.0)
      ,m_std0camDelta(0.0)
    
      ,m_accelMeasRange(0.0)
      ,m_gyroMeasRange(0.0)
      ,m_stdAccelMeasNoise(0.0)
      ,m_stdGyroMeasNoise(0.0)
    
      ,m_stdCamNoise(0.0)
      ,m_minStdPixelNoise(0.0)
    
      ,m_failHighPixelNoiseScaleFactor(0.0)
      ,m_logDepthBootstrap(0.0)
      ,m_useLogCameraHeight(false)
      ,m_logCameraHeightBootstrap(0.0)
      ,m_noInitWhenMoving(false)
      ,m_limitedIMUbWtrigger(0.0)
      ,m_staticMaskFilename("na")
      ,m_gpsImuTimeAlignment(0.0)
      ,m_mapping(false)
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
      parameterUpdated(FwPrmIdType id)
    {
        DEBUG_PRINT("prm %d updated\n", id);
        Fw::ParamValid valid;

        switch (id) {
            // default params
          
            case PARAMID_MVVISLAM_INITWHENMOVING:
            {
                bool temp = paramGet_MVVISLAM_initWhenMoving(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_noInitWhenMoving = !temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_MAPPING:
            {
                bool temp = paramGet_MVVISLAM_mapping(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_mapping = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STATICMASKFILENAME:
            {
                Fw::ParamString temp = paramGet_MVVISLAM_staticMaskFilename(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_staticMaskFilename = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_LOGDEPTHBOOTSTRAP:
            {
                F32 temp = paramGet_MVVISLAM_logDepthBootstrap(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_logDepthBootstrap = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_USELOGCAMERAHEIGHT:
            {
                U8 temp = paramGet_MVVISLAM_useLogCameraHeight(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_useLogCameraHeight = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_LOGCAMERAHEIGHTBOOTSTRAP:
            {
                F32 temp = paramGet_MVVISLAM_logCameraHeightBootstrap(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_logCameraHeightBootstrap = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_UVOFFSET:
            {
                U32 temp = paramGet_MVVISLAM_uvOffset(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.uvOffset = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTIONMODEL:
            {
                U32 temp = paramGet_MVVISLAM_distortionModel(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortionModel = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_READOUTTIME:
            {
                F32 temp = paramGet_MVVISLAM_readoutTime(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_readoutTime = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STD0_CAMDELTA:
            {
                F32 temp = paramGet_MVVISLAM_std0_camDelta(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_std0camDelta = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STD0_TBC_X:
            {
                F32 temp = paramGet_MVVISLAM_std0_tbc_x(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_std0tbc[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STD0_TBC_Y:
            {
                F32 temp = paramGet_MVVISLAM_std0_tbc_y(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_std0tbc[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STD0_TBC_Z:
            {
                F32 temp = paramGet_MVVISLAM_std0_tbc_z(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_std0tbc[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STD0_OMBC_X:
            {
                F32 temp = paramGet_MVVISLAM_std0_ombc_x(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_std0ombc[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STD0_OMBC_Y:
            {
                F32 temp = paramGet_MVVISLAM_std0_ombc_y(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_std0ombc[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STD0_OMBC_Z:
            {
                F32 temp = paramGet_MVVISLAM_std0_ombc_z(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_std0ombc[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_TBA_X:
            {
                F32 temp = paramGet_MVVISLAM_tba_x(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_tba[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_TBA_Y:
            {
                F32 temp = paramGet_MVVISLAM_tba_y(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_tba[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_TBA_Z:
            {
                F32 temp = paramGet_MVVISLAM_tba_z(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_tba[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_PIXELWIDTH:
            {
                U32 temp = paramGet_MVVISLAM_pixelWidth(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.pixelWidth = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_PIXELHEIGHT:
            {
                U32 temp = paramGet_MVVISLAM_pixelHeight(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.pixelHeight = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_MEMORYSTRIDE:
            {
                U32 temp = paramGet_MVVISLAM_memoryStride(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.memoryStride = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
 
            // imu <-> camera extrinsics and time alignment
                
            case PARAMID_MVVISLAM_CAMDELTA:
            {
                F32 temp = paramGet_MVVISLAM_camDelta(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camDelta = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_TBC_X:
            {
                F32 temp = paramGet_MVVISLAM_tbc_x(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_tbc[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_TBC_Y:
            {
                F32 temp = paramGet_MVVISLAM_tbc_y(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_tbc[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_TBC_Z:
            {
                F32 temp = paramGet_MVVISLAM_tbc_z(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_tbc[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_OMBC_X:
            {
                F32 temp = paramGet_MVVISLAM_ombc_x(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_ombc[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_OMBC_Y:
            {
                F32 temp = paramGet_MVVISLAM_ombc_y(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_ombc[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_OMBC_Z:
            {
                F32 temp = paramGet_MVVISLAM_ombc_z(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_ombc[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;

            // intrinsics
                
            case PARAMID_MVVISLAM_PRINCIPALPOINT_U:
            {
                F64 temp = paramGet_MVVISLAM_principalPoint_u(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.principalPoint[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_PRINCIPALPOINT_V:
            {
                F64 temp = paramGet_MVVISLAM_principalPoint_v(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.principalPoint[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_FOCALLENGTH_U:
            {
                F64 temp = paramGet_MVVISLAM_focalLength_u(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.focalLength[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_FOCALLENGTH_V:
            {
                F64 temp = paramGet_MVVISLAM_focalLength_v(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.focalLength[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
                
            case PARAMID_MVVISLAM_DISTORTION_0:
            {
                F64 temp = paramGet_MVVISLAM_distortion_0(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[0] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_1:
            {
                F64 temp = paramGet_MVVISLAM_distortion_1(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[1] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_2:
            {
                F64 temp = paramGet_MVVISLAM_distortion_2(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[2] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_3:
            {
                F64 temp = paramGet_MVVISLAM_distortion_3(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[3] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_4:
            {
                F64 temp = paramGet_MVVISLAM_distortion_4(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[4] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_5:
            {
                F64 temp = paramGet_MVVISLAM_distortion_5(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[5] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_6:
            {
                F64 temp = paramGet_MVVISLAM_distortion_6(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[6] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_7:
            {
                F64 temp = paramGet_MVVISLAM_distortion_7(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_camCfg.distortion[7] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_8:
            {
                F64 temp = paramGet_MVVISLAM_distortion_8(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    // NOTE(Mereweth) - only 8 elements in array in header decl
                    //m_camCfg.distortion[8] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_DISTORTION_9:
            {
                F64 temp = paramGet_MVVISLAM_distortion_9(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    // NOTE(Mereweth) - only 8 elements in array in header decl
                    //m_camCfg.distortion[9] = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;

            // image sensor noise characteristics
                
            case PARAMID_MVVISLAM_STDCAMNOISE:
            {
                F32 temp = paramGet_MVVISLAM_stdCamNoise(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_stdCamNoise = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_MINSTDPIXELNOISE:
            {
                F32 temp = paramGet_MVVISLAM_minStdPixelNoise(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_minStdPixelNoise = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_FAILHIGHPIXELNOISESCALEFACTOR:
            {
                F32 temp = paramGet_MVVISLAM_failHighPixelNoiseScaleFactor(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_failHighPixelNoiseScaleFactor = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;

            // imu characteristics
                
            case PARAMID_MVVISLAM_ACCELMEASRANGE:
            {
                F32 temp = paramGet_MVVISLAM_accelMeasRange(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_accelMeasRange = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_GYROMEASRANGE:
            {
                F32 temp = paramGet_MVVISLAM_gyroMeasRange(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_gyroMeasRange = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STDACCELMEASNOISE:
            {
                F32 temp = paramGet_MVVISLAM_stdAccelMeasNoise(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_stdAccelMeasNoise = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_STDGYROMEASNOISE:
            {
                F32 temp = paramGet_MVVISLAM_stdGyroMeasNoise(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_stdGyroMeasNoise = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
            case PARAMID_MVVISLAM_LIMITEDIMUBWTRIGGER:
            {
                F32 temp = paramGet_MVVISLAM_limitedIMUbWtrigger(valid);
                if (Fw::PARAM_VALID == valid) {
#ifdef BUILD_SDFLIGHT
                    m_limitedIMUbWtrigger = temp;
#endif
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
        }
    }

    void MVVislamComponentImpl ::
      parametersLoaded()
    {
        for (U32 i = 0; i < __MAX_PARAMID; i++) {
            parameterUpdated(i);
        }
    }

  void MVVislamComponentImpl ::
    initHelper(void)
  {
#ifdef BUILD_SDFLIGHT
      this->m_mvVISLAMPtr = 
        mvVISLAM_Initialize(&m_camCfg,
                            m_readoutTime,
                            m_tbc,
                            m_ombc,
                            m_camDelta,
                            m_std0tbc,
                            m_std0ombc,
                            m_std0camDelta,
                            m_accelMeasRange,
                            m_gyroMeasRange,
                            m_stdAccelMeasNoise,
                            m_stdGyroMeasNoise,
                            m_stdCamNoise,
                            m_minStdPixelNoise,
                            m_failHighPixelNoiseScaleFactor,
                            m_logDepthBootstrap,
                            m_useLogCameraHeight,
                            m_logCameraHeightBootstrap,
                            m_noInitWhenMoving,
                            m_limitedIMUbWtrigger,
                            m_staticMaskFilename.toChar(),
                            m_gpsImuTimeAlignment,
                            m_tba,
                            m_mapping
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
    addImuHelper(ROS::sensor_msgs::ImuNoCov &imu)
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
  }

  void MVVislamComponentImpl ::
    Imu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &imu
    )
  {
      addImuHelper(imu);
      if (isConnected_ImuFwd_OutputPort(0)) {
          ImuFwd_out(0, imu);
      }
  }

  void MVVislamComponentImpl ::
    BatchImu_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::BatchImu &BatchImu
    )
  {
      NATIVE_INT_TYPE size = 0;
      ROS::sensor_msgs::ImuNoCov rwArray[100];
      const ROS::sensor_msgs::ImuNoCov* imuArray = BatchImu.getsamples(size);
      NATIVE_INT_TYPE setSize = FW_MIN(BatchImu.getsamples_count(), size);
      if (setSize > FW_NUM_ARRAY_ELEMENTS(rwArray)) {
          // TODO(mereweth) - EVR
          return;
      }
      for (U32 ii = 0; ii < setSize; ii++) {
          rwArray[ii] = imuArray[ii];
          addImuHelper(rwArray[ii]);
      }
      
      BatchImu.setsamples(rwArray, setSize);

      if (isConnected_BatchImuFwd_OutputPort(0)) {
          BatchImuFwd_out(0, BatchImu);
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

          DEBUG_PRINT("tbc %0.4f, %0.4f, %0.4f\n", vio_pose.tbc[0], vio_pose.tbc[1], vio_pose.tbc[2]);
          DEBUG_PRINT("Rbc\n[%0.4f, %0.4f, %0.4f\n%0.4f, %0.4f, %0.4f\n%0.4f, %0.4f, %0.4f\n]\n",
                      vio_pose.Rbc[0][0], vio_pose.Rbc[0][1], vio_pose.Rbc[0][2],
                      vio_pose.Rbc[1][0], vio_pose.Rbc[1][1], vio_pose.Rbc[1][2],
                      vio_pose.Rbc[2][0], vio_pose.Rbc[2][1], vio_pose.Rbc[2][2]);
          DEBUG_PRINT("aAccInv\n[%0.4f, %0.4f, %0.4f\n%0.4f, %0.4f, %0.4f\n%0.4f, %0.4f, %0.4f\n]\n",
                      vio_pose.aAccInv[0][0], vio_pose.aAccInv[0][1], vio_pose.aAccInv[0][2],
                      vio_pose.aAccInv[1][0], vio_pose.aAccInv[1][1], vio_pose.aAccInv[1][2],
                      vio_pose.aAccInv[2][0], vio_pose.aAccInv[2][1], vio_pose.aAccInv[2][2]);
          DEBUG_PRINT("aGyrInv\n[%0.4f, %0.4f, %0.4f\n%0.4f, %0.4f, %0.4f\n%0.4f, %0.4f, %0.4f\n]\n",
                      vio_pose.aGyrInv[0][0], vio_pose.aGyrInv[0][1], vio_pose.aGyrInv[0][2],
                      vio_pose.aGyrInv[1][0], vio_pose.aGyrInv[1][1], vio_pose.aGyrInv[1][2],
                      vio_pose.aGyrInv[2][0], vio_pose.aGyrInv[2][1], vio_pose.aGyrInv[2][2]);

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

              // NOTE(Mereweth) - velocity is expressed in MV spatial, so this is correct to get
              // to body
              Vector3f vel(vio_pose.velocity[0], vio_pose.velocity[1], vio_pose.velocity[2]);
              vel = imu_start_to_imu.rotation() * vel;
              
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
