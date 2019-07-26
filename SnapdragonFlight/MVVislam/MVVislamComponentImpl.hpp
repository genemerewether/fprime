// ====================================================================== 
// \title  MVVislamImpl.hpp
// \author mereweth
// \brief  hpp file for MVVislam component implementation class
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

#ifndef MVVislam_HPP
#define MVVislam_HPP

#include "SnapdragonFlight/MVVislam/MVVislamComponentAc.hpp"

#ifdef BUILD_SDFLIGHT
#include "mv.h"
#include "mvVISLAM.h"
#endif

namespace SnapdragonFlight {

  class MVVislamComponentImpl :
    public MVVislamComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MVVislam
      //!
      MVVislamComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object MVVislam
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object MVVislam
      //!
      ~MVVislamComponentImpl(void);

      void setTBDes(TimeBase tbDes);
    
    PRIVATE:

      //! Preamble override
      void preamble(void);
      //! Finalizer override
      void finalizer(void);

      // add IMU packet to Vislam; converts time as side-effect
      void addImuHelper(ROS::sensor_msgs::ImuNoCov &imu);
    
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Imu
      //!
      void Imu_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &imu
      );

      //! Handler implementation for BatchImu
      //!
      void BatchImu_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::BatchImu &BatchImu 
      );

      //! Handler implementation for Image
      //!
      void ImageIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::Image &Image 
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

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations 
      // ----------------------------------------------------------------------

      //! Implementation for MVVISLAM_Reinit command handler
      //! 
      void MVVISLAM_Reinit_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      //! Implementation for MVVISLAM_Activate command handler
      //! 
      void MVVISLAM_Activate_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          bool enable 
      );
    
      //! Implementation for MVVISLAM_Reset command handler
      //! 
      void MVVISLAM_Reset_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );
      
      void initHelper();

#ifdef BUILD_SDFLIGHT
      mvVISLAM* m_mvVISLAMPtr;
#endif

      bool m_initialized;
      
      bool m_activated;

      U32 m_errorCode;

      ROS::geometry_msgs::Quaternion w_q_b;
    
      ROS::geometry_msgs::Vector3 x_b;

      TimeBase m_tbDes;

    };

} // end namespace SnapdragonFlight

#endif
