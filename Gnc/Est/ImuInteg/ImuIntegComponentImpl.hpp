// ======================================================================
// \title  ImuIntegImpl.hpp
// \author mereweth
// \brief  hpp file for ImuInteg component implementation class
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

#ifndef ImuInteg_HPP
#define ImuInteg_HPP

#include "Gnc/Est/ImuInteg/ImuIntegComponentAc.hpp"
#include "Gnc/Est/ImuInteg/ImuIntegComponentImplCfg.hpp"

#include "quest_gnc/est/imu_integ.h"
#include "quest_gnc/utils/world_params.h"

namespace Gnc {

  class ImuIntegComponentImpl :
    public ImuIntegComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ImuInteg
      //!
      ImuIntegComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ImuInteg
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ImuInteg
      //!
      ~ImuIntegComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Imu
      //!
      void Imu_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &ImuNoCov
      );

      //! Handler implementation for ImuStateUpdate
      //!
      void ImuStateUpdate_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuStateUpdate &ImuStateUpdate
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      U32 seq;

      // State update in
      ROS::geometry_msgs::Vector3 wBias;

      ROS::geometry_msgs::Vector3 aBias;

      // IMU sample in
      ROS::geometry_msgs::Vector3 omega_b;

      ROS::geometry_msgs::Vector3 a_b;

      // Odometry out
      ROS::geometry_msgs::Point x_w;

      ROS::geometry_msgs::Quaternion w_q_b;

      ROS::geometry_msgs::Vector3 v_b;

      quest_gnc::estimation::ImuInteg imuInteg;

    };

} // end namespace Gnc

#endif
