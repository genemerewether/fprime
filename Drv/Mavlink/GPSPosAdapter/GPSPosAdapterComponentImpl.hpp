// ======================================================================
// \title  GPSPosAdapterComponentImpl.hpp
// \author decoy
// \brief  hpp file for GPSPosAdapter component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#ifndef GPSPosAdapter_HPP
#define GPSPosAdapter_HPP

#include "Drv/Mavlink/GPSPosAdapter/GPSPosAdapterComponentAc.hpp"
#include <Drv/Mavlink/c_library_v2/common/mavlink.h>
#include <Drv/Mavlink/c_library_v2/mavlink_types.h>
#include <Drv/Mavlink/c_library_v2/mavlink_helpers.h>
//#include <Drv/Mavlink/c_library_v2/common/mavlink_msg_set_position_target_local_ned.h>

// #include "ros/ros.h"
// #include "mav_msgs/FlatOutput.h"
// #include "Os/Task.hpp"
// #include "Os/Mutex.hpp"


/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

                                                // bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

#define MAVLINK_MSG_SET_FLATOUTPUT_TARGET_LOCAL_NED_POSITION   0b0000100111111000

namespace Drv {

  class GPSPosAdapterComponentImpl :
    public GPSPosAdapterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object GPSPosAdapter
      //!
      GPSPosAdapterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object GPSPosAdapter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object GPSPosAdapter
      //!
      ~GPSPosAdapterComponentImpl(void);

    PRIVATE:

      mavlink_message_t current_message;
      int system_id;
      int autopilot_id;

      // Member variables for position and attitude from Pixhawk
      mavlink_local_position_ned_t pos;
      mavlink_attitude_t att;
      //mavlink_set_position_target_local_ned_t sp;
      
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Guid
      //!
      void Guid_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
      );

      //! Handler implementation for Nav
      //!
      void Nav_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for SerReadPort
      //!
      void SerReadPort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          SerialReadStatus &status /*!< Status of read*/
      );

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------
      
      // struct FlatOutSet {
      //     Os::Mutex mutex; //! Mutex lock to guard flat output object
      //     ROS::mav_msgs::FlatOutput flatOutput; //! flat output object
      //     bool fresh; //! Whether object has been updated
      //     NATIVE_UINT_TYPE overflows; //! Number of times port overwritten
      // } m_flatOutSet[NUM_FLATOUTPUT_OUTPUT_PORTS];
      
      ROS::mav_msgs::FlatOutput GuidFlat;
      ROS::mav_msgs::FlatOutput NavFlat;
      // ROS::mav_msgs::FlatOutput &PX4Flat;
      // ROS::mav_msgs::FlatOutput DesFlat; Now constructing as a copy of Guid in source
      
      bool Guid_new = false, Nav_new = false, PX4_new = false, init_offset = true;
      
      double dpsi = 0.0, dz = 0.0, dx = 0.0, dy = 0.0;
      double des_x = 0.0, des_y = 0.0, des_z = 0.0, des_psi = 0.0;

      // ROS::geometry_msgs::Point nav_r;
      // ROS::geometry_msgs::Point gui_r;
      // ROS::geometry_msgs::Point px4_r;
      
      // ROS::geometry_msgs::Point des_r;
      // ROS::geometry_msgs::Vector3 des_v;
      // ROS::geometry_msgs::Vector3 des_a;
    };

} // end namespace Drv

#endif
