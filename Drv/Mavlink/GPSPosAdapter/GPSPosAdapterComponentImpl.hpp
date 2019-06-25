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

// #include "ros/ros.h"
// #include "mav_msgs/FlatOutput.h"
// #include "Os/Task.hpp"
// #include "Os/Mutex.hpp"

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

      // Member variables for position and attitude frmo Pixhawk
      mavlink_local_position_ned_t pos;
      mavlink_attitude_t att;

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
