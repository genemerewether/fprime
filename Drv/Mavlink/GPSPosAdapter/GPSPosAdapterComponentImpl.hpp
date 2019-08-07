// ====================================================================== 
// \title  GPSPosAdapterImpl.hpp
// \author mereweth
// \brief  hpp file for GPSPosAdapter component implementation class
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


#ifndef GPSPosAdapter_HPP
#define GPSPosAdapter_HPP

#include "Drv/Mavlink/GPSPosAdapter/GPSPosAdapterComponentAc.hpp"
#include <Drv/Mavlink/c_library_v2/common/mavlink.h>
#include <Drv/Mavlink/c_library_v2/mavlink_types.h>
#include <Drv/Mavlink/c_library_v2/mavlink_helpers.h>

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111

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
      // Member variables for position and attitude from Pixhawk
      mavlink_local_position_ned_t posGPS;
      mavlink_attitude_t attGPS;
      bool receivedGPS = false;
      int system_id; // system id
	    int autopilot_id; // autopilot component id
	    int companion_id; // companion computer component id

      // member function to send  desired GPS coordinates through output port
      void sendPosDesGPS(float xDesGPS, float yDesGPS, float zDesGPS, float yawDesGPS);

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

    };

} // end namespace Drv

#endif
