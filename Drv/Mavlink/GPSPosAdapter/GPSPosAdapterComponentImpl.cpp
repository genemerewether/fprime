// ======================================================================
// \title  GPSPosAdapterComponentImpl.cpp
// \author decoy
// \brief  cpp file for GPSPosAdapter component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Drv/Mavlink/GPSPosAdapter/GPSPosAdapterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <stdio.h>
#include <math.h>

//#define PI 3.14159265

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  GPSPosAdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    GPSPosAdapterComponentImpl(
        const char *const compName
    ) :
      GPSPosAdapterComponentBase(compName)
#else
    GPSPosAdapterComponentImpl(void)
#endif
  {
  }

  void GPSPosAdapterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    GPSPosAdapterComponentBase::init(instance);
  }

  GPSPosAdapterComponentImpl ::
    ~GPSPosAdapterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void GPSPosAdapterComponentImpl ::
    Guid_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    GuidFlat = FlatOutput;
    Guid_new = true;

    using namespace ROS::std_msgs;
    using namespace ROS::mav_msgs;
    using namespace ROS::geometry_msgs;
    
    if(Guid_new && Nav_new && receivedGPS)
    {

      if (!__builtin_isfinite(GuidFlat.getposition().getx()) ||
          !__builtin_isfinite(GuidFlat.getposition().gety()) ||
          !__builtin_isfinite(GuidFlat.getposition().getz()) ||
          !__builtin_isfinite(GuidFlat.getyaw()) ||
  
          !__builtin_isfinite(NavFlat.getposition().getx()) ||
          !__builtin_isfinite(NavFlat.getposition().gety()) ||
          !__builtin_isfinite(NavFlat.getposition().getz()) ||
          !__builtin_isfinite(NavFlat.getyaw())) {
          return;
      }

      if(init_offset)
      {
        dpsi = attGPS.yaw - NavFlat.getyaw();
        dz = posGPS.z - NavFlat.getposition().getz();
        dx = posGPS.x - (NavFlat.getposition().getx()*cos(dpsi) - NavFlat.getposition().gety()*sin(dpsi));
        dy = posGPS.y - (NavFlat.getposition().getx()*sin(dpsi) + NavFlat.getposition().gety()*cos(dpsi));

        init_offset = false;
      }
      
      des_x = GuidFlat.getposition().getx()*cos(dpsi) - GuidFlat.getposition().gety()*sin(dpsi) - dx;
      des_y = GuidFlat.getposition().getx()*sin(dpsi) + GuidFlat.getposition().gety()*cos(dpsi) - dy;
      des_z = GuidFlat.getposition().getz() - dz;
      des_psi = GuidFlat.getyaw() - dpsi;
      
      Guid_new = false;
      Nav_new = false;
      PX4_new = false;
    }

    // Send desired flatoutput to the Pixhawk
    sendPosDesGPS(des_x, des_y, des_z, des_psi);

  }

  void GPSPosAdapterComponentImpl ::
    Nav_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    NavFlat = FlatOutput;
    Nav_new = true;
  }

  void GPSPosAdapterComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void GPSPosAdapterComponentImpl ::
    SerReadPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        SerialReadStatus &status
    )
  {
    U8 *buf = (U8 *)serBuffer.getdata();
    //U32 size = serBuffer.getsize();
    int size = serBuffer.getsize();

    // declare Mavlink parsing variable
    mavlink_message_t message;
    mavlink_status_t status_comm;

    for (int i = 0; i < size; i++)
    {
      // parse message checking for result
      if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &message, &status_comm))
      {
        
        current_message = message;
        // capture system and companion ids
        system_id = message.sysid;
        autopilot_id = message.compid;

        // set received gps flag to true
        receivedGPS = true;

        // Handle Message ID for position and attitude
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
          // decode position
          mavlink_local_position_ned_t posNewGPS;
          mavlink_msg_local_position_ned_decode(&message, &posNewGPS);
          // assign coordinates to member variables
          posGPS.x = posNewGPS.x;
          posGPS.y = posNewGPS.y;
          posGPS.z = posNewGPS.z;
          break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
          // decode attitude
          mavlink_attitude_t attNewGPS;
          mavlink_msg_attitude_decode(&message, &attNewGPS);
          //assign yaw to member variable
          attGPS.yaw = attNewGPS.yaw;
          break;
        }

        default:
        {
          //printf("Warning, did not handle message id %i\n",message.msgid);
          break;
        }
        }
      }
    }
  }

  void GPSPosAdapterComponentImpl::sendPosDesGPS(float xDesGPS, float yDesGPS, float zDesGPS, float yawDesGPS) {
    // write desired pos to GPS
    printf("DESIRED POSITION XYZ & YAW GPS = [ % .4f , % .4f , % .4f, % .4f ] \n", xDesGPS, yDesGPS, zDesGPS, yawDesGPS);
    
    // declare and update setpoint struct with target XYZ in local NED frame in meters
    mavlink_set_position_target_local_ned_t sp;
    sp.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;
    sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
    sp.x = xDesGPS;
    sp.y = yDesGPS;
    sp.z = zDesGPS;

    // update yaw target
    sp.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;
    sp.yaw = yawDesGPS;

    // PACK PAYLOAD
	  //   ENCODE
    mavlink_message_t message;
    companion_id = 0; // companion computer component id
    mavlink_msg_set_position_target_local_ned_encode(system_id, companion_id, &message, &sp);
    }

    //   WRITE
    // TO DO: write message
    char buffer[300]; //setup buffer
    mavlink_msg_to_send_buffer((uint8_t*)buffer, &message);
    SerWritePort_out(0, buffer);

} // end namespace Drv
