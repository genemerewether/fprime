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

#define PI 3.14159265

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
    
    if(Guid_new && Nav_new && PX4_new)
    {
      // if (!std::isfinite(GuidFlat.getHeader().stamp.sec) ||
      //     !std::isfinite(GuidFlat.getHeader().stamp.nsec)) {
      //     //TODO(mereweth) - EVR
      //     return;
      // }
  
      if (!__builtin_isfinite(GuidFlat.getposition().getx()) ||
          !__builtin_isfinite(GuidFlat.getposition().gety()) ||
          !__builtin_isfinite(GuidFlat.getposition().getz()) ||
          !__builtin_isfinite(GuidFlat.getyaw()) ||
  
          !__builtin_isfinite(NavFlat.getposition().getx()) ||
          !__builtin_isfinite(NavFlat.getposition().gety()) ||
          !__builtin_isfinite(NavFlat.getposition().getz()) ||
          !__builtin_isfinite(NavFlat.getyaw())) {
          //TODO(mereweth) - EVR
          return;
      }

      if(init_offset)
      {
        dpsi = att.yaw - NavFlat.getyaw();
        dz = pos.z - NavFlat.getposition().getz();
        dx = pos.x - (NavFlat.getposition().getx()*cos(PI*dpsi/180.) - NavFlat.getposition().gety()*sin(PI*dpsi/180.));
        dy = pos.y - (NavFlat.getposition().getx()*sin(PI*dpsi/180.) + NavFlat.getposition().gety()*cos(PI*dpsi/180.));

        init_offset = false;
      }
      
      des_x = GuidFlat.getposition().getx()*cos(PI*dpsi/180.) - GuidFlat.getposition().gety()*sin(PI*dpsi/180.) - dx;
      des_y = GuidFlat.getposition().getx()*sin(PI*dpsi/180.) + GuidFlat.getposition().gety()*cos(PI*dpsi/180.) - dy;
      des_z = GuidFlat.getposition().getz() - dz;
      des_psi = GuidFlat.getyaw() - dpsi;
      
      // Header h = GuidFlat.getheader();
      // 
      // FlatOutput DesFlat(Header(h.getseq(),
      //    Fw::Time(TB_ROS_TIME,0,msg->header.stamp.sec,msg->header.stamp.nsec/1000),0),
      //    Point(des_x,des_y,des_z),
      //    Vector3(0.0,0.0,0.0),
      //    Vector3(0.0,0.0,0.0),
      //    des_psi);

      Guid_new = false;
      Nav_new = false;
      PX4_new = false;
    }

    // // Output the desired Pose in GPS_PX4 frame
    // GPSPosAdapterComponentBase::Des_out(0,DesFlat);

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
    // Nothing to do here?
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

        // Handle Message ID for position and attitude
        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
          // decode position
          mavlink_local_position_ned_t posNew;
          mavlink_msg_local_position_ned_decode(&message, &posNew);
          // assign coordinates to member variables
          pos.x = posNew.x;
          pos.y = posNew.y;
          pos.z = posNew.z;
          break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
          // decode attitude
          mavlink_attitude_t attNew;
          mavlink_msg_attitude_decode(&message, &attNew);
          //assign yaw to member variable
          att.yaw = attNew.yaw;
          break;
        }

        default:
        {
          //printf("Warning, did not handle message id %i\n",message.msgid);
          break;
        }
        }
        printf("CURRENT POSITION XYZ & YAW = [ % .4f , % .4f , % .4f, % .4f ] \n", pos.x, pos.y, pos.z, att.yaw);
      }
    }
  }
} // end namespace Drv
