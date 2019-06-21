// ====================================================================== 
// \title  GPSPosAdapterImpl.cpp
// \author mereweth
// \brief  cpp file for GPSPosAdapter component implementation class
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


#include <Drv/Mavlink/GPSPosAdapter/GPSPosAdapterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <stdio.h>

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
    GPSPosAdapterImpl(void)
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
    // TODO
  }

  void GPSPosAdapterComponentImpl ::
    Nav_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    // TODO
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
