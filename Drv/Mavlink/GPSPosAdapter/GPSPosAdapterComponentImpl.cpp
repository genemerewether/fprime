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
    U8* buf = (U8*) serBuffer.getdata();
    //U32 size = serBuffer.getsize();
    int size = serBuffer.getsize();
    
    // declare Mavlink parsing variablemessages
    mavlink_message_t message;
    mavlink_status_t status_comm;
    for (int i = 0; i < size; i++) {
      printf("%x\n", buf[i]);
      
      // parse message
      mavlink_parse_char(MAVLINK_COMM_1, buf[i], &message, &status_comm);

    //} include switch in the loop
    // Unpack message msgReceived
			// Handle Message ID for position and attitude only
			switch (message.msgid)
			{
				case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
				{printf("inside POSITION_NED");
					//printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
					//mavlink_msg_local_position_ned_decode(&message, &(message.local_position_ned));
          mavlink_local_position_ned_t posNew;
          mavlink_local_position_ned_t * posNewPoint = &posNew;
          const mavlink_message_t * newMessage = &message;
          posNewPoint->x = mavlink_msg_local_position_ned_get_x(newMessage);
          int i = 0;
          printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f] \n", i, posNewPoint->x,posNewPoint->x,posNewPoint->x);

          //pos.x = message.local_position_ned.x;
          //pos.y = message.local_position_ned.y;
          //pos.z = message.local_position_ned.z;
					break;
				}

        case MAVLINK_MSG_ID_ATTITUDE:
				{
					//printf("MAVLINK_MSG_ID_ATTITUDE\n");
					//mavlink_msg_attitude_decode(&message, &(message.attitude));
          //att.yaw = message.attitude.yaw;
					break;
				}

				default:
				{
					// printf("Warning, did not handle message id %i\n",message.msgid);
					break;
				}
      }
    }
      //printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f, % .4f ] \n", i, pos.x, pos.y, pos.z, att.yaw);

  }

} // end namespace Drv
