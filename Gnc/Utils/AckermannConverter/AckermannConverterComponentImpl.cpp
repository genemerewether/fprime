// ====================================================================== 
// \title  AckermannConverterImpl.cpp
// \author mereweth
// \brief  cpp file for AckermannConverter component implementation class
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


#include <Gnc/Utils/AckermannConverter/AckermannConverterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  AckermannConverterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    AckermannConverterComponentImpl(
        const char *const compName
    ) :
      AckermannConverterComponentBase(compName)
#else
    AckermannConverterImpl(void)
#endif
  {

  }

  void AckermannConverterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    AckermannConverterComponentBase::init(instance);
  }

  AckermannConverterComponentImpl ::
    ~AckermannConverterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void AckermannConverterComponentImpl ::
    ackermann_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::ackermann_msgs::AckermannDriveStamped &AckermannDriveStamped
    )
  {
      F64 steering_angle[1] = { AckermannDriveStamped.getdrive().getsteering_angle() };
      F64 speed[1] = { AckermannDriveStamped.getdrive().getspeed() };

      ROS::mav_msgs::Actuators actObj;
      actObj.setheader(AckermannDriveStamped.getheader());
      I32 size = 1;
      actObj.setangles(steering_angle, size);
      actObj.setangles_count(1);
      size = 1;
      actObj.setangular_velocities(speed, size);
      actObj.setangular_velocities_count(1);
      actObj.setnormalized_count(0);
      
      this->actuators_out(0, actObj);
  }

} // end namespace Gnc
