// ======================================================================
// \title  FlatOutputAdapterComponentImpl.cpp
// \author decoy
// \brief  cpp file for FlatOutputAdapter component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Gnc/Utils/FlatOutputAdapter/FlatOutputAdapterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Eigen/Geometry>

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  FlatOutputAdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    FlatOutputAdapterComponentImpl(
        const char *const compName
    ) :
      FlatOutputAdapterComponentBase(compName)
#else
    FlatOutputAdapterComponentImpl(void)
#endif
  {

  }

  void FlatOutputAdapterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    FlatOutputAdapterComponentBase::init(instance);
  }

  FlatOutputAdapterComponentImpl ::
    ~FlatOutputAdapterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void FlatOutputAdapterComponentImpl ::
    se3Cmd_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Se3FeedForward &Se3FeedForward
    )
  {
      using namespace Eigen;

      ROS::geometry_msgs::Quaternion quat1(Se3FeedForward.getattitude());
      
      Eigen::Quaterniond quat2(quat1.getw(),quat1.getx(),quat1.gety(),quat1.getz());
      
      quat2.normalize();
      
      Eigen::Vector3d euler = quat2.toRotationMatrix().eulerAngles(0,1,2);

      printf("after euler in the function\n");
      ROS::mav_msgs::FlatOutput flatoutput(Se3FeedForward.getheader(),
                                           Se3FeedForward.getposition(),
                                           Se3FeedForward.getvelocity(),
                                           Se3FeedForward.getacceleration(),
                                           euler(2));
      this->flatOutput_out(0,flatoutput);
  }

} // end namespace Gnc
