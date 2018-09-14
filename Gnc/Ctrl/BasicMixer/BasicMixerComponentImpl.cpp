// ======================================================================
// \title  BasicMixerImpl.cpp
// \author mereweth
// \brief  cpp file for BasicMixer component implementation class
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


#include <Gnc/Ctrl/BasicMixer/BasicMixerComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Eigen/Eigen>

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  BasicMixerComponentImpl ::
#if FW_OBJECT_NAMES == 1
    BasicMixerComponentImpl(
        const char *const compName
    ) :
      BasicMixerComponentBase(compName)
#else
    BasicMixerImpl(void)
#endif
  {

  }

  void BasicMixerComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    BasicMixerComponentBase::init(instance);
  }

  BasicMixerComponentImpl ::
    ~BasicMixerComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void BasicMixerComponentImpl ::
    controls_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::TorqueThrust &TorqueThrust
    )
  {
      using ROS::geometry_msgs::Vector3;

      Vector3 moment_b = TorqueThrust.gettorque();
      Vector3 thrust_b = TorqueThrust.getthrust();

      // NOTE(mereweth) - we can only apply thrust in z body frame
      Eigen::Vector4d controls(moment_b.getx(),
                               moment_b.gety(),
                               moment_b.getz(),
                               thrust_b.getz());

      Eigen::MatrixXd mixer;
      mixer.resize(4, 6);
      mixer <<
       9.18972e-07,  1.83794e-06,  9.18972e-07, -9.18972e-07, -1.83794e-06, -9.18972e-07,
      -1.59171e-06, -8.99966e-18,  1.59171e-06,  1.59171e-06, -8.99966e-18, -1.59171e-06,
      -1.36777e-07,  1.36777e-07, -1.36777e-07,  1.36777e-07, -1.36777e-07,  1.36777e-07,
       8.54858e-06,  8.54858e-06,  8.54858e-06,  8.54858e-06,  8.54858e-06,  8.54858e-06;

       // TODO(mereweth) - calculate pseudoinverse in init

       Eigen::MatrixXd mixerPinv = mixer.transpose() * (mixer * mixer.transpose()).inverse();

       Eigen::VectorXd rotorVel = mixerPinv * controls;

       // TODO(mereweth) - store actuator values
  }

  void BasicMixerComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO(mereweth) - call Actuators output port
  }

} // end namespace Gnc
