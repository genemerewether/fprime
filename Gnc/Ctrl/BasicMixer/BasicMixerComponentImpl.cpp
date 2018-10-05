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

//#include <Eigen/Eigen>

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  BasicMixerComponentImpl ::
#if FW_OBJECT_NAMES == 1
    BasicMixerComponentImpl(
        const char *const compName
    ) :
      BasicMixerComponentBase(compName),
#else
    BasicMixerImpl(void) :
      BasicMixerImpl(void),
#endif
      basicMixer()
  {

    //NOTE(mgardie) - is there a better way of doing this? Use same mrModel as leeControl?
    Eigen::MatrixXd mixer;
    mixer.resize(4, 6);
    mixer <<
     9.18972e-07,  1.83794e-06,  9.18972e-07, -9.18972e-07, -1.83794e-06, -9.18972e-07,
    -1.59171e-06, -8.99966e-18,  1.59171e-06,  1.59171e-06, -8.99966e-18, -1.59171e-06,
    -1.36777e-07,  1.36777e-07, -1.36777e-07,  1.36777e-07, -1.36777e-07,  1.36777e-07,
     8.54858e-06,  8.54858e-06,  8.54858e-06,  8.54858e-06,  8.54858e-06,  8.54858e-06;

     (void) basicMixer.SetMixer(mixer);

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

      this->basicMixer.SetTorqueThrustDes(Eigen::Vector3d(
                                            moment_b.getz(),
                                            moment_b.gety(),
                                            moment_b.getz()),
                                          Eigen::Vector3d(
                                            thrust_b.getx(),
                                            thrust_b.gety(),
                                            thrust_b.getz()));
      Eigen::VectorXd rotorVel;
      this->basicMixer.GetRotorVelCommand(&rotorVel);

       F64 angVel[6], angles[0], normalized[0];
       for (int i = 0; i < 6; i ++) {
          angVel[i] = rotorVel(i);
       }

       ROS::std_msgs::Header h = TorqueThrust.getheader();
       ROS::mav_msgs::Actuators rotorVel__comm(h, angles, 0, angVel, 6, normalized, 0);
       if (this->isConnected_motor_OutputPort(0)) {
         this->motor_out(0, rotorVel__comm);
       }
  }

  void BasicMixerComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // NOTE(mgardine) - Output port now called in controls_handler
  }

} // end namespace Gnc
