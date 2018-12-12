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
      basicMixer(),
      paramsInited(false)
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
  
  void BasicMixerComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
    printf("prm %d updated\n", id);
  }
  
  void BasicMixerComponentImpl ::
    parametersLoaded()
  {
      Fw::ParamValid valid[4];
      Eigen::MatrixXd mixer;
      mixer.resize(4, paramGet_numRotors(valid[0]));
      if (Fw::PARAM_VALID != valid[0]) {  return;  }
    
      for (U32 i = 0; i < mixer.cols(); i++) {
          switch (i) {
              case 0:
                  mixer.col(i) << paramGet_m_x__1(valid[0]),
                                  paramGet_m_y__1(valid[1]),
                                  paramGet_m_z__1(valid[2]),
                                  paramGet_t__1(valid[3]);
                  break;
              case 1:
                  mixer.col(i) << paramGet_m_x__2(valid[0]),
                                  paramGet_m_y__2(valid[1]),
                                  paramGet_m_z__2(valid[2]),
                                  paramGet_t__2(valid[3]);
                  break;
              case 2:
                  mixer.col(i) << paramGet_m_x__3(valid[0]),
                                  paramGet_m_y__3(valid[1]),
                                  paramGet_m_z__3(valid[2]),
                                  paramGet_t__3(valid[3]);
                  break;
              case 3:
                  mixer.col(i) << paramGet_m_x__4(valid[0]),
                                  paramGet_m_y__4(valid[1]),
                                  paramGet_m_z__4(valid[2]),
                                  paramGet_t__4(valid[3]);
                  break;
              case 4:
                  mixer.col(i) << paramGet_m_x__5(valid[0]),
                                  paramGet_m_y__5(valid[1]),
                                  paramGet_m_z__5(valid[2]),
                                  paramGet_t__5(valid[3]);
                  break;
              case 5:
                  mixer.col(i) << paramGet_m_x__6(valid[0]),
                                  paramGet_m_y__6(valid[1]),
                                  paramGet_m_z__6(valid[2]),
                                  paramGet_t__6(valid[3]);
                  break;
              default:
                  FW_ASSERT(0, i);
          }

          for (U32 j = 0; j < 4; j++) {
              if (Fw::PARAM_VALID != valid[j]) {  return;  }
          }
      }
      
      (void) basicMixer.SetMixer(mixer);

      paramsInited = true;
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
      if (!paramsInited) {
          return;
      }
    
      using ROS::geometry_msgs::Vector3;

      Vector3 moment_b = TorqueThrust.gettorque();
      Vector3 thrust_b = TorqueThrust.getthrust();

      this->basicMixer.SetTorqueThrustDes(Eigen::Vector3d(
                                            moment_b.getx(),
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
      this->tlmWrite_BMIX_Rot0(angVel[0]);
      this->tlmWrite_BMIX_Rot1(angVel[1]);
      this->tlmWrite_BMIX_Rot2(angVel[2]);
      this->tlmWrite_BMIX_Rot3(angVel[3]);
      this->tlmWrite_BMIX_Rot4(angVel[4]);
      this->tlmWrite_BMIX_Rot5(angVel[5]);

      ROS::std_msgs::Header h = TorqueThrust.getheader();
      ROS::mav_msgs::Actuators rotorVel__comm(h, angles, 0, 0, angVel, 6, 6, normalized, 0, 0);
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

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void BasicMixerComponentImpl ::
    BMIX_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
      this->parametersLoaded();
      if (this->paramsInited) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
      else {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
      }
  }

  
} // end namespace Gnc
