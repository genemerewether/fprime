// ====================================================================== 
// \title  FlatOutputAdapter.hpp
// \author decoy
// \brief  cpp file for FlatOutputAdapter test harness implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
// 
// ====================================================================== 

#include "Tester.hpp"

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      FlatOutputAdapterGTestBase("Tester", MAX_HISTORY_SIZE),
      component("FlatOutputAdapter")
#else
      FlatOutputAdapterGTestBase(MAX_HISTORY_SIZE),
      component()
#endif
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester(void) 
  {
    
  }

  // ----------------------------------------------------------------------
  // Tests 
  // ----------------------------------------------------------------------

  void Tester ::
    inputFlatOutputTest(void) 
  {
    printf("In Flatoutput Tests\n");

    printf("Before Init\n");
    this->init();
    printf("After Init\n");
    Fw::Time time__t;
    ROS::std_msgs::Header header(10,time__t,0);

    ROS::mav_msgs::Se3FeedForward inSe3FF(header,ROS::geometry_msgs::Point(0.0,0.0,0.0),
                                               ROS::geometry_msgs::Vector3(0.0,0.0,0.0),
                                               ROS::geometry_msgs::Vector3(0.0,0.0,0.0),
                                               //ROS::geometry_msgs::Quaternion(0.2996729,0.0574224,0.4055504,0.8616424),
                                               ROS::geometry_msgs::Quaternion(0.0,0.0,0.7071068,0.7071068),
                                               ROS::geometry_msgs::Vector3(0.0,0.0,0.0),
                                               ROS::geometry_msgs::Vector3(0.0,0.0,0.0));
    printf("Before Invoke new\n");
    this->invoke_to_se3Cmd(0,inSe3FF);
    printf("After Invoke\n");
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_flatOutput_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::FlatOutput &FlatOutput
    )
  {
    this->pushFromPortEntry_flatOutput(FlatOutput);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // se3Cmd
    this->connect_to_se3Cmd(
        0,
        this->component.get_se3Cmd_InputPort(0)
    );

    // CmdDisp
    this->connect_to_CmdDisp(
        0,
        this->component.get_CmdDisp_InputPort(0)
    );

    // flatOutput
    this->component.set_flatOutput_OutputPort(
        0, 
        this->get_from_flatOutput(0)
    );

    // CmdStatus
    this->component.set_CmdStatus_OutputPort(
        0, 
        this->get_from_CmdStatus(0)
    );

    // CmdReg
    this->component.set_CmdReg_OutputPort(
        0, 
        this->get_from_CmdReg(0)
    );

    // ParamGet
    this->component.set_ParamGet_OutputPort(
        0, 
        this->get_from_ParamGet(0)
    );

    // ParamSet
    this->component.set_ParamSet_OutputPort(
        0, 
        this->get_from_ParamSet(0)
    );




  }

  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        INSTANCE
    );
  }

} // end namespace Gnc
