// ====================================================================== 
// \title  GPSPosAdapter.hpp
// \author mereweth
// \brief  cpp file for GPSPosAdapter test harness implementation class
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <errno.h>

#include "Tester.hpp"

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      GPSPosAdapterGTestBase("Tester", MAX_HISTORY_SIZE),
      component("GPSPosAdapter")
#else
      GPSPosAdapterGTestBase(MAX_HISTORY_SIZE),
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
    uartConnTest(void) 
  {
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
    ASSERT_NE(fd, -1);
    struct termios config;
    int stat = tcgetattr(fd, &config);
    ASSERT_NE(fd, -1);

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
			INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
			ONOCR | OFILL | OPOST);

#ifdef OLCUC
    config.c_oflag &= ~OLCUC;
#endif

#ifdef ONOEOT
    config.c_oflag &= ~ONOEOT;
#endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0

    stat = cfsetispeed(&config, B57600);
    ASSERT_NE(stat, -1);
    stat = cfsetospeed(&config, B57600);
    ASSERT_NE(stat, -1);
    
    stat = tcsetattr(fd, TCSAFLUSH, &config);
    ASSERT_NE(stat, -1);    
    
    U8 buf[1024];
    while (1) {
      int bytes = read(fd, buf, 1024);
      if (-1 == bytes) {
	printf("bad serial read\n");
	continue;
      }

      Fw::Buffer bufObj(0, 0, (U64) buf, bytes);
      Drv::SerialReadStatus serStat = SER_OK;
      this->invoke_to_SerReadPort(0, bufObj, serStat);
    }
  }

  void Tester::
    inputFlatoutputTest(void)
  {
     printf("In Flatoutput Test\n");

     this->init(); //Do I need this?
     printf("First\n");
     ROS::mav_msgs::InputFlatOutputPort flatNav_p;
     ROS::mav_msgs::InputFlatOutputPort flatGuid_p;
     
     Fw::Time time__t;//(TB_WORKSTATION_TIME,10,11);
     time__t.set(1000,1000);
     printf("Time: %lu\n",time__t.getSeconds());

     printf("Firstv1\n");
     ROS::std_msgs::Header header(10,time__t,(char) 0);
     
     printf("Firstv2\n");
     ROS::mav_msgs::FlatOutput flatNav_t(header,ROS::geometry_msgs::Point(0.0,0.0,0.0),ROS::geometry_msgs::Vector3(0.0,0.0,0.0),ROS::geometry_msgs::Vector3(0.0,0.0,0.0),0.0);
     ROS::mav_msgs::FlatOutput flatGuid_t(header,ROS::geometry_msgs::Point(0.0,0.0,0.0),ROS::geometry_msgs::Vector3(0.0,0.0,0.0),ROS::geometry_msgs::Vector3(0.0,0.0,0.0),0.0);
     
     //printf("Second\n");
     //this->connect_to_Nav(0,&flatNav_p);
     
     //printf("Third\n");
     //this->connect_to_Guid(1,&flatGuid_p);
     
     printf("Fourth\n");
     this->invoke_to_Nav(0,flatNav_t);
     this->invoke_to_Guid(0,flatGuid_t);

     


  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_SerWritePort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer
    )
  {
    this->pushFromPortEntry_SerWritePort(serBuffer);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // Guid
    this->connect_to_Guid(
        0,
        this->component.get_Guid_InputPort(0)
    );

    // Nav
    this->connect_to_Nav(
        0,
        this->component.get_Nav_InputPort(0)
    );

    // sched
    this->connect_to_sched(
        0,
        this->component.get_sched_InputPort(0)
    );

    // SerReadPort
    this->connect_to_SerReadPort(
        0,
        this->component.get_SerReadPort_InputPort(0)
    );

    // SerWritePort
    this->component.set_SerWritePort_OutputPort(
        0, 
        this->get_from_SerWritePort(0)
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

} // end namespace Drv
