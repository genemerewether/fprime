// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Tester.hpp"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc,argv,"HLRosIface_ut");

  ros::start();

  //component.startPub();
  //component.startIntTask(30, 20*1024);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
