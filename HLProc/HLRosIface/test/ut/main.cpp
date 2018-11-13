// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Tester.hpp"
#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "mav_msgs/Actuators.h"
#include "mav_msgs/ImuStateUpdate.h"

TEST(Nominal, NominalInit) {
    HLProc::Tester tester;

    for (int i = 0; i < 10; i++) {
        ros::spinOnce();
        Os::Task::delay(100);
    }
}

TEST(Nominal, NominalLoopback) {
    HLProc::Tester tester;

    ros::NodeHandle n;
    ros::Publisher actPub = n.advertise<mav_msgs::Actuators>("dsp_pwm_command", 1);

    mav_msgs::Actuators msg;
    msg.angles.clear();
    msg.angular_velocities.clear();
    msg.normalized.clear();

    for (int i = 0; i < 10; i++) {
        actPub.publish(msg);
        ros::spinOnce();
        tester.callSched();
        Os::Task::delay(100);

        msg.angles.push_back(i);
        msg.angular_velocities.push_back(i);
        msg.normalized.push_back(i);
    }
}

int main(int argc, char **argv) {
  ros::init(argc,argv,"HLRosIface_ut");

  ros::start();

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
