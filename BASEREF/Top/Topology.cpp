#include <ros/ros.h>

//#include <Components.hpp>

#include <Fw/Types/Assert.hpp>
#include <Os/Task.hpp>
#include <Os/Log.hpp>
#include <Fw/Types/MallocAllocator.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

void manualConstruct() {

}

void constructApp(int port_number, char* hostname) {

}

void exitTasks(void) {

}

void print_usage() {
    (void) printf("Usage: ./BASEREF [options]\n-p\tport_number\n-a\thostname/IP address\n");
}


#include <signal.h>
#include <stdio.h>

extern "C" {
    int main(int argc, char* argv[]);
};

volatile sig_atomic_t terminate = 0;

static void sighandler(int signum) {
    terminate = 1;
    ros::shutdown();
}

#include <ROS/Gen/mav_msgs/Ports/AttitudeRateThrustPortAc.hpp>
#include <mav_msgs/AttitudeRateThrust.h>
void attCallback(const mav_msgs::AttitudeRateThrust::ConstPtr& msg) {
    DEBUG_PRINT("Attitude rate thrust output handler\n");

    {
        using namespace ROS::std_msgs;
        using namespace ROS::mav_msgs;
        using namespace ROS::geometry_msgs;
        AttitudeRateThrust attRateThrust(
          Header(msg->header.seq,
                 Fw::Time(TB_ROS_TIME, 0,
                          msg->header.stamp.sec,
                          msg->header.stamp.nsec / 1000),
                 // TODO(mereweth) - convert frame id
                 0/*Fw::EightyCharString(msg->header.frame_id.data())*/),
          
          Quaternion(msg->attitude.x, msg->attitude.y, msg->attitude.z, msg->attitude.w),
          Vector3(msg->angular_rates.x, msg->angular_rates.y, msg->angular_rates.z),
          Vector3(msg->thrust.x, msg->thrust.y, msg->thrust.z)
          
        ); // end AttitudeRateThrust constructor
    }

    
}

int main(int argc, char* argv[]) {
    U32 port_number = 0;
    I32 option = 0;
    char *hostname = NULL;

    // Removes ROS cmdline args as a side-effect
    ros::init(argc,argv,"BASEREF", ros::init_options::NoSigintHandler);

    while ((option = getopt(argc, argv, "hp:a:")) != -1){
        switch(option) {
            case 'h':
                print_usage();
                return 0;
                break;
            case 'p':
                port_number = atoi(optarg);
                break;
            case 'a':
                hostname = optarg;
                break;
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);
    signal(SIGKILL,sighandler);

    (void) printf("Hit Ctrl-C to quit\n");
    
    constructApp(port_number, hostname);

    ros::start();

    ros::NodeHandle n;
    ros::Subscriber updateSub = n.subscribe("att_setpoint", 10,
                                            attCallback);
        
    ros::spin();

    // stop tasks
    DEBUG_PRINT("Stopping tasks\n");
    exitTasks();

    // Give time for threads to exit
    (void) printf("Waiting for threads...\n");
    Os::Task::delay(1000);

    (void) printf("Exiting...\n");

    return 0;
}
