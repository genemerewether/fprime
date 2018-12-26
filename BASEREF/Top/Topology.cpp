#include <ros/ros.h>

//#include <Components.hpp>
#include <Svc/UdpSender/UdpSenderComponentImpl.hpp>

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

Svc::UdpSenderComponentImpl udpSender
#if FW_OBJECT_NAMES == 1
                    ("UDPSND")
#endif
;

void manualConstruct() {

}

#include <ROS/Gen/mav_msgs/Ports/AttitudeRateThrustPortAc.hpp>
void constructApp(int port_number, char* hostname, char* udp_server_addr, char* udp_port_number) {
    NATIVE_INT_TYPE udpSender_buff_size = ROS::mav_msgs::InputAttitudeRateThrustPort::SERIALIZED_SIZE + 25;
    udpSender.init(10, udpSender_buff_size);
    
    if (udp_server_addr && udp_port_number) {
        udpSender.open(udp_server_addr,udp_port_number);
    }
    udpSender.start(0,80,20*1024);
}

void exitTasks(void) {
    udpSender.exit();
}

void print_usage() {
    (void) printf("Usage: ./BASEREF [options]\n-p\tport_number\n-a\thostname/IP address\n-u\tUDP port\n-s\tUDP address\n");
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

#include <mav_msgs/AttitudeRateThrust.h>
#include <Fw/Types/SerialBuffer.hpp>
#include <Fw/Port/InputSerializePort.hpp>
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

        Fw::InputSerializePort* port = udpSender.get_PortsIn_InputPort(0);
        Fw::SerializeStatus status;
        U8 m_buff[ROS::mav_msgs::InputAttitudeRateThrustPort::SERIALIZED_SIZE];
        Fw::SerialBuffer _buffer(m_buff, FW_NUM_ARRAY_ELEMENTS(m_buff));
        status = _buffer.serialize(attRateThrust);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));

        status = port->invokeSerial(_buffer);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
    }
}

int main(int argc, char* argv[]) {
    U32 port_number = 0;
    I32 option = 0;
    char *hostname = NULL;
    char* udp_port_number = 0;
    char* udp_server_addr = 0;

    // Removes ROS cmdline args as a side-effect
    ros::init(argc,argv,"BASEREF", ros::init_options::NoSigintHandler);

    while ((option = getopt(argc, argv, "hp:a:u:s:")) != -1){
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
            case 'u':
                udp_port_number = optarg;
                break;
            case 's':
                udp_server_addr = optarg;
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
    
    constructApp(port_number, hostname, udp_server_addr, udp_port_number);

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
