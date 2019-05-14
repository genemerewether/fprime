#include <ROS/RosTime/RosTimeImpl.hpp>
#include <Fw/Time/Time.hpp>
#include <time.h>

#include <ros/ros.h>

namespace ROS {

    #if FW_OBJECT_NAMES == 1
    RosTimeImpl::RosTimeImpl(const char* name) : TimeComponentBase(name)
    #else
    RosTimeImpl::RosTimeImpl()
    #endif
    {
    }

    RosTimeImpl::~RosTimeImpl() {
    }

    void RosTimeImpl::timeGetPort_handler(
            NATIVE_INT_TYPE portNum, /*!< The port number*/
            Fw::Time &time /*!< The U32 cmd argument*/
        ) {

        ros::Time now = ros::Time::now();

	time.set(TB_ROS_TIME, 0,
		 now.sec,
		 now.nsec / 1000);
    }

    void RosTimeImpl::init(NATIVE_INT_TYPE instance) {
        TimeComponentBase::init(instance);
    }

}
