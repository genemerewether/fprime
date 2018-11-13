TEST_SRC = GTestBase.cpp TesterBase.cpp Tester.cpp main.cpp

TEST_MODS = HLProc/HLRosIface \
			\
	ROS/Gen/std_msgs/Ports  \
	ROS/Gen/geometry_msgs/Ports      \
	ROS/Gen/nav_msgs/Ports           \
	ROS/Gen/std_srvs/Ports           \
	ROS/Gen/rosgraph_msgs/Ports	 \
	ROS/Gen/actionlib_msgs/Ports     \
	ROS/Gen/mav_msgs/Ports		 \
	ROS/Gen/sensor_msgs/Ports	 \
			\
	ROS/Gen/std_msgs/Types  \
	ROS/Gen/geometry_msgs/Types      \
	ROS/Gen/nav_msgs/Types           \
	ROS/Gen/std_srvs/Types           \
	ROS/Gen/rosgraph_msgs/Types	 \
	ROS/Gen/actionlib_msgs/Types     \
	ROS/Gen/mav_msgs/Types		 \
	ROS/Gen/sensor_msgs/Types	 \
			\
			Svc/Sched \
			Svc/Ping \
			Fw/Cmd \
			Fw/Tlm \
			Fw/Comp \
			Fw/Log \
			Fw/Obj \
			Fw/Port \
			Fw/Time \
			Fw/Buffer \
			Fw/Types \
			Utils/Hash \
			Os \
			gtest
