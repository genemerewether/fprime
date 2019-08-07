#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.

TEST_SRC = 	TesterBase.cpp \
			GTestBase.cpp \
			Tester.cpp \
			LIDARLiteV3.cpp

TEST_MODS = Drv/Altimeter/LIDARLiteV3 \
			Svc/Sched \
			Fw/Log \
			Fw/Tlm \
			Fw/Port \
			Fw/Comp \
			Fw/Obj \
			Fw/Time \
			Fw/Types \
			Fw/Buffer \
			Drv/I2CDriverPorts \
			ROS/Gen/sensor_msgs/Ports \
			ROS/Gen/sensor_msgs/Types \
			ROS/Gen/std_msgs/Ports \
			ROS/Gen/std_msgs/Types \
			Os \
			gtest
