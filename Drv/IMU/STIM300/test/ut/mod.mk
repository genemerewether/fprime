#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.
#
#   Information included herein is controlled under the International
#   Traffic in Arms Regulations ("ITAR") by the U.S. Department of State.
#   Export or transfer of this information to a Foreign Person or foreign
#   entity requires an export license issued by the U.S. State Department
#   or an ITAR exemption prior to the export or transfer.
#

TEST_SRC = 	TesterBase.cpp \
			GTestBase.cpp \
			Tester.cpp \
			STIM300.cpp \
			STIM300Model.cpp

TEST_MODS = Drv/IMU/STIM300 \
			Svc/Sched \
			Fw/Log \
			Fw/Tlm \
			Fw/Port \
			Fw/Comp \
			Fw/Obj \
			Fw/Time \
			Fw/Types \
			Fw/Buffer \
			Drv/SerialDriverPorts \
			ROS/Gen/sensor_msgs/Ports \
			ROS/Gen/sensor_msgs/Types \
			ROS/Gen/geometry_msgs/Ports \
			ROS/Gen/geometry_msgs/Types \
			ROS/Gen/std_msgs/Ports \
			ROS/Gen/std_msgs/Types \
			Os \
			gtest
