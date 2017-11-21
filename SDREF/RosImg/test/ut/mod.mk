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

TEST_SRC_SDFLIGHT = TesterBase.cpp GTestBase.cpp Tester.cpp main.cpp

TEST_MODS_SDFLIGHT = SDREF/RosImg gtest Fw/Cmd Fw/Comp Fw/Port Fw/Time Fw/Tlm \
                     Fw/Types Fw/Log Fw/Obj Os Fw/Com Fw/Buffer Svc/Sched


COMPARGS_SDFLIGHT = -I $(HEXAGON_ARM_SYSROOT)/usr/include/arm-linux-gnueabihf \
                    -I $(HEXAGON_ARM_SYSROOT)/usr/include \
                    -I $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/include

COMPARGS_LINUX = -I /opt/ros/indigo/include

TEST_LIBS_SDFLIGHT = $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libimage_transport.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libmessage_filters.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libclass_loader.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/libPocoFoundation.so $(HEXAGON_ARM_SYSROOT)/usr/lib/arm-linux-gnueabihf/libdl.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libroscpp.so $(HEXAGON_ARM_SYSROOT)/usr/lib/arm-linux-gnueabihf/libpthread.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libboost_signals.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/librosconsole.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/librosconsole_log4cxx.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/librosconsole_backend_interface.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/liblog4cxx.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libboost_regex.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libxmlrpcpp.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libroslib.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/librospack.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libpython2.7.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libboost_filesystem.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libboost_program_options.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libtinyxml.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libroscpp_serialization.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/librostime.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libboost_date_time.so $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/lib/libcpp_common.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libboost_system.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libboost_thread.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libpthread.so /opt/tools/leo//Qualcomm/qrlinux_sysroot//usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
