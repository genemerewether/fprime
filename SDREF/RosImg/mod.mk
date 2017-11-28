#
#   Copyright 2004-2016, by the California Institute of Technology.
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

SRC = RosImgComponentAi.xml RosImgComponentImpl.cpp

#SRC_SDFLIGHT = RosImgComponentImplSdFlight.cpp

#SRC_DARWIN = RosImgComponentImplStub.cpp

#SRC_LINUX = RosImgComponentImplStub.cpp

#SRC_CYGWIN = RosImgComponentImplStub.cpp

HDR = RosImgComponentImpl.hpp

SUBDIRS = test

COMPARGS_SDFLIGHT = -I $(HEXAGON_ARM_SYSROOT)/usr/include/arm-linux-gnueabihf \
                    -I $(HEXAGON_ARM_SYSROOT)/usr/include \
                    -I $(HEXAGON_ARM_SYSROOT)/opt/ros/indigo/include

COMPARGS_LINUX = -I /opt/ros/indigo/include
