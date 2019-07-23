#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.
#

SRC = BlspSerialDriverComponentAi.xml BlspSerialDriverComponentImplCommon.cpp

SRC_SDFLIGHT = BlspSerialDriverComponentImplSdFlight.cpp

SRC_DARWIN = BlspSerialDriverComponentImplStub.cpp 

SRC_LINUX = BlspSerialDriverComponentImplStub.cpp 

SRC_CYGWIN = BlspSerialDriverComponentImplStub.cpp 

HDR = BlspSerialDriverComponentImpl.hpp

SUBDIRS = test

SRC_LINUXRT = BlspSerialDriverComponentImplStub.cpp
