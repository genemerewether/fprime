#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.
#

SRC = BlspSpiDriverComponentAi.xml BlspSpiDriverComponentImplCommon.cpp

SRC_SDFLIGHT = BlspSpiDriverComponentImplSdFlight.cpp

SRC_DARWIN =  BlspSpiDriverComponentImplStub.cpp

SRC_LINUX =  BlspSpiDriverComponentImplStub.cpp

SRC_CYGWIN = BlspSpiDriverComponentImplStub.cpp

HDR = BlspSpiDriverComponentImpl.hpp

SUBDIRS = test

SRC_LINUXRT = BlspSpiDriverComponentImplStub.cpp
