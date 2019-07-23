#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.

SRC = BlspGpioDriverComponentAi.xml BlspGpioDriverComponentImplCommon.cpp

SRC_SDFLIGHT = BlspGpioDriverComponentImplSdFlight.cpp

SRC_DARWIN = BlspGpioDriverComponentImplStub.cpp 

SRC_LINUX = BlspGpioDriverComponentImplStub.cpp 

SRC_CYGWIN = BlspGpioDriverComponentImplStub.cpp 

HDR = BlspGpioDriverComponentImpl.hpp

SUBDIRS = test

SRC_LINUXRT = BlspGpioDriverComponentImplStub.cpp
