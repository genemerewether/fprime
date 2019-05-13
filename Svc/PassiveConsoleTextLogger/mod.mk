#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged.
#
#


SRC = 	ConsoleTextLoggerImplCommon.cpp

SRC_CYGWIN = Stub/PrintfLoggerImplStub.cpp

SRC_LINUX = Stub/PrintfLoggerImplStub.cpp

SRC_SDFLIGHT = Stub/PrintfLoggerImplStub.cpp

SRC_DSPAL = Dspal/DspalLoggerImpl.cpp

SRC_TIMSP430 = MSP/TextLoggerImplMsp.cpp

SRC_DARWIN = Stub/PrintfLoggerImplStub.cpp

SRC_RASPIAN = Stub/PrintfLoggerImplStub.cpp

HDR = ConsoleTextLoggerImpl.hpp
