SRC = R5SpiSlaveDriverComponentAi.xml 

SRC_TIR5 = SpiSlaveDrv.cpp R5SpiSlaveDriverComponentImplR5.cpp

SRC_CYGWIN = R5SpiSlaveDriverComponentImplStub.cpp

SRC_DARWIN = R5SpiSlaveDriverComponentImplStub.cpp

SRC_LINUX = R5SpiSlaveDriverComponentImplStub.cpp

HDR = SpiSlaveDrv.hpp R5SpiSlaveDriverComponentImpl.hpp

SUBDIRS = test
