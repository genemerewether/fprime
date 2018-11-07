SRC =  R5SpiMasterDriverComponentAi.xml 

SRC_TIR5 = SpiMasterDrv.cpp R5SpiMasterDriverComponentImplR5.cpp

SRC_CYGWIN = R5SpiMasterDriverComponentImplStub.cpp

SRC_DARWIN = R5SpiMasterDriverComponentImplStub.cpp

SRC_LINUX = R5SpiMasterDriverComponentImplStub.cpp

HDR = SpiMasterDrv.hpp R5SpiMasterDriverComponentImpl.hpp

SUBDIRS = test
