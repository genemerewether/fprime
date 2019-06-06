#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged.
#
#

# This is a template for the mod.mk file that goes in each module
# and each module's subdirectories.
# With a fresh checkout, "make gen_make" should be invoked. It should also be
# run if any of the variables are updated. Any unused variables can 
# be deleted from the file.

# There are some standard files that are included for reference

SRC = \	
	PolyType.cpp \
	StringType.cpp \
	Serializable.cpp \
	SerialBuffer.cpp \
	Assert.cpp \
	EightyCharString.cpp \
	InternalInterfaceString.cpp \
	BasicTypes.cpp \
	MemAllocator.cpp \
	MallocAllocator.cpp

SRC_LINUX = \
	MmapAllocator.cpp

SRC_LINUXRT = \
	MmapAllocator.cpp

SRC_SDFLIGHT = \
	MmapAllocator.cpp

SRC_DARWIN = \
	MmapAllocator.cpp
#	FwStructSerializable.cpp

HDR = \
	BasicTypes.hpp \
	PolyType.hpp \
	StringType.hpp \
	Serializable.hpp \
	SerialBuffer.hpp \
	Assert.hpp \
	EightyCharString.hpp \
	InternalInterfaceString.hpp \
	CAssert.hpp \
	MemAllocator.hpp \
	MallocAllocator.hpp \
	MmapAllocator.hpp

#	FwStructSerializable.hpp

SRC_BAERAD750 = \
	VxWorks/VxWorksLogAssert.cpp

SUBDIRS = test

