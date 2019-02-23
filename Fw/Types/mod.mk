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
	MallocAllocator.cpp \
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

