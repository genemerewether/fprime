include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/linux_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/gnu4.7.2-common.mk

CC :=  /usr/bin/gcc
CXX := /usr/bin/g++
GCOV := /usr/bin/gcov

BUILD_32BIT := -m32

CC_MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh

LINK_LIB := $(CXX)
LINK_LIB_FLAGS := -r -nodefaultlibs -nostdlib #$(BUILD_32BIT)

LINK_BIN := $(CXX)
LINK_BIN_FLAGS := -z muldefs $(LIBS) #$(BUILD_32BIT)

FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)

POST_LINK_LIB := $(BUILD_ROOT)/mk/bin/empty.sh

LINK_LIBS := -ldl -lpthread -lm -lrt -lssl -lcrypto

OPT_SPEED := -Os
DEBUG := -g3

LINUX_GNU_CFLAGS := $(LINUX_FLAGS_COMMON) \
					$(COMMON_DEFINES) \
					$(GNU_CFLAGS_COMMON) \
					#$(BUILD_32BIT) # Quantum framework won't build 32-bit

LINUX_GNU_CXXFLAGS :=	$(LINUX_FLAGS_COMMON) \
						$(COMMON_DEFINES) \
						$(GNU_CXXFLAGS_COMMON) \
						#$(BUILD_32BIT)

COVERAGE := -fprofile-arcs -ftest-coverage

LINUX_GNU_INCLUDES := $(LINUX_INCLUDES_COMMON) $(COMMON_INCLUDES)


DUMP = $(BUILD_ROOT)/mk/bin/empty.sh

MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh
