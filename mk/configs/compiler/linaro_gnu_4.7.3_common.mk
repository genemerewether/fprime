include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/linux_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/gnu4.7.3-common.mk

CC :=  /usr/local/DS-5/bin/arm-linux-gnueabihf-gcc
CXX := /usr/local/DS-5/bin/arm-linux-gnueabihf-g++
GCOV := /usr/local/DS-5/bin/arm-linux-gnueabihf-gcov

CC_MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh

LINK_LIB := $(CXX)
LINK_LIB_FLAGS := -r -nodefaultlibs -nostdlib 

LINK_BIN_PRE := $(CXX)
LINK_BIN_PRE_FLAGS := -z muldefs $(LIBS)

POST_LINK_BIN := echo

FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)

POST_LINK_LIB := $(BUILD_ROOT)/mk/bin/empty.sh
SYMBOL_CHECK := $(BUILD_ROOT)/mk/bin/empty.sh
DEMANGLE := $(BUILD_ROOT)/mk/bin/empty.sh
POST_LINK_LIB_ARGS := 

LINK_LIBS := -ldl -lpthread -lm -lrt

OPT_SPEED := -Os
DEBUG := -g3

LINUX_GNU_CFLAGS := $(LINUX_FLAGS_COMMON) \
					$(COMMON_DEFINES) \
					$(GNU_CFLAGS_COMMON) 

LINUX_GNU_CXXFLAGS :=	$(LINUX_FLAGS_COMMON) \
						$(COMMON_DEFINES) \
						$(GNU_CXXFLAGS_COMMON)

COVERAGE := -fprofile-arcs -ftest-coverage

LINUX_GNU_INCLUDES := $(LINUX_INCLUDES_COMMON) $(COMMON_INCLUDES)


DUMP = $(BUILD_ROOT)/mk/bin/empty.sh

MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh
