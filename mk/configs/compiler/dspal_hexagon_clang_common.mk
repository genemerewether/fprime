include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/dspal_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/hexagon_clang_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/ut_flags.mk
include $(BUILD_ROOT)/mk/configs/compiler/hexagon_sdk_common.mk

CC := $(HEXAGON_BIN)/hexagon-clang
CXX := $(HEXAGON_BIN)/hexagon-clang++
# GCOV :=
AR := $(HEXAGON_BIN)/hexagon-ar

BUILD_32BIT := -m32

CC_MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh

LINK_LIB := $(AR)
LINK_LIB_FLAGS := rcsD #qcD
LIBRARY_TO :=
POST_LINK_LIB := ranlib

FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)

OPT_SPEED := -O3 -funroll-loops
DEBUG := -g3

DUMP = $(BUILD_ROOT)/mk/bin/empty.sh

MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh

DSPAL_HEX_CLANG_CFLAGS := $(DSPAL_FLAGS_COMMON) \
		    $(COMMON_DEFINES) \
		    $(HEX_CLANG_CFLAGS_COMMON)
#$(BUILD_32BIT) # Quantum framework won't build 32-bit

DSPAL_HEX_CLANG_CXXFLAGS :=	$(DSPAL_FLAGS_COMMON) \
				$(COMMON_DEFINES) \
				$(HEX_CLANG_CXXFLAGS_COMMON)
#$(BUILD_32BIT)

COVERAGE := -fprofile-arcs -ftest-coverage

# TODO(mereweth)

DSPAL_HEX_CLANG_INCLUDES := 	$(DSPAL_INCLUDES_COMMON) \
				$(COMMON_INCLUDES) \
				-I $(HEXAGON_SDK_ROOT)/incs \
				-I $(HEXAGON_SDK_ROOT)/incs/stddef #\
#-I $(HEXAGON_SDK_ROOT)/libs/common/rpcmem/inc #\
#-I $(HEXAGON_SDK_ROOT)/libs/common/qurt/ADSPv55MP/include #\
#-I$(HEXAGON_TOOLS_ROOT)

LINK_BIN := $(HEXAGON_BIN)/hexagon-link
LINK_BIN_FLAGS := 

LINK_LIBS := 
