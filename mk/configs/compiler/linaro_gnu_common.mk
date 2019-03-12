include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/linux_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/gnu-common.mk
include $(BUILD_ROOT)/mk/configs/compiler/ut_flags.mk

BUILD_32BIT := -m32

CC_MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh

LINK_LIB := $(AR)
LINK_LIB_FLAGS := rcs
LIBRARY_TO :=
POST_LINK_LIB := ranlib

LINK_BIN := $(CXX)
LINK_BIN_FLAGS := -rdynamic -z muldefs $(LIBS) #$(BUILD_32BIT)

FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)

LINK_LIBS := -ldl -lpthread -lm -lrt -lutil
LINK_LIBS += 	--sysroot=$(HEXAGON_ARM_SYSROOT) \
		-ladsprpc
ifeq ($(TARGET_8096),)
LINK_LIBS += -L$(HEXAGON_SDK_ROOT)/libs/common/remote/ship/UbuntuARM_Debug
endif

OPT_SPEED := -O3 -funroll-loops
DEBUG := -g3

FP_FLAGS := -DARM_NEON -DENABLE_NEON -mfpu=neon -march=armv7-a -mfloat-abi=softfp

LINUX_GNU_CFLAGS := $(LINUX_FLAGS_COMMON) \
			$(COMMON_DEFINES) \
			$(GNU_CFLAGS_COMMON) \
			$(FP_FLAGS)

LINUX_GNU_CXXFLAGS :=	$(LINUX_FLAGS_COMMON) \
			-std=c++11 \
			$(COMMON_DEFINES) \
			$(GNU_CXXFLAGS_COMMON) \
			$(FP_FLAGS)

COVERAGE := -fprofile-arcs -ftest-coverage

LINUX_GNU_INCLUDES := 	$(LINUX_INCLUDES_COMMON) \
			$(COMMON_INCLUDES) \
			--sysroot=$(HEXAGON_ARM_SYSROOT) \
			-I$(HEXAGON_SDK_ROOT)/incs \
			-I$(HEXAGON_SDK_ROOT)/incs/stddef \
			-I$(HEXAGON_SDK_ROOT)/libs/common/rpcmem/inc \
			-I$(HEXAGON_SDK_ROOT)/libs/common/qurt/ADSP$(HEXAGON_V_ARCH)MP/include \
			-I$(HEXAGON_SDK_ROOT)/libs/common/rpcmem \
			-I$(HEXAGON_SDK_ROOT)/libs/common/adspmsgd/ship/UbuntuARM_Debug \
			-I$(HEXAGON_SDK_ROOT)/libs/common/remote/ship/UbuntuARM_Debug \
			-I$(HEXAGON_SDK_ROOT)/incs/stddef

DUMP = $(BUILD_ROOT)/mk/bin/empty.sh

MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh
