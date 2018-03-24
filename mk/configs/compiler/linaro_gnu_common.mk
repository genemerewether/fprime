include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/linux_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/gnu-common.mk
include $(BUILD_ROOT)/mk/configs/compiler/ut_flags.mk

CC :=  $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin/arm-linux-gnueabihf-gcc
CXX := $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin/arm-linux-gnueabihf-g++
GCOV := $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin/arm-linux-gnueabihf-gcov
AR := $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2014.11-x86_64_arm-linux-gnueabihf_linux/bin/arm-linux-gnueabihf-ar
#CC :=  $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc
#CXX := $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++
#GCOV := $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcov
#AR := $(HEXAGON_SDK_ROOT)/gcc-linaro-4.9-2016.02-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-ar

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
LINK_LIBS += 	-L$(HEXAGON_SDK_ROOT)/libs/common/remote/ship/UbuntuARM_Debug \
				-ladsprpc \
				$(HEXAGON_SDK_ROOT)/libs/common/rpcmem/UbuntuARM_Debug/rpcmem.a
				# -L $(HEXAGON_ARM_SYSROOT)/usr/lib/ \
				# -lcamera \
				# -lcamparams \
				# -lz \
				# -lCB \
				# -lssl \
				# -lOpenCL \
				# -lglib-2.0 \
				# -lEGL_adreno \
				# -lGLESv2_adreno \
				# -ladreno_utils \
				# -lgsl \
				# -lsc-a3xx \
				# $(HEXAGON_ARM_SYSROOT)/lib/libcrypto.so.1.0.0 \
				# $(HEXAGON_ARM_SYSROOT)/lib/libstdc++.so.6
				#-lmv1 # Machine Vision SDK, depends on libraries above
				#$(HEXAGON_ARM_SYSROOT)/usr/lib/arm-linux-gnueabihf/libstdc++.so.6
				# not necessary if libraries above are specified explicitly
				#-Wl,-rpath-link=$(HEXAGON_ARM_SYSROOT)/usr/lib \
				#-Wl,-rpath-link=$(HEXAGON_ARM_SYSROOT)/lib

OPT_SPEED := -O3 -funroll-loops
DEBUG := -g3

LINUX_GNU_CFLAGS := $(LINUX_FLAGS_COMMON) \
					$(COMMON_DEFINES) \
					$(GNU_CFLAGS_COMMON)

					#$(BUILD_32BIT) # Quantum framework won't build 32-bit

LINUX_GNU_CXXFLAGS :=	$(LINUX_FLAGS_COMMON) \
						$(COMMON_DEFINES) \
						$(GNU_CXXFLAGS_COMMON)
						#$(BUILD_32BIT)

COVERAGE := -fprofile-arcs -ftest-coverage

LINUX_GNU_INCLUDES := 	$(LINUX_INCLUDES_COMMON) \
						$(COMMON_INCLUDES) \
						-I$(HEXAGON_SDK_ROOT)/incs \
						-I$(HEXAGON_SDK_ROOT)/incs/stddef \
						-I$(HEXAGON_SDK_ROOT)/libs/common/rpcmem/inc \
						-I$(HEXAGON_SDK_ROOT)/libs/common/qurt/ADSPv5MP/include \
						-I$(HEXAGON_SDK_ROOT)/libs/common/rpcmem \
						-I$(HEXAGON_SDK_ROOT)/libs/common/adspmsgd/ship/UbuntuARM_Debug \
						-I$(HEXAGON_SDK_ROOT)/libs/common/remote/ship/UbuntuARM_Debug \
						-I$(HEXAGON_SDK_ROOT)/incs/stddef

DUMP = $(BUILD_ROOT)/mk/bin/empty.sh

MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh
