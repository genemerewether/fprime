include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/dspal_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/hexagon_clang_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/ut_flags.mk
include $(BUILD_ROOT)/mk/configs/compiler/hexagon_sdk_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/hexagon_v55.mk

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
		    $(HEX_CLANG_CFLAGS_COMMON) \
		    $(HEXAGON_ARCH_CPU_FLAGS) \
		    $(HEXAGON_ARCH_CFLAGS)

#$(BUILD_32BIT) # Quantum framework won't build 32-bit

DSPAL_HEX_CLANG_CXXFLAGS :=	$(DSPAL_FLAGS_COMMON) \
				$(COMMON_DEFINES) \
				$(HEX_CLANG_CXXFLAGS_COMMON) \
				$(HEXAGON_ARCH_CPU_FLAGS) \
				$(HEXAGON_ARCH_CXXFLAGS)
#$(BUILD_32BIT)

COVERAGE := -fprofile-arcs -ftest-coverage

# TODO(mereweth)

DSPAL_HEX_CLANG_INCLUDES := 	$(DSPAL_INCLUDES_COMMON) \
				$(COMMON_INCLUDES) \
				$(HEXAGON_SDK_INCLUDES) \
				$(HEXAGON_ARCH_INCLUDES)

#-I$(HEXAGON_TOOLS_ROOT)

LINK_BIN := $(HEXAGON_BIN)/hexagon-link
LINK_BIN_FLAGS := $(HEXAGON_ARCH_LINK_FLAGS) \
		  -shared \
		  -call_shared \
		  -G0 \
		  $(HEXAGON_ARCH_LIB_DIR)/initS.o

LINK_LIBS := 

LIBS_START := -L$(HEXAGON_ARCH_LIB_DIR) \
	-Bsymbolic \
	$(HEXAGON_ARCH_LIB_DIR)/libgcc.a \
	--wrap=malloc \
	--wrap=calloc \
	--wrap=free \
	--wrap=realloc \
	--wrap=memalign \
	--wrap=__stack_chk_fail \
	-lc \
	\
	--start-group --whole-archive

LIBS_END := --no-whole-archive \
	$(HEXAGON_ARCH_LIB_DIR)/libstdc++.a \
	--end-group \
	\
	--start-group \
	-lgcc \
	--end-group \
	$(HEXAGON_ARCH_LIB_DIR)/finiS.o

