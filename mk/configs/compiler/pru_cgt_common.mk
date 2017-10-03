include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/pru_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/cgt_common.mk

PRU_CGT_FLAGS_COMMON := $(COMMON_DEFINES) \
			$(CGT_FLAGS_COMMON) \
			-v3 \
			--endian=little \
			--hardware_mac=on \
			--abi=eabi

PRU_CGT_INCLUDES_COMMON := $(COMMON_INCLUDES) \
			   $(CGT_INCLUDES_COMMON) \
			   $(INCLUDE_PATH)$(PRU_SDK_DIR)/include

COVERAGE :=

CC := $(PRU_SDK_DIR)/bin/clpru
CXX := $(CC)
AR := $(PRU_SDK_DIR)/bin/arpru

LINK_LIB := $(AR)
LINK_LIB_FLAGS := -r
POST_LINK_LIB := $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.py

#TODO(genemerewether@gmail.com) - how to handle different linker command files?
LINK_LIBS := # TODO(genemerewether@gmail.com) - add rpmsg?

LINK_BIN := $(CC)
#TODO(genemerewether@gmail.com) - do we need to specify c/c++ standard to the linker?
#TODO(genemerewether@gmail.com) - should we always have debug option for linker?
LINK_BIN_FLAGS := $(CGT_FLAGS_COMMON) \
		  -v3 \
		  --abi=eabi \
		  -z -m"PRUBin.map" \
		  --xml_link_info="PRUBin_linkInfo.xml" \
		  --reread_libs \
		  --warn_sections \
		  --rom_model

LINK_BIN_POST_LIB_FLAGS := -I=$(PRU_SDK_DIR)lib/ \
			   --library=$(PRU_SDK_DIR)lib/libc.a

FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)




OPT_SPEED := -Os
DEBUG := -g

PRU_CGT_CFLAGS := $(PRU_FLAGS_COMMON) \
		  $(COMMON_DEFINES) \
		  $(CGT_CFLAGS_COMMON)

PRU_CGT_CXXFLAGS := $(PRU_FLAGS_COMMON) \
						$(COMMON_DEFINES) \
						$(CGT_CXXFLAGS_COMMON)

PRU_CGT_INCLUDES := $(PRU_INCLUDES_COMMON) $(COMMON_INCLUDES)

DUMP = $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.sh
SYMBOL_SIZES = $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.sh
MUNCH := $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.sh
