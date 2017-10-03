include $(BUILD_ROOT)/mk/configs/compiler/pru_cgt_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/ut_flags.mk

CFLAGS := 	$(DEBUG) \
			$(PRU_CGT_CFLAGS) \
			$(PRU_CGT_INCLUDES) \
			$(COVERAGE) \
			$(UT_FLAGS)


CXXFLAGS := $(DEBUG) \
			$(PRU_CGT_CXXFLAGS) \
			$(PRU_CGT_INCLUDES) \
			$(COVERAGE) \
			$(UT_FLAGS)


LINK_BIN_FLAGS := $(LINK_BIN_FLAGS) $(COVERAGE) 

COMPILER := cgt2.1.5-ut
