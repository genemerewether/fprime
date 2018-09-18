include $(BUILD_ROOT)/mk/configs/compiler/ccs7.0_r5_common.mk

CFLAGS := 	$(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CCS_CFLAGS_COMMON) \
			$(CCS_R5_COMMON_FLAGS) \
			$(CCS_R5_COMMON_INCLUDES)

CXXFLAGS := $(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CCS_CXXFLAGS_COMMON) \
			$(CCS_R5_COMMON_FLAGS) \
			$(CCS_R5_COMMON_INCLUDES)
				

COMPILER := nortos-tms570lc43x-debug-opt-ccs7.0

