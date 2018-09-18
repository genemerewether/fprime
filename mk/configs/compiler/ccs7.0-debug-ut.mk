include $(BUILD_ROOT)/mk/configs/compiler/ccs7.0_r5_common.mk
# We dont want to include UT flags for the R5 b/c it will use the std library
#include $(BUILD_ROOT)/mk/configs/compiler/ut_flags.mk

CFLAGS := 	$(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CCS_CFLAGS_COMMON) \
			$(CCS_R5_COMMON_FLAGS) \
			$(CCS_R5_COMMON_INCLUDES)
			#$(UT_FLAGS)
			

CXXFLAGS := $(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CCS_CXXFLAGS_COMMON) \
			$(CCS_R5_COMMON_FLAGS) \
			$(CCS_R5_COMMON_INCLUDES)
			#$(UT_FLAGS)
				

COMPILER := nortos-tms570lc43x-ut-ccs7.0

