include $(BUILD_ROOT)/mk/configs/compiler/cubemx_stm32f4_common.mk
# We dont want to include UT flags for the STM32F4 b/c it will use the std library
#include $(BUILD_ROOT)/mk/configs/compiler/ut_flags.mk

CFLAGS := 	$(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CUBEMX_CFLAGS_COMMON) \
			$(CUBEMX_STM32F4_COMMON_FLAGS) \
			$(CUBEMX_STM32F4_COMMON_INCLUDES)
			#$(UT_FLAGS)
			

CXXFLAGS := $(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CUBEMX_CXXFLAGS_COMMON) \
			$(CUBEMX_STM32F4_COMMON_FLAGS) \
			$(CUBEMX_STM32F4_COMMON_INCLUDES)
			#$(UT_FLAGS)
				

COMPILER := nortos-stm32f405rg-cubemx-debug-opt

