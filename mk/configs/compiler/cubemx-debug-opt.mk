include $(BUILD_ROOT)/mk/configs/compiler/cubemx_stm32f4_common.mk

CFLAGS := 	$(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CUBEMX_CFLAGS_COMMON) \
			$(CUBEMX_STM32F4_COMMON_FLAGS) \
			$(CUBEMX_STM32F4_COMMON_INCLUDES)

CXXFLAGS := $(DEBUG) \
			$(OPT_SPEED) \
			\
			$(CUBEMX_CXXFLAGS_COMMON) \
			$(CUBEMX_STM32F4_COMMON_FLAGS) \
			$(CUBEMX_STM32F4_COMMON_INCLUDES)
				

COMPILER := nortos-stm32f405rg-cubemx-debug-opt

