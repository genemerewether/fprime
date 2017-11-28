include $(BUILD_ROOT)/mk/configs/compiler/linaro_gnu_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk

CFLAGS := 	$(DEBUG) \
			$(LINUX_GNU_CFLAGS) \
			$(LINUX_GNU_INCLUDES) \
			$(UT_FLAGS)
			
			

CXXFLAGS := $(DEBUG) \
			$(LINUX_GNU_CXXFLAGS) \
			$(LINUX_GNU_INCLUDES) \
			$(UT_FLAGS)

COMPILER := linaro-cross-arm-ut-nocov-gnu
