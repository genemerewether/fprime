include $(BUILD_ROOT)/mk/configs/compiler/linux_gnu_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk

CFLAGS := 	$(OPT_SPEED) \
			$(DEBUG) \
			$(LINUX_GNU_CFLAGS) \
			$(LINUX_GNU_INCLUDES)


CXXFLAGS := $(OPT_SPEED) \
			$(DEBUG) \
			$(LINUX_GNU_CXXFLAGS) \
			$(LINUX_GNU_INCLUDES)

COMPILER := linux-x86-opt-gnu
