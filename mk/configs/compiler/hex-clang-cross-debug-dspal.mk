include $(BUILD_ROOT)/mk/configs/compiler/dspal_hexagon_clang_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk

CFLAGS := 	$(DEBUG) \
			$(DSPAL_HEX_CLANG_CFLAGS) \
			$(DSPAL_HEX_CLANG_INCLUDES)
			

CXXFLAGS := $(DEBUG) \
			$(DSPAL_HEX_CLANG_CXXFLAGS) \
			$(DSPAL_HEX_CLANG_INCLUDES)

COMPILER := hex-clang-cross-debug-dspal
