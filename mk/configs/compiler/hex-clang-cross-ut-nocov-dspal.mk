include $(BUILD_ROOT)/mk/configs/compiler/dspal_hexagon_clang_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk

CFLAGS := 	$(DEBUG) \
			$(DSPAL_HEX_CLANG_CFLAGS) \
			$(DSPAL_HEX_CLANG_INCLUDES) \
			$(UT_FLAGS)
			

CXXFLAGS := $(DEBUG) \
			$(DSPAL_HEX_CLANG_CXXFLAGS) \
			$(DSPAL_HEX_CLANG_INCLUDES) \
			$(UT_FLAGS)

COMPILER := hex-clang-cross-ut-nocov-dspal
