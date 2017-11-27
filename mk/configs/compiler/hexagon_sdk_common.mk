HEXAGON_BIN := $(HEXAGON_TOOLS_ROOT)/bin
HEXAGON_ISS_DIR := $(HEXAGON_TOOLS_ROOT)/lib/iss

HEXAGON_SDK_INCLUDES := -I $(HEXAGON_SDK_ROOT)/incs \
			-I $(HEXAGON_SDK_ROOT)/incs/stddef \
			-I $(HEXAGON_SDK_ROOT)/libs/common/rpcmem/inc \
			-I $(HEXAGON_SDK_ROOT)/libs/common/rpcmem \
			-I $(HEXAGON_SDK_ROOT)/libs/common/remote/ship/hexagon_Debug
