GCC_BASE := /dsw/gcc-4.1.1_32

CC :=  $(GCC_BASE)/bin/gcc
CC_MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh
CXX := $(GCC_BASE)/bin/g++
LINK_LIB := /usr/bin/ar
LINK_BIN_PRE := $(CC)
LINK_BIN_POST := $(BUILD_ROOT)/mk/bin/renameforlinux.sh
FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)

SYMBOL_CHECK := $(BUILD_ROOT)/mk/bin/empty.sh


POST_LINK_LIB := 
POST_LINK_LIB_ARGS := 

LIBS := 

OPT := -g

INCLUDES := -I$(BUILD_ROOT)
				
CFLAGS := $(OPT) $(INCLUDES) -DBUILD_$(BUILD) -DTGT_LINUX_SSIM -DCPU=SIMLINUX \
			-std=c99 -Wall -Werror-implicit-function-declaration 			
				
CXXFLAGS := $(OPT) $(INCLUDES) -DBUILD_$(BUILD) -DTGT_LINUX_SSIM -DCPU=SIMLINUX -Wall -fcheck-new

DUMP := $(BUILD_ROOT)/mk/bin/empty.sh

MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh
				
LINK_LIB_FLAGS := -rcv

LINK_BIN_PRE_FLAGS := $(LIBS) 
LINK_BIN_POST_FLAGS :=  

COMPILER := linux-x86-debug-gnu-4.1.1
