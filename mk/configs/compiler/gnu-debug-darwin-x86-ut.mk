#GCC_BASE := /dsw/gcc-3.4.6s
GCC_BASE := /usr

CC :=  $(GCC_BASE)/bin/gcc
CXX := $(GCC_BASE)/bin/g++
GCOV := $(GCC_BASE)/bin/gcov
#LINK_LIB := $(CXX)
LINK_LIB := /usr/bin/libtool
LINK_BIN_PRE := $(CXX)
LINK_BIN_POST := $(BUILD_ROOT)/mk/bin/renameforlinux.sh
FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)

POST_LINK_LIB := $(BUILD_ROOT)/mk/bin/empty.sh
SYMBOL_CHECK := $(BUILD_ROOT)/mk/bin/empty.sh
#ranlib
POST_LINK_LIB_ARGS := 

LIBS := -lgcov

OPT := -g -MD

#INCLUDES := -I$(BUILD_ROOT) -I/usr/include -I/usr/include/sys/
INCLUDES := -I$(BUILD_ROOT)

CFLAGS := $(OPT) $(INCLUDES) -DBUILD_$(BUILD) -DBUILD_LINUX -DTGT_LINUX_SSIM \
	-DPPC604=1 -DSIMLINUX=2 -DSTATIC= -DBUILD_UT -DEVRS_TO_CONSOLE \
	-DCPU=SIMLINUX -std=c99 -Wall -Werror-implicit-function-declaration \
    -fprofile-arcs -ftest-coverage

CXXFLAGS := $(OPT) $(INCLUDES) -DBUILD_$(BUILD) -DBUILD_LINUX -DTGT_LINUX_SSIM \
	-DPPC604=1 -DSIMLINUX=2 -DSTATIC= -DBUILD_UT -DEVRS_TO_CONSOLE \
	-DCPU=SIMLINUX -Wall -fcheck-new \
    -fprofile-arcs -ftest-coverage
    
AC_CC_FLAGS :=	-Wno-extra
AC_CXX_FLAGS :=
AC_HSM_FLAGS := -Wno-extra -Wno-parentheses
AC_PARAMS_FLAGS := -fno-strict-aliasing	-Wno-extra	
    

DUMP = $(BUILD_ROOT)/mk/bin/empty.sh

MUNCH := $(BUILD_ROOT)/mk/bin/empty.sh

#LINK_LIB_FLAGS := -shared --export-dynamic
#LINK_LIB_FLAGS := -rcv
LINK_LIB_FLAGS := -static 

LINK_BIN_PRE_FLAGS := $(LIBS) -all_load
LINK_BIN_POST_FLAGS := 

COMPILER := darwin-x86-debug-gnu-ut
