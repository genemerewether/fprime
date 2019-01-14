CUBEMX_COMMON := 

DEPEND_FILE := -MMD -MP -MF
               
CUBEMX_CFLAGS_COMMON := $(CUBEMX_COMMON) -std=c99 -fdata-sections -ffunction-sections

CUBEMX_CXXFLAGS_COMMON := $(CUBEMX_COMMON)


# Special compiler flags to get around known AC warnings

AC_CC_FLAGS := 
AC_CXX_FLAGS :=
AC_HSM_FLAGS := 
AC_PARAMS_FLAGS := 

COMPILE_ONLY := -c
SHARED_LIBRARY :=  -fPIC

COMPILE_TO := -o
LIBRARY_TO := -o
LINK_BIN_PRE_TO := -o
LINK_BIN_POST_TO := -o
LINK_BIN_TO := -o

POST_LINK_BIN := @echo

LIBS_START := -Wl,--start-group
LIBS_END := -Wl,--end-group

INCLUDE_PATH := -I

CUBEMX_INCLUDES_COMMON := -I$(BUILD_ROOT)/Fw/Types/StdIntInc


# Some warning override variables. These will go in individual modules where the warning-as-error needs to be overridden

NO_STRICT_ALIASING := -fno-strict-aliasing
NO_WARNINGS := -w

DEBUG := -g -gdwarf-2

OPT_SPEED := -O2

OPT_SIZE := -O4 

OPT_NONE := -On

SYMBOL_CHECK := $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.py
