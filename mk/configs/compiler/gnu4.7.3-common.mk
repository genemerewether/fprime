GNU_COMMON := -Wall -Wextra \
               -fno-builtin -fno-asm \
               -fstrength-reduce \
               -Wno-unused-parameter \
               -Wno-long-long
               
DEPEND_FILE := -MMD -MP -MF

GNU_CFLAGS_COMMON := $(GNU_COMMON) -std=c99 \
				-fno-cond-mismatch -pedantic \
				-Werror-implicit-function-declaration \
				-Wstrict-prototypes

GNU_CXXFLAGS_COMMON := $(GNU_COMMON)  -fcheck-new \
				-fno-rtti

GNU_VERS := 4.7.3

# Special compiler flags to get around known AC warnings

AC_CC_FLAGS :=	-Wno-extra
AC_CXX_FLAGS :=
AC_HSM_FLAGS := -Wno-extra -Wno-parentheses
AC_PARAMS_FLAGS := -fno-strict-aliasing	-Wno-extra	

COMPILE_ONLY := -c

COMPILE_TO := -o
LIBRARY_TO := -o
LINK_BIN_PRE_TO := -o
LINK_BIN_POST_TO := -o

INCLUDE_PATH := -I


DEBUG := -g3

OPT_SPEED := -Os

OPT_NONE := -O0

SYMBOL_CHECK := /proj/msl/fsw/tools/bin/extract_symbols u