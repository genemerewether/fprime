# TODO(genemerewether@gmail.com) confirm which error is #255
CGT_FLAGS_COMMON := --display_error_number --diag_wrap=off # --diag_warning=255

DEPEND_FILE := -ppa -ppd=

CGT_CFLAGS_COMMON := $(CGT_FLAGS_COMMON) --c99

CGT_CXXFLAGS_COMMON := $(CGT_FLAGS_COMMON) --c++03


AC_CC_FLAGS :=
AC_CXX_FLAGS :=
AC_HSM_FLAGS :=
AC_PARAMS_FLAGS :=

COMPILE_ONLY := --compile_only
COMPILE_TO := --output_file
LIBRARY_TO :=
LINK_BIN_TO := -o
INCLUDE_PATH := --include_path=

CGT_INCLUDES_COMMON :=

NO_STRICT_ALIASING := -fno-strict-aliasing
NO_WARNINGS := -w

DEBUG := -g

OPT_SPEED := --opt_level=2 --opt_for_speed=5

OPT_SIZE := --opt_level=4 --opt_for_speed=0

OPT_NONE := -On

SYMBOL_CHECK := $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.py