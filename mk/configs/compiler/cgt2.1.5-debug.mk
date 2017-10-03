include $(BUILD_ROOT)/mk/configs/compiler/pru_cgt_common.mk

CFLAGS := $(DEBUG) \
	  \
	  $(OPT_SIZE) \
	  \
	  $(PRU_FLAGS_COMMON) \
	  \
	  $(CGT_CFLAGS_COMMON) \
	  \
	  $(PRU_CGT_FLAGS_COMMON) \
	  $(PRU_CGT_INCLUDES_COMMON) \
	  \
	  $(PRU_INCLUDES_COMMON) \
	  --gen_func_subsections --printf_support=minimal --float_operations_allowed=none -k \
	  --output_all_syms --embedded_cpp

CXXFLAGS := $(DEBUG) \
	    \
	    $(OPT_SIZE) \
	    \
	    $(PRU_FLAGS_COMMON) \
	    \
	    $(CGT_CXXFLAGS_COMMON) \
	    \
	    $(PRU_CGT_FLAGS_COMMON) \
	    $(PRU_CGT_INCLUDES_COMMON) \
	    \
	    $(PRU_INCLUDES_COMMON) \
	    --gen_func_subsections --printf_support=minimal --float_operations_allowed=none -k \
	    --output_all_syms --embedded_cpp

LINK_BIN_FLAGS := $(LINK_BIN_FLAGS) \
		  -o4

LINK_LIBS := $(LINK_LIBS) \
	     $(BUILD_ROOT)/PRU/AM335x_PRU.cmd

COMPILER := cgt2.1.5-debug
