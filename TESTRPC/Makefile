# Makefile to run global make.
DEPLOYMENT := TESTRPC
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

__DEFAULT_BUILD = DSPAL_opt
__DEFAULT_BUILD_OUT = dspal-hex-clang-cross-opt-dspal-bin

default_build: $(__DEFAULT_BUILD)

COV_BUILD: DSPAL_bin_clean DSPAL

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

load_8074: default_build
	adb push ./$(__DEFAULT_BUILD_OUT)/TESTRPC /usr/share/data/adsp/libhexref_skel.so

only_load_8074:
	adb push ./$(__DEFAULT_BUILD_OUT)/TESTRPC /usr/share/data/adsp/libhexref_skel.so

load_8096: default_build
	adb push ./$(__DEFAULT_BUILD_OUT)/TESTRPC /usr/lib/rfsa/adsp/libhexref_skel.so

only_load_8096:
	adb push ./$(__DEFAULT_BUILD_OUT)/TESTRPC /usr/lib/rfsa/adsp/libhexref_skel.so

rebuild_hex:
	make gen_make DSPAL_opt_clean DSPAL_opt
