# Makefile to run global make.
DEPLOYMENT := HEXREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: DSPAL_opt

COV_BUILD: DSPAL_bin_clean DSPAL

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

load: default_build
	adb push ./dspal-hex-clang-cross-opt-dspal-bin/HEXREF /usr/share/data/adsp/libhexref_skel.so

rebuild_hex:
	make gen_make DSPAL_opt_clean DSPAL_opt
