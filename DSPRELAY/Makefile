# Makefile to run global make.
DEPLOYMENT := DSPRELAY
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: DSPAL_opt

COV_BUILD: DSPAL_bin_clean DSPAL

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

load_8074: default_build
	adb push ./dspal-hex-clang-cross-opt-dspal-bin/DSPRELAY /usr/share/data/adsp/libdsp_relay_skel.so

load_8096: default_build
	adb push ./dspal-hex-clang-cross-opt-dspal-bin/DSPRELAY /usr/lib/rfsa/adsp/libdsp_relay_skel.so

rebuild_hex:
	make gen_make DSPAL_opt_clean DSPAL_opt
