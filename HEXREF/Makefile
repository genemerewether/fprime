# Makefile to run global make.
DEPLOYMENT := HEXREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: DSPAL_debug

COV_BUILD: DSPAL_bin_clean DSPAL

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

#load: default_build
#	adb push ./linux-linaro-cross-arm-opt-gnu-bin/SDREF /usr/share/data/adsp

rebuild_sd:
	make gen_make DSPAL_debug_clean DSPAL_debug
