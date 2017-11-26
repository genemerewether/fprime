# Makefile to run global make.
DEPLOYMENT := HEXREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: DSPAL #DSPAL_opt

COV_BUILD: DSPAL_bin_clean DSPAL

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

#load: default_build
#	adb push ./linux-linaro-cross-arm-opt-gnu-bin/SDREF /usr/share/data/adsp

rebuild_hex:
	make gen_make DSPAL_clean DSPAL #DSPAL_opt_clean DSPAL_opt
