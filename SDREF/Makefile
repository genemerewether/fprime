# Makefile to run global make.
DEPLOYMENT := SDREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: SDFLIGHT_opt

COV_BUILD: SDFLIGHT_bin_clean SDFLIGHT

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

load: default_build
	adb push ./linux-linaro-cross-arm-opt-gnu-bin/SDREF /home/linaro

rebuild_sd:
	make gen_make SDFLIGHT_opt_clean SDFLIGHT_opt dict_install
