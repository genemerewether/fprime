# Makefile to run global make.
DEPLOYMENT := SDREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: SDFLIGHT_opt

COV_BUILD: SDFLIGHT_bin_clean SDFLIGHT

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

include $(BUILD_ROOT)/Svc/Startup/patch/patch.mk

rebuild_sd:
	make gen_make SDFLIGHT_opt_clean SDFLIGHT_opt dict_install
