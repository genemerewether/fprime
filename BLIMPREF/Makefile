# Makefile to run global make.
DEPLOYMENT := BLIMPREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

FSW_RUN_SCRIPT = runBLIMPREF.sh
__DEFAULT_BUILD = SDFLIGHT_opt
__DEFAULT_BUILD_OUT = linux-linaro-cross-arm-opt-gnu-bin

default_build: $(__DEFAULT_BUILD)

COV_BUILD: SDFLIGHT_bin_clean SDFLIGHT

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

include $(BUILD_ROOT)/Svc/Startup/patch/patch.mk

rebuild_sd:
	make gen_make SDFLIGHT_opt_clean SDFLIGHT_opt dict_install

only_load:
	adb push ./$(__DEFAULT_BUILD_OUT)/$(DEPLOYMENT) /eng/load/$(DEPLOYMENT)
