# Makefile to run global make.
DEPLOYMENT := BLIMPSIMREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

rosser:
	touch $(BUILD_ROOT)/Gse/generated/$(DEPLOYMENT)/serializable/ROS/__init__.py

load: SDFLIGHT_opt
	adb push ./linux-linaro-cross-arm-opt-gnu-bin/$(DEPLOYMENT) /eng/load/$(DEPLOYMENT)
