# Makefile to run global make.
DEPLOYMENT := SIMREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk
