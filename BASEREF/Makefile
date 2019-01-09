# Makefile to run global make.
DEPLOYMENT := BASEREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk
