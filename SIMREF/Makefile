# Makefile to run global make.
DEPLOYMENT := SIMREF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: DARWIN

COV_BUILD: DARWIN_bin_clean DARWIN

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk
