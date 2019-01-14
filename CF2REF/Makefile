# Makefile to run global make.
DEPLOYMENT := CF2REF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: CF2

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

rebuild_cf2:
	make gen_make CF2_clean CF2
