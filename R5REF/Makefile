# Makefile to run global make.
DEPLOYMENT := R5REF
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))

export BUILD_ROOT

default_build: TIR5

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

rebuild_r5:
	make gen_make TIR5_clean TIR5
