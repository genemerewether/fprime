# Makefile to run global make.
DEPLOYMENT := R5RELAY
BUILD_ROOT ?= $(subst /$(DEPLOYMENT),,$(CURDIR))
BUILD_SITE := davinci

export BUILD_ROOT

all: LINUX

include $(BUILD_ROOT)/mk/makefiles/deployment_makefile.mk

include $(BUILD_ROOT)/Svc/Startup/patch/patch.mk

