COMP ?= DEFAULT

ifeq ($(COMP),DEFAULT)
 include $(BUILD_ROOT)/mk/configs/builds/pru-am335x-cgt-debug.mk
endif

ifeq ($(COMP),comp-debug)
 include $(BUILD_ROOT)/mk/configs/builds/pru-am335x-cgt-debug.mk
endif

ifeq ($(COMP),comp-ut)
 include $(BUILD_ROOT)/mk/configs/builds/pru-am335x-cgt-ut.mk
endif

