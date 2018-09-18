COMP ?= DEFAULT

ifeq ($(COMP),DEFAULT)
 include $(BUILD_ROOT)/mk/configs/builds/tir5-tms570lc43x-ccs-debug-opt.mk
endif

ifeq ($(COMP),comp-ut)
 include $(BUILD_ROOT)/mk/configs/builds/tir5-tms570lc43x-ccs-ut.mk
endif

ifeq ($(COMP),comp-debug)
 include $(BUILD_ROOT)/mk/configs/builds/tir5-tms570lc43x-ccs-debug-opt.mk
endif

ifeq ($(COMP),comp-integ)
 include $(BUILD_ROOT)/mk/configs/builds/tir5-tms570lc43x-ccs-ut.mk
endif
