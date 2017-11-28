COMP ?= DEFAULT

ifeq ($(COMP),DEFAULT)
 include $(BUILD_ROOT)/mk/configs/builds/sdflight-gcc-debug.mk
endif

ifeq ($(COMP),gcc-debug)
 include $(BUILD_ROOT)/mk/configs/builds/sdflight-gcc-debug.mk
endif

ifeq ($(COMP),gcc-opt)
 include $(BUILD_ROOT)/mk/configs/builds/sdflight-gcc-opt.mk
endif

ifeq ($(COMP),comp-opt)
 include $(BUILD_ROOT)/mk/configs/builds/sdflight-gcc-opt.mk
endif

ifeq ($(COMP),comp-ut)
 include $(BUILD_ROOT)/mk/configs/builds/sdflight-gcc-ut-nocov.mk
endif

ifeq ($(COMP),comp-ut-nocov)
 include $(BUILD_ROOT)/mk/configs/builds/sdflight-gcc-ut-nocov.mk
endif
