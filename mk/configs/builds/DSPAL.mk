COMP ?= DEFAULT

ifeq ($(COMP),DEFAULT)
 include $(BUILD_ROOT)/mk/configs/builds/dspal-clang-debug.mk
endif

ifeq ($(COMP),clang-debug)
 include $(BUILD_ROOT)/mk/configs/builds/dspal-clang-debug.mk
endif

ifeq ($(COMP),clang-opt)
 include $(BUILD_ROOT)/mk/configs/builds/dspal-clang-opt.mk
endif

ifeq ($(COMP),comp-opt)
 include $(BUILD_ROOT)/mk/configs/builds/dspal-clang-opt.mk
endif

ifeq ($(COMP),comp-ut)
 include $(BUILD_ROOT)/mk/configs/builds/dspal-clang-ut-nocov.mk
endif

ifeq ($(COMP),comp-ut-nocov)
 include $(BUILD_ROOT)/mk/configs/builds/dspal-clang-ut-nocov.mk
endif
