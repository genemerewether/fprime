COMP ?= DEFAULT

ifeq ($(COMP),DEFAULT)
 include $(BUILD_ROOT)/mk/configs/builds/cf2-stm32f405rg-cubemx-debug-opt.mk
endif

ifeq ($(COMP),comp-ut)
 include $(BUILD_ROOT)/mk/configs/builds/cf2-stm32f405rg-cubemx-ut.mk
endif

ifeq ($(COMP),comp-debug)
 include $(BUILD_ROOT)/mk/configs/builds/cf2-stm32f405rg-cubemx-debug-opt.mk
endif

ifeq ($(COMP),comp-integ)
 include $(BUILD_ROOT)/mk/configs/builds/cf2-stm32f405rg-cubemx-ut.mk
endif
