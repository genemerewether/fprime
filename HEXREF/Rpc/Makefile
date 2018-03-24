# This Makefile goes in each module, and allows building of an individual module library.
# It is expected that each developer will add targets of their own for building and running
# tests, for example.

# derive module name from directory

MODULE_DIR = HEXREF/Rpc
MODULE = $(subst /,,$(MODULE_DIR))

BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))
export BUILD_ROOT

include $(BUILD_ROOT)/mk/makefiles/module_targets.mk

# TODO(mereweth) - replace this with cleaner dependencies
idl: hexref_skel.c hexref_stub.c hexref.h

hexref_skel.c hexref_stub.c hexref.h: hexref.idl
	$(HEXAGON_SDK_ROOT)/tools/qaic/Ubuntu14/qaic -mdll -I $(HEXAGON_SDK_ROOT)/incs/stddef hexref.idl

rebuild_nav: ut_nocov_clean_SDFLIGHT ut_nocov_SDFLIGHT

load: ut_nocov_SDFLIGHT
	adb push ./test/ut/linux-linaro-cross-arm-ut-nocov-gnu-bin/test_ut /home/linaro/hexref_rpc_test_ut

.PHONY: idl load rebuild_nav

# Add module specific targets here
