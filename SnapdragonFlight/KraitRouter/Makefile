# This Makefile goes in each module, and allows building of an individual module library.
# It is expected that each developer will add targets of their own for building and running
# tests, for example.

# derive module name from directory

MODULE_DIR = SnapdragonFlight/KraitRouter
MODULE = $(subst /,,$(MODULE_DIR))

BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))
export BUILD_ROOT

include $(BUILD_ROOT)/mk/makefiles/module_targets.mk

load: DSPAL
	adb push ./test/ut/dspal-hex-clang-cross-debug-dspal-bin/test_ut /usr/share/data/adsp/libhexref_skel.so

# Add module specific targets here
