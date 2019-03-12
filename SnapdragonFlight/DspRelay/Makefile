# This Makefile goes in each module, and allows building of an individual module library.
# It is expected that each developer will add targets of their own for building and running
# tests, for example.

# derive module name from directory

MODULE_DIR = SnapdragonFlight/DspRelay
MODULE = $(subst /,,$(MODULE_DIR))

BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))
export BUILD_ROOT

include $(BUILD_ROOT)/mk/makefiles/module_targets.mk

# Add module specific targets here

# TODO(mereweth) - replace this with cleaner dependencies
idl: dsp_relay_skel.c dsp_relay_stub.c dsp_relay.h

dsp_relay_skel.c dsp_relay_stub.c dsp_relay.h: dsp_relay.idl
	$(HEXAGON_SDK_ROOT)/tools/qaic/Ubuntu14/qaic -mdll -I $(HEXAGON_SDK_ROOT)/incs/stddef dsp_relay.idl
