#
#   Copyright 2004-2016, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.
#
#   This software may be subject to U.S. export control laws and
#   regulations.  By accepting this document, the user agrees to comply
#   with all U.S. export laws and regulations.  User has the
#   responsibility to obtain export licenses, or other export authority
#   as may be required before exporting such information to foreign
#   countries or providing access to foreign persons.
#

# This makefile can be included by other makefiles to provide patching
# functionality for the `Svc::Startup` scripts

__DEFAULT_BUILD ?= SDFLIGHT_opt
__DEFAULT_BUILD_OUT ?= linux-linaro-cross-arm-opt-gnu-bin

GIT_HASH := $(shell git log --pretty=format:%h -n 1)
GSE_FILENAME := $(DEPLOYMENT)_$(GIT_HASH)_Gse.tar.gz
PACKET_XML_FILENAME := $(DEPLOYMENT)_$(GIT_HASH)_Packets.xml

SHELL := /bin/bash
STARTUP_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))/..

# Check if environment variable VER is set; otherwise generate name
ifndef VER
FULLVER := $(shell $(STARTUP_DIR)/patch/mkver.bash -d $(DEPLOYMENT))
ifeq ($(FULLVER),)
	$(error VER is undefined and naming date/time-stamped dir failed)
endif
VER_UNSET := 1
else
FULLVER := $(shell $(STARTUP_DIR)/patch/mkver.bash -d $(DEPLOYMENT) -v $(VER))
endif

# TODO(mereweth) - remove once nested namespace serializable Gse is working
rosser:
	touch $(BUILD_ROOT)/$(DEPLOYMENT)/py_dict/serializable/ROS/__init__.py

# For switching fsw partition between read-only and read-write
rwfsw:
	$(BUILD_ROOT)/Svc/Startup/patch/remount_fsw_rw.sh

edmrwfsw:
	$(BUILD_ROOT)/Svc/Startup/patch/remount_edm_fsw_rw.sh

rwroot:
	$(BUILD_ROOT)/Svc/Startup/patch/remount_root_rw.sh

edmrwroot:
	$(BUILD_ROOT)/Svc/Startup/patch/remount_edm_root_rw.sh

rofsw:
	$(BUILD_ROOT)/Svc/Startup/patch/remount_fsw_ro.sh

edmrofsw:
	$(BUILD_ROOT)/Svc/Startup/patch/remount_edm_fsw_ro.sh

start:
	adb shell service startup start

screen:
	adb shell 'kill -CHLD `pgrep screen`'

stop: screen
	adb shell service startup stop
	adb shell "screen -S RUN_ONCE -X stuff "
	adb shell "screen -S CURRENT -X stuff "
	adb shell "screen -S GOLDEN -X stuff "
	adb shell "sleep 10"
	adb shell "screen -S RUN_ONCE -X quit"
	adb shell "screen -S CURRENT -X quit"
	adb shell "screen -S GOLDEN -X quit"
	adb shell 'for i in {0..3}; do echo performance > /sys/devices/system/cpu/cpu$${i}/cpufreq/scaling_governor; done'
	adb shell "chronyc -m \"password 1234\" makestep"

makestep:
	adb shell "chronyc -m \"password 1234\" makestep"

power:
	adb shell 'for i in {0..3}; do echo performance > /sys/devices/system/cpu/cpu$${i}/cpufreq/scaling_governor; done'

restart: stop start

date:
	$(STARTUP_DIR)/patch/adb_set_date.sh

# Create directory for new FSW version
mkver:
ifneq ($(VER_UNSET),)
	@echo "VER is undefined; using date/time stamped dir $(FULLVER)"
endif
	adb shell mkdir $(FULLVER)

gsetar: dict_install
	tar -C $(BUILD_ROOT)/$(DEPLOYMENT)/py_dict -czf ./$(__DEFAULT_BUILD_OUT)/$(GSE_FILENAME) .

gseload: gsetar
	adb push ./$(__DEFAULT_BUILD_OUT)/$(GSE_FILENAME) $(FULLVER)/$(GSE_FILENAME)
	rm ./$(__DEFAULT_BUILD_OUT)/$(GSE_FILENAME)

gsegold: gsetar
	adb push ./$(__DEFAULT_BUILD_OUT)/$(GSE_FILENAME) /golden/$(GSE_FILENAME)
	rm ./$(__DEFAULT_BUILD_OUT)/$(GSE_FILENAME)

# Only set the appropriate symlink
check-env:
ifneq ($(VER_UNSET),)
	#Use as follows: VER=my_ver make setonce
	$(error VER is undefined and no new binary loaded)
endif

setonce: check-env
	$(STARTUP_DIR)/patch/setup_folder.bash -d $(DEPLOYMENT) -f $(FULLVER) -t setonce

setcur: check-env
	$(STARTUP_DIR)/patch/setup_folder.bash -d $(DEPLOYMENT) -f $(FULLVER) -t setcur

# Only load a new FSW version
release: mkver $(__DEFAULT_BUILD) gseload

ifdef NOLOAD
	$(STARTUP_DIR)/patch/setup_folder.bash -i ./$(__DEFAULT_BUILD_OUT) -d $(DEPLOYMENT) -f $(FULLVER) -t load -n
else
	$(STARTUP_DIR)/patch/setup_folder.bash -i ./$(__DEFAULT_BUILD_OUT) -d $(DEPLOYMENT) -f $(FULLVER) -t load
endif

ifneq ($(FSW_RUN_SCRIPT),)
	adb push $(FSW_RUN_SCRIPT) $(FULLVER)/$(FSW_RUN_SCRIPT)
endif
ifneq ($(FSW_RUN_SCRIPT_AUX),)
	adb push $(FSW_RUN_SCRIPT_AUX) $(FULLVER)/$(FSW_RUN_SCRIPT_AUX)
endif
ifneq ($(PACKET_XML),)
	adb push $(PACKET_XML) $(FULLVER)/$(PACKET_XML_FILENAME)
endif

	adb shell sync

load: $(__DEFAULT_BUILD)
	adb shell mkdir -p /eng/load

ifdef NOLOAD
	$(STARTUP_DIR)/patch/setup_folder.bash -i ./$(__DEFAULT_BUILD_OUT) -d $(DEPLOYMENT) -f /eng/load -t load -n
else
	$(STARTUP_DIR)/patch/setup_folder.bash -i ./$(__DEFAULT_BUILD_OUT) -d $(DEPLOYMENT) -f /eng/load -t load
endif

ifneq ($(FSW_RUN_SCRIPT),)
	adb push $(FSW_RUN_SCRIPT) /eng/load/$(FSW_RUN_SCRIPT)
endif
ifneq ($(FSW_RUN_SCRIPT_AUX),)
	adb push $(FSW_RUN_SCRIPT_AUX) /eng/load/$(FSW_RUN_SCRIPT_AUX)
endif

	adb shell sync

# Load new FSW version and update appropriate symlink
once: release
	$(STARTUP_DIR)/patch/setup_folder.bash -d $(DEPLOYMENT) -f $(FULLVER) -t setonce

cur: release
	$(STARTUP_DIR)/patch/setup_folder.bash -d $(DEPLOYMENT) -f $(FULLVER) -t setcur

# Load the golden FSW version
gold: $(__DEFAULT_BUILD) gsegold
ifdef NOLOAD
	$(STARTUP_DIR)/patch/setup_folder.bash -i ./$(__DEFAULT_BUILD_OUT) -d $(DEPLOYMENT) -t gold -n
else
	$(STARTUP_DIR)/patch/setup_folder.bash -i ./$(__DEFAULT_BUILD_OUT) -d $(DEPLOYMENT) -t gold
endif

ifneq ($(FSW_RUN_SCRIPT),)
	adb push $(FSW_RUN_SCRIPT) /golden/$(FSW_RUN_SCRIPT)
endif
ifneq ($(FSW_RUN_SCRIPT_AUX),)
	adb push $(FSW_RUN_SCRIPT_AUX) /golden/$(FSW_RUN_SCRIPT_AUX)
endif
ifneq ($(PACKET_XML),)
	adb push $(PACKET_XML) /golden/$(PACKET_XML_FILENAME)
endif

	adb shell sync

rebuild_$(DEPLOYMENT): check_platform
	make gen_make $(__DEFAULT_BUILD)_clean $(__DEFAULT_BUILD) dict_install

.PHONY: rwfsw ecmrwfsw rofsw ecmrofsw cur once gold rebuild_$(DEPLOYMENT) setcur setonce check-env mkver gsetar gseload gsegold stop screen start restart power makestep
