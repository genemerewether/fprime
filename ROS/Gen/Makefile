MODULE_DIR = ROS/Gen
BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))
export BUILD_ROOT
include ../../mk/makefiles/module_targets.mk

SUBDIR := $(shell find . -type d -name '*_msgs')
#SUBDIR := std_msgs

clean_msgs:
	$(foreach dir, $(SUBDIR), rm -r $(dir);)

$(BUILDS): ac_lvl4
	$(foreach dir, $(SUBDIR), echo $(dir); cd $(dir); $(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile BUILD=$@ $(subst .,,$(subst /,,ROS/Gen/$(dir)/Types)); cd -)
	#$(foreach dir, $(SUBDIR), echo $(dir); cd $(dir); $(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile BUILD=$@ $(subst .,,$(subst /,,ROS/Gen/$(dir)/Ports)); cd -)

.PHONY: $(BUILDS) clean_msgs
