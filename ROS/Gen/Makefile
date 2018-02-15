MODULE_DIR = ROS/Gen
BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))
export BUILD_ROOT
# include $(BUILD_ROOT)/mk/makefiles/module_targets.mk

#SUBDIR := $(shell find . -type d -name '*_msgs')

SUBDIRS := std_msgs/Types \
	   actionlib_msgs/Types \
	   diagnostic_msgs/Types \
	   geometry_msgs/Types \
	   nav_msgs/Types \
	   rosgraph_msgs/Types \
	   sensor_msgs/Types \
	   shape_msgs/Types \
	   std_srvs/Types \
	   stereo_msgs/Types \
	   trajectory_msgs/Types \
	   visualization_msgs/Types \
	   planning_msgs/Types \
	   mav_msgs/Types \
	   \
	   std_msgs/Ports \
	   actionlib_msgs/Ports \
	   diagnostic_msgs/Ports \
	   geometry_msgs/Ports \
	   nav_msgs/Ports \
	   rosgraph_msgs/Ports \
	   sensor_msgs/Ports \
	   shape_msgs/Ports \
	   std_srvs/Ports \
	   stereo_msgs/Ports \
	   trajectory_msgs/Ports \
	   visualization_msgs/Ports \
	   planning_msgs/Ports \
	   mav_msgs/Ports

all:
	$(foreach dir, $(SUBDIRS), echo $(dir); cd $(dir); make -j 4; cd $(CURDIR))

clean:
	$(foreach dir, $(SUBDIRS), echo $(dir); cd $(dir); $(MAKE) clean; cd $(CURDIR))

#$(foreach dir, $(SUBDIRS), rm -r $(dir)/Ports/* $(dir)/Types/*;)

#$(BUILDS):
	#$(foreach dir, $(SUBDIRS), echo $(dir); cd $(dir); $(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile BUILD=$@ $(subst .,,$(subst /,,ROS/Gen/$(dir)/Types)); cd -)
	#$(foreach dir, $(SUBDIRS), echo $(dir); cd $(dir); $(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile BUILD=$@ $(subst .,,$(subst /,,ROS/Gen/$(dir)/Ports)); cd -)

.PHONY: clean all #$(BUILDS)
