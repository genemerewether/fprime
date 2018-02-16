MODULE_DIR = ROS/Gen
BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))
export BUILD_ROOT

#SUBDIR := $(shell find . -type d -name '*_msgs')

TYPE_SUBDIRS := std_msgs/Types \
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
	   mav_msgs/Types

PORT_SUBDIRS := \
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

all: ports types

types:
	$(foreach dir, $(TYPE_SUBDIRS), echo $(dir); cd $(dir); make -j 4; cd $(CURDIR))

ports:
	$(foreach dir, $(PORT_SUBDIRS), echo $(dir); cd $(dir); make -j 4; cd $(CURDIR))

clean: clean_ports clean_types

clean_types:
	$(foreach dir, $(TYPE_SUBDIRS), echo $(dir); cd $(dir); $(MAKE) clean; cd $(CURDIR))

clean_ports:
	$(foreach dir, $(PORT_SUBDIRS), echo $(dir); cd $(dir); $(MAKE) clean; cd $(CURDIR))

.PHONY: clean all clean_ports clean_types ports types #$(BUILDS)
