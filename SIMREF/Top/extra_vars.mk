-include $(BUILD_ROOT)/ROS/fprime_ws/src/fprime/mod.mk

EXTRA_INC_DIRS_SIMREFTop_DARWIN := $(EXTRA_INC_DIRS_SIMREFTop_DARWIN) $(EXTRA_INC_DIRS_DARWIN) $(EXTRA_INC_DIRS)
EXTRA_LIBS_SIMREFTop_DARWIN := $(EXTRA_LIBS_SIMREFTop_DARWIN) $(EXTRA_LIBS_DARWIN) $(EXTRA_LIBS)

EXTRA_INC_DIRS_SIMREFTop_SDFLIGHT := $(EXTRA_INC_DIRS_SIMREFTop_SDFLIGHT) $(EXTRA_INC_DIRS_SDFLIGHT) $(EXTRA_INC_DIRS)
EXTRA_LIBS_SIMREFTop_SDFLIGHT := $(EXTRA_LIBS_SIMREFTop_SDFLIGHT) $(EXTRA_LIBS_SDFLIGHT) $(EXTRA_LIBS)

EXTRA_INC_DIRS_SIMREFTop_LINUX := $(EXTRA_INC_DIRS_SIMREFTop_LINUX) $(EXTRA_INC_DIRS_LINUX) $(EXTRA_INC_DIRS)
EXTRA_LIBS_SIMREFTop_LINUX := $(EXTRA_LIBS_SIMREFTop_LINUX) $(EXTRA_LIBS_LINUX) $(EXTRA_LIBS)

EXTRA_INC_DIRS_DARWIN=
EXTRA_LIBS_DARWIN=
EXTRA_INC_DIRS_SDFLIGHT=
EXTRA_LIBS_SDFLIGHT=
EXTRA_INC_DIRS_LINUX=
EXTRA_LIBS_LINUX=
EXTRA_INC_DIRS=
EXTRA_LIBS=

-include $(BUILD_ROOT)/Gnc/quest_external/traj/ewok/inc.mk

EXTRA_INC_DIRS_SIMREFTop_DARWIN := $(EXTRA_INC_DIRS_SIMREFTop_DARWIN) $(EXTRA_INC_DIRS_DARWIN) $(EXTRA_INC_DIRS)
EXTRA_LIBS_SIMREFTop_DARWIN := $(EXTRA_LIBS_SIMREFTop_DARWIN) $(EXTRA_LIBS_DARWIN) $(EXTRA_LIBS)

EXTRA_INC_DIRS_SIMREFTop_SDFLIGHT := $(EXTRA_INC_DIRS_SIMREFTop_SDFLIGHT) $(EXTRA_INC_DIRS_SDFLIGHT) $(EXTRA_INC_DIRS)
EXTRA_LIBS_SIMREFTop_SDFLIGHT := $(EXTRA_LIBS_SIMREFTop_SDFLIGHT) $(EXTRA_LIBS_SDFLIGHT) $(EXTRA_LIBS)

EXTRA_INC_DIRS_SIMREFTop_LINUX := $(EXTRA_INC_DIRS_SIMREFTop_LINUX) $(EXTRA_INC_DIRS_LINUX) $(EXTRA_INC_DIRS)
EXTRA_LIBS_SIMREFTop_LINUX := $(EXTRA_LIBS_SIMREFTop_LINUX) $(EXTRA_LIBS_LINUX) $(EXTRA_LIBS)

EXTRA_INC_DIRS_DARWIN=
EXTRA_LIBS_DARWIN=
EXTRA_INC_DIRS_SDFLIGHT=
EXTRA_LIBS_SDFLIGHT=
EXTRA_INC_DIRS_LINUX=
EXTRA_LIBS_LINUX=
EXTRA_INC_DIRS=
EXTRA_LIBS=
