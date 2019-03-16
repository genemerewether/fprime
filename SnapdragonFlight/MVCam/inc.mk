EXTRA_INC_DIRS_SDFLIGHT = $(foreach sysroot,$(INCLUDE_ARM_SYSROOTS), -I$(sysroot)/usr/include/mv/)
EXTRA_LIBS_SDFLIGHT = -lmv1 -lcamera
