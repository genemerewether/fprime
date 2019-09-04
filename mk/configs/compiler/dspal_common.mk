DSPAL_FLAGS_COMMON := -DTGT_OS_TYPE_DSPAL

ifneq ($(TARGET_8096),)
DSPAL_FLAGS_COMMON += -DTARGET_8096
endif

DSPAL_INCLUDES_COMMON := -I$(BUILD_ROOT)/Fw/Types/Dspal \
			 -I$(BUILD_ROOT)/SnapdragonFlight/dspal/include \
			 -I$(BUILD_ROOT)/SnapdragonFlight/
