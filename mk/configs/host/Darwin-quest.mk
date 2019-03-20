include $(BUILD_ROOT)/mk/configs/host/Darwin-generic.mk

TI_CCS_DIR := /Applications/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.3.LTS

export TURBOJPEG := /usr/local/Cellar/jpeg-turbo/1.5.1
export TURBOJPEG_INCLUDE := -I$(TURBOJPEG)/include/
