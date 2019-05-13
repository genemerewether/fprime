include $(BUILD_ROOT)/mk/configs/host/Linux-common.mk

export PYTHON_BIN := python

# This assumes that you have installed lxml and cheetah to the python distribution. Probably requires sudo access.
export LXML_PATH := 
export CHEETAH_COMPILE ?= cheetah-compile
export MARKDOWN ?= markdown_py -x markdown.extensions.extra -x markdown.extensions.codehilite

CKSUM_SH := $(BUILD_ROOT)/mk/bin/md5.sh
export CKSUM := /usr/bin/openssl md5
CKSUM_SUFFIX := .md5

DOXYGEN := 

JOBS := -j `nproc`
#JOBS =

-include $(BUILD_ROOT)/mk/configs/target/wind-$(TARGET).mk

CRC := $(BUILD_ROOT)/mk/bin/run_file_crc.sh

<<<<<<< HEAD
=======

export PI_TOOLS := /opt/tools/rpi


>>>>>>> upstream-pub-fprime/master
SYMBOL_CHECK := echo
