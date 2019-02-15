include $(BUILD_ROOT)/mk/configs/host/Linux-common.mk
# This assumes that you have installed lxml and cheetah to the python distribution. Probably requires sudo access.

export PYTHON_BASE ?= /usr
export PYTHON_BIN := $(PYTHON_BASE)/bin/python
export LD_LIBRARY_PATH := $(LD_LIBRARY_PATH):$(PYTHON_BASE)/lib

export LXML_PATH :=
export CHEETAH_COMPILE ?= cheetah-compile
export MARKDOWN ?= $(PYTHON_BASE)/bin/markdown_py -x markdown.extensions.extra -x markdown.extensions.codehilite

JOBS := -j `nproc`

TI_CCS_DIR := /opt/ti/ccsv8/tools/compiler/ti-cgt-arm_18.1.2.LTS

export HEXAGON_SDK_ROOT := /opt/tools/quest/Qualcomm/Hexagon_SDK/3.0
export HEXAGON_TOOLS_ROOT := /opt/tools/quest/Qualcomm/HEXAGON_Tools/7.2.12/Tools
export HEXAGON_ARM_SYSROOT := /opt/tools/quest/Qualcomm/qrlinux_sysroot

export INDIGO_ARM_SYSROOT :=  /opt/tools/quest/Qualcomm/indigo_sysroot

CRC := $(BUILD_ROOT)/mk/bin/run_file_crc.sh

NM:= /usr/bin/nm

SYMBOL_CHECK := echo
