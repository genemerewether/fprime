include $(BUILD_ROOT)/mk/configs/compiler/include_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/defines_common.mk
include $(BUILD_ROOT)/mk/configs/compiler/cubemx-common.mk

CUBEMX_STM32F4_COMMON_FLAGS :=	$(COMMON_DEFINES) \
						$(CUBEMX_FLAGS_COMMON) \
						-DUSE_HAL_DRIVER \
						-DSTM32F405xx \
						-mcpu=cortex-m4 -mthumb \
						-mfpu=fpv4-sp-d16 -mfloat-abi=hard \
						-DEIGEN_NO_DEBUG -D'EIGEN_ASM_COMMENT(X)=(X)' \
						-DEIGEN_NO_MALLOC \
						-D'EIGEN_ALIGN_TO_BOUNDARY(n)=__attribute__((aligned(n)))' \
						-D'EIGEN_ALIGNOF(x)=__alignof(x)' \
						-D'EIGEN_HAS_CXX11_ATOMIC=0'

CUBEMX_STM32F4_COMMON_INCLUDES := 	$(COMMON_INCLUDES) \
					$(CUBEMX_INCLUDES_COMMON) \
	-I$(BUILD_ROOT)/STM32/CF2Hal/Inc \
	-I$(BUILD_ROOT)/STM32/CF2Hal/Drivers/STM32F4xx_HAL_Driver/Inc \
	-I$(BUILD_ROOT)/STM32/CF2Hal/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
	-I$(BUILD_ROOT)/STM32/CF2Hal/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
	-I$(BUILD_ROOT)/STM32/CF2Hal/Drivers/CMSIS/Include

COVERAGE := 

CC :=  arm-none-eabi-gcc
CXX := $(CC)

LINK_LIB := arm-none-eabi-ar
LINK_LIB_FLAGS := r
POST_LINK_LIB := $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.py

LINK_LIBS := -lc -lm -lnosys

LINK_BIN :=  $(CC)
LINK_BIN_FLAGS := -specs=nano.specs -T$(BUILD_ROOT)/STM32/CF2Hal/STM32F405RGTx_FLASH.ld \
		  -Wl,-Map=CF2Bin.map,--cref -Wl,--gc-sections

FILE_SIZE := $(LS) $(LS_SIZE)
LOAD_SIZE := $(SIZE)


DUMP = $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.py
SYMBOL_SIZES = $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.py
MUNCH := $(PYTHON_BIN) $(BUILD_ROOT)/mk/bin/empty.py
