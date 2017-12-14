obj-m := dsp-offset.o
SRC := $(shell pwd)
KERNEL_SRC := /lib/modules/$(shell uname -r)/build

# uncomment this to show debug prints
CFLAGS_dsp-offset.o := -DDEBUG

all:
	make -C $(KERNEL_SRC) M=$(SRC) modules

clean:
	make -C $(KERNEL_SRC) M=$(SRC) clean

modules_install:
	make INSTALL_MOD_DIR=kernel/drivers/dsp -C $(KERNEL_SRC) M=$(SRC) modules_install
