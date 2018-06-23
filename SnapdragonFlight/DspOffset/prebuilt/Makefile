PUSH_FILE := /lib/modules/3.4.0-eagle8074/kernel/drivers/dsp/dsp-offset.ko

taskset_cpustat_mqueue: boot_load
	adb push dsp-offset_taskset_cpustat_mqueue.ko $(PUSH_FILE)

boot_load:
	adb shell 'grep dsp-offset /etc/modules || echo dsp-offset >> /etc/modules'

.PHONY: taskset_cpustat_mqueue boot_load
