/*
 * Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

// TODO (mereweth@jpl.nasa.gov) - scrub these headers; not all are necessary
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/sysfs.h>

#include <asm/arch_timer.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/irq.h>

#include <linux/time.h>

#define LPASS_QDSP6SS_QTMR_V1_BASE              0xFE2A1000
#define LPASS_QDSP6SS_QTMR_V1_CNTPCT_HI_0       0x00000004
#define LPASS_QDSP6SS_QTMR_V1_CNTPCT_LO_0       0x00000000

// https://github.com/PX4/Firmware/blob/master/src/modules/muorb/krait/px4muorb_KraitRpcWrapper.cpp#L125
// Fraction equivalent to 19.2
#define QDSP6SS_QTMR_MAGIC_NUM   (5)
#define QDSP6SS_QTMR_MAGIC_DENOM (96)

static void __iomem *qdsp6ss_qtmr_base;

static struct kobject *dsp_offset_kobject;

// Function name is used automatically by the __ATTR_RO macro
static ssize_t walltime_dsp_diff_show(struct kobject *kobj, struct kobj_attribute *attr,
                                      char *buf)
{
        DEFINE_SPINLOCK(mr_lock);
        unsigned long flags;

        uint64_t qdsp_qtimer_value = 0;
        uint32_t cvall = 0;
        uint32_t cvalh = 0;
        uint32_t thigh = 0;

        struct timespec linux_ts;

        // The DSP could end up ahead of wall time if walltime is reset, so make this signed
        int64_t walltime_dsp_lead_us = 0;

        if (qdsp6ss_qtmr_base == NULL) {
                pr_err("qdsp6ss_qtmr_base is NULL \n");
                return 0;
        }

        // Enter critical section - lock out interrupts
        spin_lock_irqsave(&mr_lock, flags);

        do {
                cvalh = __raw_readl(qdsp6ss_qtmr_base + LPASS_QDSP6SS_QTMR_V1_CNTPCT_HI_0);
                cvall = __raw_readl(qdsp6ss_qtmr_base + LPASS_QDSP6SS_QTMR_V1_CNTPCT_LO_0);
                thigh = __raw_readl(qdsp6ss_qtmr_base + LPASS_QDSP6SS_QTMR_V1_CNTPCT_HI_0);
        } while (cvalh != thigh);

        // TODO(mereweth@jpl.nasa.gov) - is Wall time what we want?
        getnstimeofday(&linux_ts);

        // Exit critical section - re-enable interrupts
        spin_unlock_irqrestore(&mr_lock, flags);

        qdsp_qtimer_value = ((uint64_t) cvalh << 32) | cvall;

        pr_debug("DSP timer before divide %llu\n", qdsp_qtimer_value);

        /* TODO(mereweth@jpl.nasa.gov) - what to do about possible overflow here?
         * tradeoff seems to be loss of precision by dividing first
         */
        qdsp_qtimer_value *= QDSP6SS_QTMR_MAGIC_NUM;
        do_div(qdsp_qtimer_value, QDSP6SS_QTMR_MAGIC_DENOM); // modifies qdsp_timer_value in place
        //qdsp_qtimer_value /= QDSP6SS_QTMR_MAGIC_NUMBER; // convert to microseconds

        pr_debug("DSP timer after divide %llu\n", qdsp_qtimer_value);
        pr_debug("Walltime: %lld.%.9ld\n", (long long) linux_ts.tv_sec, linux_ts.tv_nsec);

        /* TODO(mereweth@jpl.nasa.gov) - what to do about possible overflow here?
         * should we do computations in floating point seconds instead?
         */
        walltime_dsp_lead_us = ((int64_t) linux_ts.tv_sec) * 1000LL * 1000LL
                               + (int64_t) (linux_ts.tv_nsec / 1000LL)
                               - (int64_t) qdsp_qtimer_value;

        pr_debug("Walltime leads DSP by %lld usec\n", walltime_dsp_lead_us);

        return sprintf(buf, "%lld\n",walltime_dsp_lead_us);
}

static struct kobj_attribute walltime_dsp_diff_attribute = __ATTR_RO(walltime_dsp_diff);

static int __init dspoffset_init(void) {
          int error;
          
          pr_debug("DSP offset module loaded\n");
          qdsp6ss_qtmr_base = ioremap(0xFE2A1000 , 0x32);
          if (qdsp6ss_qtmr_base == NULL) {
                  pr_debug("qdsp timer ioremap fail\n");
                  return -1;
          }

          dsp_offset_kobject = kobject_create_and_add("dsp_offset", kernel_kobj);
          if(!dsp_offset_kobject)
                  return -ENOMEM;

          error = sysfs_create_file(dsp_offset_kobject, &walltime_dsp_diff_attribute.attr);
          if (error) {
                  pr_debug("failed to create /sys/kernel/dsp_offset/walltime_dsp_diff \n");
          }

          return 0;
}
static void __exit dspoffset_fini(void) {
          kobject_put(dsp_offset_kobject);
          pr_debug("DSP offset module unloaded\n");
}
module_init(dspoffset_init);
module_exit(dspoffset_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gene Merewether");
MODULE_DESCRIPTION("DSP to Linux clock offset");
