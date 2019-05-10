// ======================================================================
// \title  i2c_relay.c
// \author tcanham
// \brief  spi relay for dsp_relay
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
//
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ======================================================================

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdbool.h>
#include <dev_fs_lib_i2c.h>
#include <errno.h>

#include <dspal_platform.h>
#include <dspal/test/include/dspal_log.h>

int dsp_relay_i2c_relay_open(int device) {
    /* Request 100% of max clock speed, 100% of max bus speed, and max of 1us
     * hardware wakeup latency
     * See:
     * /opt/tools/leo/Qualcomm/Hexagon_SDK/3.0/incs/HAP_power.h
     * https://github.com/PX4/Firmware/blob/master/src/modules/muorb/adsp/px4muorb.cpp#L58
     */
    HAP_power_request(100, 100, 1);

    int fd = -1;

    // Open:
    char devName[256];
#ifdef DSPAL_sdsp
    snprintf(devName,sizeof(devName), DEV_FS_I2C_SSC_DEVICE_TYPE_STRING "%d",device);
#else
    snprintf(devName,sizeof(devName), DEV_FS_I2C_DEVICE_TYPE_STRING "%d",device);
#endif
    // null terminate
    devName[sizeof(devName)-1] = 0;
    LOG_INFO("Opening I2C device %s\n",devName);

    fd = open(devName, O_RDWR);
    if (fd == -1) {
	LOG_ERR("open I2C device %d failed. %d\n",device,errno);
    } else {
	LOG_INFO("Successfully opened I2C device %s fd %d\n",devName,fd);
    }

    return fd;
}

int dsp_relay_i2c_relay_read_write(int fd, const unsigned char* write_data, int write_dataLen, unsigned char* read_data, int read_dataLen) {

        int result = 0;
	struct dspal_i2c_ioctl_combined_write_read read_write;
	
	read_write.read_buf = (void*) read_data;
        read_write.read_buf_len = read_dataLen;
        read_write.write_buf = (void*) write_data;
        read_write.write_buf_len = write_dataLen;

	if (((NULL == read_write.read_buf)  ||
             (0 == read_write.read_buf_len))   &&
            ((NULL == read_write.write_buf) ||
             (0 == read_write.write_buf_len))) {
            // nothing to read or write
  	    return -2;
        }

        if ((NULL == read_write.read_buf) ||
            (0 == read_write.read_buf_len)) {
            // I2C write only
            ssize_t writeBytes = write(fd,
                                       read_write.write_buf,
                                       read_write.write_buf_len);
            if (writeBytes != read_write.write_buf_len) {
                LOG_ERR("Tried to write %d bytes; wrote %d",
                            read_write.write_buf_len,
                            writeBytes);
                return -3;
            }
        }
        else if ((NULL == read_write.write_buf) ||
                 (0 == read_write.write_buf_len)) {
            // I2C read only
            for (unsigned int byte = 0; byte < read_write.read_buf_len; byte++) {
                read_write.read_buf[byte] = 0xA5;
            }
            ssize_t readBytes = read(fd,
                                     read_write.read_buf,
                                     read_write.read_buf_len);
            if (readBytes != read_write.read_buf_len) {
                LOG_ERR("Tried to write %d bytes; wrote %d",
                            read_write.read_buf_len,
                            readBytes);
                return -4;
            }
        }
        else {
            // Combined read-write
            LOG_INFO("Write %d bytes, read %d bytes to I2C",
                        read_write.write_buf_len,
                        read_write.read_buf_len);
            for (unsigned int byte = 0; byte < read_write.read_buf_len; byte++) {
                read_write.read_buf[byte] = 0xA5;
            }
            result = ioctl(fd, I2C_IOCTL_RDWR, &read_write);
            if (result != read_write.read_buf_len) {
                LOG_ERR("I2C %d read/write error %d vs %d actual! %d: %s",
                            fd, read_write.read_buf_len, result,
                            errno,strerror(errno));
                return errno;
            }
        }

	return 0;
}

int dsp_relay_i2c_relay_configure(int fd, int busSpeed, int slaveAddr, int timeout) {
    struct dspal_i2c_ioctl_slave_config slaveConfig = {
	.bus_frequency_in_khz = busSpeed,
	.slave_address = slaveAddr,
	.byte_transer_timeout_in_usecs = timeout
    };

    // configure I2C clock rate
    if (ioctl(fd, I2C_IOCTL_SLAVE, (void *)&slaveConfig) != 0) {
	LOG_ERR("ioctl I2C SLAVE fd %d failed. %d: %s",fd,errno,strerror(errno));
	return errno;
    } else {
      LOG_INFO("I2C fd %d freq %u, slave 0x%0x",fd, busSpeed, slaveAddr);
    }
    
    return 0;
}

int dsp_relay_i2c_relay_close(int fd) {
    LOG_INFO("Closing I2C device %d",fd);
    return close(fd);
}
