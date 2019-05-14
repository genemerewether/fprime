#ifndef _DSP_RELAY_H
#define _DSP_RELAY_H
#include "AEEStdDef.h"
#ifndef __QAIC_HEADER
#define __QAIC_HEADER(ff) ff
#endif //__QAIC_HEADER

#ifndef __QAIC_HEADER_EXPORT
#define __QAIC_HEADER_EXPORT
#endif // __QAIC_HEADER_EXPORT

#ifndef __QAIC_HEADER_ATTRIBUTE
#define __QAIC_HEADER_ATTRIBUTE
#endif // __QAIC_HEADER_ATTRIBUTE

#ifndef __QAIC_IMPL
#define __QAIC_IMPL(ff) ff
#endif //__QAIC_IMPL

#ifndef __QAIC_IMPL_EXPORT
#define __QAIC_IMPL_EXPORT
#endif // __QAIC_IMPL_EXPORT

#ifndef __QAIC_IMPL_ATTRIBUTE
#define __QAIC_IMPL_ATTRIBUTE
#endif // __QAIC_IMPL_ATTRIBUTE
#ifdef __cplusplus
extern "C" {
#endif
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_test)(void) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_open)(int gpio) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_close)(int fd) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_configure)(int fd, int type) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_write)(int fd, int value) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_read)(int fd) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_start_int)(int gpio, int fd) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_isr_wait)(int gpio, int fd) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_gpio_relay_quit)(int gpio, int fd) __QAIC_HEADER_ATTRIBUTE;
typedef struct _dsp_relay_dataBuffer__seq_octet _dsp_relay_dataBuffer__seq_octet;
typedef _dsp_relay_dataBuffer__seq_octet dsp_relay_dataBuffer;
struct _dsp_relay_dataBuffer__seq_octet {
   unsigned char* data;
   int dataLen;
};
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_uart_relay_open)(int device) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_uart_relay_configure)(int fd, int device, int baud, int parity, int bits, int stop_bits, int flow_control, int block) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_uart_relay_read)(int device, unsigned char* buff, int buffLen, int* bytes) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_uart_relay_write)(int fd, int device, const unsigned char* buff, int buffLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_uart_relay_close)(int fd, int device) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_uart_relay_quit)(int device) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_uart_receive_allocate)(int device, int size) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_spi_relay_open)(int device) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_spi_relay_configure)(int fd, int clock) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_spi_relay_read_write)(int fd, const unsigned char* write_data, int write_dataLen, unsigned char* read_data, int read_dataLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_spi_relay_close)(int fd) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_i2c_relay_open)(int device) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_i2c_relay_configure)(int fd, int busSpeed, int slaveAddr, int timeout) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_i2c_relay_read_write)(int fd, const unsigned char* write_data, int write_dataLen, unsigned char* read_data, int read_dataLen) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_i2c_relay_close)(int fd) __QAIC_HEADER_ATTRIBUTE;
typedef struct _dsp_relay_configBuffer__seq_unsignedLong _dsp_relay_configBuffer__seq_unsignedLong;
typedef _dsp_relay_configBuffer__seq_unsignedLong dsp_relay_configBuffer;
struct _dsp_relay_configBuffer__seq_unsignedLong {
   unsigned int* data;
   int dataLen;
};
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_pwm_relay_open)(int device) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_pwm_relay_configure)(int fd, uint64* handle, const unsigned int* channel, int channelLen, unsigned int period_in_usecs) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_pwm_relay_set_duty)(uint64 handle, const unsigned int* pulse_width_in_usecs, int pulse_width_in_usecsLen, unsigned int bitmask) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(dsp_relay_pwm_relay_close)(int fd) __QAIC_HEADER_ATTRIBUTE;
#ifdef __cplusplus
}
#endif
#endif //_DSP_RELAY_H
