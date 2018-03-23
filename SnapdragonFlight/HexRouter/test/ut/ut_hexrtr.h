#ifndef _UT_HEXRTR_H
#define _UT_HEXRTR_H
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
__QAIC_HEADER_EXPORT int __QAIC_HEADER(ut_hexrtr_run)(void) __QAIC_HEADER_ATTRIBUTE;
typedef struct _ut_hexrtr_dataBuffer__seq_octet _ut_hexrtr_dataBuffer__seq_octet;
typedef _ut_hexrtr_dataBuffer__seq_octet ut_hexrtr_dataBuffer;
struct _ut_hexrtr_dataBuffer__seq_octet {
   unsigned char* data;
   int dataLen;
};
__QAIC_HEADER_EXPORT int __QAIC_HEADER(ut_hexrtr_rpc_relay_port_read)(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(ut_hexrtr_rpc_relay_buff_read)(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) __QAIC_HEADER_ATTRIBUTE;
__QAIC_HEADER_EXPORT int __QAIC_HEADER(ut_hexrtr_rpc_relay_write)(unsigned int port, const unsigned char* buff, int buffLen) __QAIC_HEADER_ATTRIBUTE;
#ifdef __cplusplus
}
#endif
#endif //_UT_HEXRTR_H
