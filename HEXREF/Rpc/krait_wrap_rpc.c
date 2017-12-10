#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>
#include <HEXREF/Rpc/hexref.h>

#ifdef __cplusplus
extern "C" {
#endif

  int rpc_relay_buff_allocate(int size) {
    return hexref_rpc_relay_buff_allocate(size);
  }
  int rpc_relay_buff_read(int* port, unsigned char* buff, int buffLen, int* bytes) {
    return hexref_rpc_relay_buff_read(port, buff, buffLen, bytes);
  }

  int rpc_relay_port_allocate(int size) {
    return hexref_rpc_relay_port_allocate(size);
  }
  int rpc_relay_port_read(int* port, unsigned char* buff, int buffLen, int* bytes) {
    return hexref_rpc_relay_port_read(port, buff, buffLen, bytes);
  }
  
  int rpc_relay_write(int port, const unsigned char* buff, int buffLen) {
    return hexref_rpc_relay_write(port, buff, buffLen);
  }
  
#ifdef __cplusplus
} // extern "C"
#endif
