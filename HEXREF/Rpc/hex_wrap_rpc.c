#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>
#include <HEXREF/Rpc/hexref.h>

#ifdef __cplusplus
extern "C" {
#endif

  int hexref_rpc_relay_buff_allocate(int size) {
    return 0;
    //return rpc_relay_buff_allocate(size);
  }
  int hexref_rpc_relay_buff_read(int* port, unsigned char* buff, int buffLen, int* bytes) {
    return 0;
    //return rpc_relay_buff_read(port, buff, buffLen, bytes);
  }

  int hexref_rpc_relay_port_allocate(int size) {
    return 0;
    //return rpc_relay_port_allocate(size);
  }
  int hexref_rpc_relay_port_read(int* port, unsigned char* buff, int buffLen, int* bytes) {
    return 0;
    //return rpc_relay_port_read(port, buff, buffLen, bytes);
  }
  
  int hexref_rpc_relay_write(int port, const unsigned char* buff, int buffLen) {
    return 0;
    //return rpc_relay_write(port, buff, buffLen);
  }
  
#ifdef __cplusplus
} // extern "C"
#endif
