#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>

#ifdef __cplusplus
extern "C" {
#endif

  int rpc_relay_buff_allocate(int size) {
    return 0;
  }
  int rpc_relay_buff_read(int* port, unsigned char* buff, int buffLen, int* bytes) {
    *port = -1;
    *bytes = 0;
    return 0;
  }

  int rpc_relay_port_allocate(int size) {
    return 0;
  }
  int rpc_relay_port_read(int* port, unsigned char* buff, int buffLen, int* bytes) {
    *port = -1;
    *bytes = 0;
    return 0;
  }
  
  int rpc_relay_write(int port, const unsigned char* buff, int buffLen) {
    return 0;
  }
  
#ifdef __cplusplus
} // extern "C"
#endif
