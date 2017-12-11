#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>
#include <HEXREF/Rpc/hexref.h>

/*#ifdef __cplusplus
extern "C" {
#endif*/

  int rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    //return 1;
    return hexref_rpc_relay_buff_read(port, buff, buffLen, bytes);
  }

  int rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    //return 1;
    return hexref_rpc_relay_port_read(port, buff, buffLen, bytes);
  }
  
  int rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen) {
    //return 1;
    return hexref_rpc_relay_write(port, buff, buffLen);
  }
  
/*#ifdef __cplusplus
} // extern "C"
#endif*/
