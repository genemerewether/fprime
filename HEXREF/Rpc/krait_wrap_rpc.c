#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>
#include <HEXREF/Rpc/hexref.h>

/*#ifdef __cplusplus
extern "C" {
#endif*/

  int rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    //return 1;
    return hexref_rpc_relay_buff_read(port, buff, buffLen, bytes);
  }

  int rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
    //return 1;
    return hexref_rpc_relay_port_read(buff, buffLen, bytes);
  }

  int rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    //return 1;
    return hexref_rpc_relay_buff_write(port, buff, buffLen);
  }

  int rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    //return 1;
    return hexref_rpc_relay_port_write(buff, buffLen);
  }

/*#ifdef __cplusplus
} // extern "C"
#endif*/
