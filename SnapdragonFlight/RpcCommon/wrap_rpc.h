#ifdef __cplusplus
extern "C" {
#endif

  // low priority, read raw data
  int rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes);

  // serialized ports, high priority
  int rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes);

  // only serialized ports to Hexagon
  int rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen);
  
#ifdef __cplusplus
} // extern "C"
#endif
