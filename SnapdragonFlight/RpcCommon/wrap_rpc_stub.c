#include "wrap_rpc.h"

// low priority, read raw data
int rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes)
{
  while(1);
}

// serialized ports, high priority
int rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes)
{
  while(1);
}

// only serialized ports to Hexagon
int rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen)
{
  while(1);
}
