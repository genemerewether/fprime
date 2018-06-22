#include "wrap_rpc.h"
#include <time.h>
#include <unistd.h>

// low priority, read raw data
int rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes)
{
  usleep(10000);
  return 0;
}

// serialized ports, high priority
int rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes)
{
  usleep(10000);
  return 0;
}

int rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen)
{
  usleep(10000);
  return 0;
}

int rpc_relay_port_write(unsigned int port, const unsigned char* buff, int buffLen)
{
  usleep(10000);
  return 0;
}
