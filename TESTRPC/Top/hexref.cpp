#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>
#include <unistd.h>

volatile bool terminate = false;

int hexref_init() {
  FARF(ALWAYS, "hexref_init");
  return 0;
}

int hexref_run() {
  FARF(ALWAYS, "hexref_run");
  while (!terminate) {
    FARF(ALWAYS, "hexref_run loop");
    usleep(10000);
  }
  return 0;
}

int hexref_fini() {
  FARF(ALWAYS, "hexref_fini");
  terminate = true;
  return 0;
}

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
  return 0;
}

int hexref_rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
  return 0;
}

int hexref_rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen) {
  return 0;
}
