#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>

int hexref_init() {
    FARF(ALWAYS, "hexref_init");
    return 0;
}

int hexref_arm() {
    FARF(ALWAYS, "hexref_arm");
    return 0;
}

int hexref_run() {
    FARF(ALWAYS, "hexref_run");
    return 0;
}

int hexref_cycle(unsigned int cycles) {
    FARF(ALWAYS, "hexref_cycle");
    return 0;
}

int hexref_wait() {
    FARF(ALWAYS, "hexref_wait");
    return 0;
}

int hexref_fini() {
    FARF(ALWAYS, "hexref_fini");
    return 0;
}

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    bytes = 0;
    return 0;
}

int hexref_rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
    bytes = 0;
    return 0;
}

int hexref_rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return 0;
}

int hexref_rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    return 0;
}
