#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>
#include <unistd.h>

volatile bool terminate = false;
volatile bool preinit = true;

int hexref_init() {
    FARF(ALWAYS, "hexref_init");
    preinit = false;
    usleep(1000 * 1000);
    return 0;
}

int hexref_run() {
    FARF(ALWAYS, "hexref_run");
    if (preinit) {
        FARF(ALWAYS, "hexref_run preinit - returning");
        return -1;
    }

    while (!terminate) {
        FARF(ALWAYS, "hexref_run loop; terminate: %d", terminate);
        usleep(1000);
    }
    return 0;
}

int hexref_cycle(unsigned int cycles) {
    FARF(ALWAYS, "hexref_cycle %d", cycles);
    if (preinit) {
        FARF(ALWAYS, "hexref_cycle preinit - returning");
        return -1;
    }

    for (unsigned int i = 0; i < cycles; i++) {
        FARF(ALWAYS, "hexref_cycle loop %d; terminate: %d", i, terminate);
        usleep(1000);
    }
    return 0;
}

int hexref_wait() {
    FARF(ALWAYS, "hexref_wait");
    while (!terminate) {
        FARF(ALWAYS, "hexref_wait loop; terminate: %d", terminate);
        usleep(1000 * 1000);
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

int hexref_rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
    return 0;
}

int hexref_rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return 0;
}

int hexref_rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    return 0;
}
