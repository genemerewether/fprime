#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>
#include <Os/Task.hpp>

#include <HAP_farf.h>
#include <ut_hexrtr.h>

#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
//#define DEBUG_PRINT(x,...)

SnapdragonFlight::KraitRouterComponentImpl kraitRouter
#if FW_OBJECT_NAMES == 1
                    ("KRAITRTR")
#endif
;

#ifdef __cplusplus
extern "C" {
#endif

int ut_hexrtr_run() {
    DEBUG_PRINT("ut_hexrtr_run");
    Os::Task::delay(1000);
    kraitRouter.init(0);
    kraitRouter.set_KraitPortsOut_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(0));

    // invoke Sched port

    kraitRouter.m_quit = true;
    DEBUG_PRINT("ut_hexrtr_run done");
    return 0;
}

int ut_hexrtr_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter.buffRead(port, buff, buffLen, bytes);
}

int ut_hexrtr_rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter.portRead(port, buff, buffLen, bytes);
}

int ut_hexrtr_rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return kraitRouter.write(port, buff, buffLen);
}

#ifdef __cplusplus
} // extern "C"
#endif
