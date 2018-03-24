#include <SnapdragonFlight/HexRouter/test/ut/Tester.hpp>
#include <SnapdragonFlight/HexRouter/HexRouterComponentImpl.hpp>
#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImplCfg.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>
#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#include <string.h>

#ifdef BUILD_SDFLIGHT
    #include <ut_hexrtr.h>
#endif

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

#ifdef BUILD_SDFLIGHT
int rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return ut_hexrtr_rpc_relay_buff_read(port, buff, buffLen, bytes);
}

int rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return ut_hexrtr_rpc_relay_port_read(port, buff, buffLen, bytes);
}

int rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return ut_hexrtr_rpc_relay_write(port, buff, buffLen);
}
#else // BUILD_SDFLIGHT
int rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    // TODO(mereweth) - return something useful for testing
    return 10;
}

int rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    // TODO(mereweth) - return something useful for testing
    return 10;
}

int rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen) {
    DEBUG_PRINT("write called on port %d, size %d\n",
                port, buffLen);
    if (buffLen > FW_NUM_ARRAY_ELEMENTS(SnapdragonFlight::gBuff)) {
        return -3; // TODO(mereweth) - error codes from KraitRouter
    }
    memcpy(SnapdragonFlight::gBuff, buff, buffLen);
    SnapdragonFlight::gLen = buffLen;
    SnapdragonFlight::gPortNum = port;

    return 0;
}
#endif // BUILD_SDFLIGHT

TEST(PortReadWrite,Nominal) {

    TEST_CASE(1, "Read and write single port");

    SnapdragonFlight::Tester tester;

    tester.run_port_read_write_test();
}

// TEST(,Nominal) {
//
//     TEST_CASE(2, "");
//
//     SnapdragonFlight::Tester tester;
//
//     tester.run__test();
// }

#include <signal.h>
#include <stdio.h>

extern "C" {
    int main(int argc, char* argv[]);
};

void print_usage() {
    (void) printf("Pass -k to kill hung unit test\n");
}

#ifdef BUILD_SDFLIGHT
static void sighandler(int signum) {
    // TODO(mereweth) - is there something that can help with SIGKILL?
    DEBUG_PRINT("Signal %d caught - exiting\n", signum);
    ut_hexrtr_fini();
}
#endif //BUILD_SDFLIGHT

int main(int argc, char* argv[]) {
    I32 option = 0;
    while ((option = getopt(argc, argv, "hk")) != -1){
        switch(option) {
            case 'h':
                print_usage();
                return 0;
                break;
            case 'k':
#ifdef BUILD_SDFLIGHT
                ut_hexrtr_fini();
#endif //BUILD_SDFLIGHT
                return 0;
                break;
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

#ifdef BUILD_SDFLIGHT
    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);
    signal(SIGABRT,sighandler);
#endif //BUILD_SDFLIGHT

    memset(SnapdragonFlight::gBuff, 0, FW_NUM_ARRAY_ELEMENTS(SnapdragonFlight::gBuff));
    SnapdragonFlight::gLen = 0;
    SnapdragonFlight::gPortNum = 0;

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
