#include <SnapdragonFlight/HexRouter/test/ut/Tester.hpp>
#include <SnapdragonFlight/HexRouter/HexRouterComponentImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>
#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>

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
    // TODO(mereweth) - return something useful for testing
    return 10;
}
#endif // BUILD_SDFLIGHT

TEST(PortReadWrite,Nominal) {

    TEST_CASE(1, "Read and write single port without sched");

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

int main(int argc, char* argv[]) {

#ifdef BUILD_SDFLIGHT
    ut_hexrtr_run();
#endif // BUILD_SDFLIGHT

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
