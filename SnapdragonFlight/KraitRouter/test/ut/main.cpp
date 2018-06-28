#include <SnapdragonFlight/KraitRouter/test/ut/Tester.hpp>
#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>

#ifdef BUILD_DSPAL
#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>
#endif

#ifdef BUILD_DSPAL
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

//#undef DEBUG_PRINT
//#define DEBUG_PRINT(x,...)

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

int ut_run(void) {
    int argc = 1;
    char* argv[] = {"test"};
    // TODO(mereweth) - fix defines for DSPAL in gtest/include/gtest/internal/gtest-port.h
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}

#ifndef BUILD_DSPAL

#include <signal.h>
#include <stdio.h>

extern "C" {
    int main(int argc, char* argv[]);
};

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#endif //ifndef BUILD_DSPAL
