/*
 * SnapdragonHealthTester.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: Gorang Gandhi
 */

#include <SnapdragonFlight/SnapdragonHealth/test/ut/Tester.hpp>
#include <SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>


TEST(TestNominal,Nominal) {


    TEST_CASE(1,"Nominal Testing");

    SnapdragonFlight::Tester tester;

    tester.run_nominal_tests();

}

TEST(TestOffNominal,OffNominal) {


    TEST_CASE(1,"Off Nominal Testing");

    SnapdragonFlight::Tester tester;

    tester.run_off_nominal_tests();

}

#ifndef TGT_OS_TYPE_VXWORKS
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}

#endif
