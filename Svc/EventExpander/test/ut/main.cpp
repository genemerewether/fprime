/*
 * ImuMgrTester.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: Gorang Gandhi
 */

#include <Svc/EventExpander/test/ut/Tester.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>

TEST(TestNominal,Nominal) {

    TEST_CASE(100.1.1,"Test event conversion");
    Svc::Tester tester;
    tester.nominal();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}
