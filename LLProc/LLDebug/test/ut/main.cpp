/*
 *  Created on: Jan 4, 2018
 *      Author: Tim Canham
 *
 */

#include "Tester.hpp"
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>

TEST(TestNominal,DebugString) {

    TEST_CASE(100.1,"Test Debug String");
    LLProc::Tester tester;
    tester.debugTest();

}

TEST(TestNominal,AssertTest) {

    LLProc::Tester tester;
    tester.assertTest();

}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}
