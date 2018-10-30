/*
 * ShortLogQueueTester.cpp
 *
 *  Created on: Jan 18, 2017
 *      Author: Gorang Gandhi
 */

#include <LLProc/ShortLogQueue/test/ut/Tester.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>

TEST(TestNominal,NominalOneIn) {

    TEST_CASE(100.1.1,"Test one event in, one event out");
    LLProc::Tester tester;
    tester.doNominalOneIn();

}

TEST(TestNominal,NominalAllIn) {

    TEST_CASE(100.1.2,"Test all events in, all events out");
    LLProc::Tester tester;
    tester.doNominalAllIn();

}

TEST(TestNominal,NominalSomeIn) {

    TEST_CASE(100.1.3,"Test some events in, that many clocked events out");
    LLProc::Tester tester;
    tester.doNominalSomeIn();

}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}
