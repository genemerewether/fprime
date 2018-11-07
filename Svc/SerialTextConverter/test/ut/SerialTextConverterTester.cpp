/*
 * SerialTextConverter.cpp
 *
 *  Created on: May 9, 2017
 *      Author: Gorang Gandhi
 */

#include <Svc/SerialTextConverter/test/ut/Tester.hpp>
#include <Svc/SerialTextConverter/SerialTextConverterImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>


TEST(TestAll,All) {

    Svc::Tester tester;

    tester.run_all_tests();

}

#ifndef TGT_OS_TYPE_VXWORKS
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}

#endif

