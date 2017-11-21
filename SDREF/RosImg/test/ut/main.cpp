/*
 * RosImgTester.cpp
 *
 *  Created on: Sept 18, 2017
 *      Author: Gene Merewether
 */

#include <NAVOUTDOOR/RosImg/test/ut/Tester.hpp>
#include <NAVOUTDOOR/RosImg/RosImgComponentImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <gtest/gtest.h>
#include <Fw/Test/UnitTest.hpp>

TEST(SendOnce,Nominal) {

    TEST_CASE(1,"Capture still images");

    Navoutdoor::Tester tester;

    tester.send_once();
}


#ifndef TGT_OS_TYPE_VXWORKS
int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);

    ros::init(argc,argv,"RosImgTester");

    return RUN_ALL_TESTS();

    return 0;
}

#endif
