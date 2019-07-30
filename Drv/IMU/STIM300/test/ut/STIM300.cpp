/*
* \author Gerik Kubiak
* \file
* \brief
*
* This file is the test driver for the active rate group unit test.
*
* Code Generated Source Code Header
*
*   Copyright 2014-2015, by the California Institute of Technology.
*   ALL RIGHTS RESERVED. United States Government Sponsorship
*   acknowledged. Any commercial use must be negotiated with the Office
*   of Technology Transfer at the California Institute of Technology.
*/

#include <Drv/IMU/STIM300/test/ut/Tester.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <gtest/gtest.h>

#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

TEST(Nominal,Nominal) {
    Drv::Tester tester;
    tester.nominalTest();
}

TEST(Nominal,ManyPackets) {
    Drv::Tester tester;
    tester.manyPackets();
}

TEST(TimeSync,Nominal) {
    Drv::Tester tester;
    tester.manualTimeSyncTest();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


