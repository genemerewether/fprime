/*
* \author Tim Canham
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
*
*   Information included herein is controlled under the International
*   Traffic in Arms Regulations ("ITAR") by the U.S. Department of State.
*   Export or transfer of this information to a Foreign Person or foreign
*   entity requires an export license issued by the U.S. State Department
*   or an ITAR exemption prior to the export or transfer.
*/

#include <Drv/Altimeter/LIDARLiteV3/test/ut/Tester.hpp>
#include <Drv/Altimeter/LIDARLiteV3/LIDARLiteV3Impl.hpp>
#include <Drv/Altimeter/LIDARLiteV3/LIDARLiteV3ImplCfg.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <gtest/gtest.h>

#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

TEST(Nominal,Nominal) {
    Drv::Tester tester;
    tester.nominal_test();
}

TEST(OffNominal,StatusErr) {
    Drv::Tester tester;
    tester.status_err_test();
}

TEST(OffNominal,Nack) {
    Drv::Tester tester;
    tester.nack_test();
}

TEST(OffNominal,EarlyRg) {
    Drv::Tester tester;
    tester.early_rg_test();
}

int main(int argc, char* argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


