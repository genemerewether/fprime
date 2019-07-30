/*
 * PrmDbTester.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: tcanham
 */

#include <Svc/PrmDb/test/ut/PrmDbImplTester.hpp>
#include <Svc/PrmDb/PrmDbImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <gtest/gtest.h>

#include <Fw/Test/UnitTest.hpp>

#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif


TEST(ParameterDbTest,NominalPopulateTest) {

    Svc::PrmDbImpl impl;

    Svc::PrmDbImplTester tester(impl);

    tester.runNominalPopulate();
}

TEST(ParameterDbTest,NominalUpdateTest) {

    Svc::PrmDbImpl impl;

    Svc::PrmDbImplTester tester(impl);

    tester.runNominalUpdate();
}

TEST(ParameterDbTest,NominalSerDesTest) {

    Svc::PrmDbImpl impl;

    Svc::PrmDbImplTester tester(impl);

    tester.runNominalSerDes();
}

TEST(ParameterDbTest,NominalClearTest) {

    Svc::PrmDbImpl impl;

    Svc::PrmDbImplTester tester(impl);

    tester.runNominalClear();
}

TEST(ParameterDbTest,NominalIteratorTest) {

    Svc::PrmDbImpl impl;

    Svc::PrmDbImplTester tester(impl);

    tester.runNominalIterator();
}

TEST(ParameterDbTest,ErrFullPopulateTest) {

    Svc::PrmDbImpl impl;

    Svc::PrmDbImplTester tester(impl);

    tester.runFullPopulate();
}

TEST(ParameterDbTest,ErrInvalidSerializeTest) {

    Svc::PrmDbImpl impl;

    Svc::PrmDbImplTester tester(impl);

    tester.runInvalidSerialize();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}



