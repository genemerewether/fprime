// ----------------------------------------------------------------------
// Main.cpp 
// ----------------------------------------------------------------------

#include "Tester.hpp"
#include <Fw/Test/UnitTest.hpp>

TEST(Nominal, SetPrm) {
    Svc::Tester tester;
    tester.nominalSetPrmTest();
}

TEST(Nominal, GetPrm) {
    Svc::Tester tester;
    tester.nominalGetPrmTest();
}

TEST(Nominal, UpdatePrm) {
    Svc::Tester tester;
    tester.nominalUpdatePrmTest();
}

TEST(Nominal, ReadFile) {
    Svc::Tester tester;
    tester.nominalReadFileTest();
}

TEST(Nominal, WriteFile) {
    Svc::Tester tester;
    tester.nominalWriteFileTest();
}

TEST(Nominal, SendPrms) {
    Svc::Tester tester;
    tester.nominalSendPrmsTest();
}

TEST(Nominal, RecvPrms) {
    Svc::Tester tester;
    tester.nominalRecvPrmsTest();
}

TEST(OffNominal, SetPrmsFull) {
    Svc::Tester tester;
    tester.offNominalSetPrmsFullTest();
}

TEST(OffNominal, GetUnknownPrm) {
    Svc::Tester tester;
    tester.offNominalGetUnknownPrmTest();
}

TEST(OffNominal, ReadInvalidFile) {
    Svc::Tester tester;
    tester.offNominalReadInvalidFileTest();
}

TEST(OffNominal, SendPrmsSize) {
    Svc::Tester tester;
    tester.offNominalSendPrmsSizeTest();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
