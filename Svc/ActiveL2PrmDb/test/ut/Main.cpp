// ----------------------------------------------------------------------
// Main.cpp 
// ----------------------------------------------------------------------

#include "Tester.hpp"
#include <Fw/Test/UnitTest.hpp>

TEST(Nominal, RequestPrms) {
    Svc::Tester tester;
    tester.requestPrmsTest();
}

TEST(Nominal, SetGetPrms) {
    Svc::Tester tester;
    tester.setGetTest();
}

TEST(Nominal, SetSendPrms) {
    Svc::Tester tester;
    tester.setSendTest();
}

TEST(Error, SendTooLarge) {
    Svc::Tester tester;
    tester.sendTooLargeTest();
}

TEST(Error, SendBuffOverflow) {
    Svc::Tester tester;
    tester.sendBuffOvfTest();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
