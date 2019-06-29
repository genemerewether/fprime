#include "Tester.hpp"

TEST(Nominal, SendPulseRecieveTimeTest) {
    Svc::Tester tester;
    tester.SendPulseRecieveTimeTest();
}

TEST(Nominal, RecieveTimeNoPulseSentTest) {
    Svc::Tester tester;
    tester.RecieveTimeNoPulseSentTest();
}

TEST(Nominal, SendPulseNoRecieveTimeTest) {
    Svc::Tester tester;
    tester.SendPulseNoRecieveTimeTest();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}