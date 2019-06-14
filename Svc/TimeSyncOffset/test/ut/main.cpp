#include "Tester.hpp"

TEST(Nominal, AddOperationTest) {
    Svc::Tester tester;
    tester.timeSyncTest();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}