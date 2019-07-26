// ----------------------------------------------------------------------
// Main.cpp 
// ----------------------------------------------------------------------

#include "Tester.hpp"
#include <Fw/Test/UnitTest.hpp>

TEST(Nominal, Integration) {
    Svc::Tester tester;
    tester.integratedTest();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
