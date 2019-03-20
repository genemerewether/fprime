// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Tester.hpp"

TEST(Test, ThreeBufferProblem) {
  Svc::Tester tester;
  tester.three_buffer_problem();
}

TEST(Test, EmptyBuffers) {
  Svc::Tester tester;
  tester.empty_buffers();
}

TEST(Test, TooManyBuffers) {
  Svc::Tester tester;
  tester.too_many_buffers();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
