// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Svc/ImgComp/test/ut/Tester.hpp"

  // ----------------------------------------------------------------------
  // Test nominal operations
  // ----------------------------------------------------------------------

TEST(NominalTest, bufGetAllOk) {
    Svc::Tester tester;
    tester.bufGetAllOk();
}

  // ----------------------------------------------------------------------
  // Test off-nominal buffer get calls
  // ----------------------------------------------------------------------

TEST(OffNominalTest, bufGetNoMeta) {
    Svc::Tester tester;
    tester.bufGetNoMeta();
}

TEST(OffNominalTest, bufGetNoThumb) {
    Svc::Tester tester;
    tester.bufGetNoThumb();
}

TEST(OffNominalTest, bufGetNoImage) {
    Svc::Tester tester;
    tester.bufGetNoImage();
}

TEST(OffNominalTest, bufGetNoMetaNoCount) {
    Svc::Tester tester;
    tester.bufGetNoMetaNoCount();
}

TEST(OffNominalTest, bufGetNoThumbNoCount) {
    Svc::Tester tester;
    tester.bufGetNoThumbNoCount();
}

TEST(OffNominalTest, bufGetNoImageNoCount) {
    Svc::Tester tester;
    tester.bufGetNoImageNoCount();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
