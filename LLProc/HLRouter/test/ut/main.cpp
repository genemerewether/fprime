// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Tester.hpp"

TEST(Nominal, NominalOneFrame) {
    LLProc::Tester tester;
    tester.nominalOneFrame();
}

TEST(Nominal, NominalTwoFrames) {
    LLProc::Tester tester;
    tester.nominalTwoFrames();
}

TEST(Nominal, NominalOneSplit) {
    LLProc::Tester tester;
    tester.nominalOneSplit();
}

TEST(Nominal, NominalOneSplitTwice) {
    LLProc::Tester tester;
    tester.nominalOneSplitTwice();
}

TEST(Nominal, NominalTwoFramesOneBuffer) {
    LLProc::Tester tester;
    tester.nominalTwoFramesOneBuffer();
}

TEST(Nominal, NominalSyncOnly) {
    LLProc::Tester tester;
    tester.nominalSyncOnly();
}

TEST(Nominal, NominalSyncPlusSize) {
    LLProc::Tester tester;
    tester.nominalSyncPlusSize();
}

TEST(Nominal, NominalSendPackets) {
    LLProc::Tester tester;
    tester.nominalSendPackets();
}

TEST(OffNominal, OffNomResyncOnePacket) {
    LLProc::Tester tester;
    tester.offNomResyncOnePacket();
}

TEST(OffNominal, OffNomPktResyncPkt) {
    LLProc::Tester tester;
    tester.offNomPktResyncPkt();
}

TEST(OffNominal, OffNomDecodeError) {
    LLProc::Tester tester;
    tester.offNomDecodeError();
}

TEST(OffNominal, OffNomSerialError) {
    LLProc::Tester tester;
    tester.offNomSerialError();
}

TEST(OffNominal, OffNomCrcError) {
    LLProc::Tester tester;
    tester.offNomCrcError();
}

TEST(OffNominal, offNomOutpuBufferOverflow) {
    LLProc::Tester tester;
    tester.offNomOutpuBufferOverflow();
}

TEST(OffNominal, offNomZeroPacketSize) {
    LLProc::Tester tester;
    tester.offNomZeroPacketSize();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
