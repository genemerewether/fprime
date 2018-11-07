// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Tester.hpp"

TEST(Nominal, NominalInit) {
    HLProc::Tester tester;
    tester.nominalInit();
}

TEST(Nominal, NominalOneFrame) {
    HLProc::Tester tester;
    tester.nominalOneFrame();
}

TEST(Nominal, NominalTwoFrames) {
    HLProc::Tester tester;
    tester.nominalTwoFrames();
}

TEST(Nominal, NominalOneSplit) {
    HLProc::Tester tester;
    tester.nominalOneSplit();
}

TEST(Nominal, NominalOneSplitTwice) {
    HLProc::Tester tester;
    tester.nominalOneSplitTwice();
}

TEST(Nominal, NominalTwoFramesOneBuffer) {
    HLProc::Tester tester;
    tester.nominalTwoFramesOneBuffer();
}

TEST(Nominal, NominalSyncOnly) {
    HLProc::Tester tester;
    tester.nominalSyncOnly();
}

TEST(Nominal, NominalSyncPlusSize) {
    HLProc::Tester tester;
    tester.nominalSyncPlusSize();
}

TEST(Nominal, NominalSendPackets) {
    HLProc::Tester tester;
    tester.nominalSendPackets();
}


TEST(OffNominal, OffNomResyncOnePacket) {
    HLProc::Tester tester;
    tester.offNomResyncOnePacket();
}

TEST(OffNominal, OffNomPktResyncPkt) {
    HLProc::Tester tester;
    tester.offNomPktResyncPkt();
}

TEST(OffNominal, OffNomDecodeError) {
    HLProc::Tester tester;
    tester.offNomDecodeError();
}

TEST(OffNominal, OffNomSerialError) {
    HLProc::Tester tester;
    tester.offNomSerialError();
}

TEST(OffNominal, OffNomCrcError) {
    HLProc::Tester tester;
    tester.offNomCrcError();
}

TEST(OffNominal, offNomOutpuBufferOverflow) {
    HLProc::Tester tester;
    tester.offNomOutpuBufferOverflow();
}

TEST(OffNominal, offNomZeroPacketSize) {
    HLProc::Tester tester;
    tester.offNomZeroPacketSize();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
