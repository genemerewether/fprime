// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Tester.hpp"
#include "Errors.hpp"
#include "Logging.hpp"
#include "Health.hpp"

TEST(Test, LogNoInit) {
    Svc::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE,
                       false); // don't call initLog for the user
    testerReg.LogNoInit();

    Svc::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE,
                       false); // don't call initLog for the user
    testerLoop.LogNoInit();

    Svc::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE,
                       false); // don't call initLog for the user
    testerBulk.LogNoInit();

    Svc::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE,
                             false); // don't call initLog for the user
    testerDirect.LogNoInit();
}

// ----------------------------------------------------------------------
// Test Errors
// ----------------------------------------------------------------------

TEST(TestErrors, LogFileOpen) {
    Svc::Errors::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.LogFileOpen();

    Svc::Errors::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.LogFileOpen();

    Svc::Errors::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.LogFileOpen();

/*#if 1 // TODO(mereweth) - byte-align the buffer for writing //def BUILD_SDFLIGHT
    Svc::Errors::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.LogFileOpen();
#endif // BUILD_SDFLIGHT
*/
}

TEST(TestErrors, LogFileWrite) {
    Svc::Errors::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.LogFileWrite();

    Svc::Errors::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.LogFileWrite();

    Svc::Errors::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.LogFileWrite();

/*#if 1 // TODO(mereweth) - byte-align the buffer for writing //def BUILD_SDFLIGHT
    Svc::Errors::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.LogFileWrite();
#endif // BUILD_SDFLIGHT
*/
}

TEST(TestErrors, LogFileValidation) {
    Svc::Errors::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.LogFileValidation();

    Svc::Errors::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.LogFileValidation();

    Svc::Errors::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.LogFileValidation();

    Svc::Errors::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.LogFileValidation();
}

// ----------------------------------------------------------------------
// Test Logging
// ----------------------------------------------------------------------

TEST(TestLogging, BufferSendIn) {
    Svc::Logging::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.BufferSendIn();

    Svc::Logging::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.BufferSendIn();

    Svc::Logging::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.BufferSendIn();

    Svc::Logging::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.BufferSendIn();
}

TEST(TestLogging, CloseFile) {
    Svc::Logging::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.CloseFile();

    Svc::Logging::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.CloseFile();

    Svc::Logging::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.CloseFile();

    Svc::Logging::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.CloseFile();
}

TEST(TestLogging, ComIn) {
    Svc::Logging::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.ComIn();

    Svc::Logging::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.ComIn();

    Svc::Logging::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.ComIn();

    Svc::Logging::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.ComIn();
}

TEST(TestLogging, OnOff) {
    Svc::Logging::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.OnOff();

    Svc::Logging::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.OnOff();

    Svc::Logging::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.OnOff();

    Svc::Logging::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.OnOff();
}

// ----------------------------------------------------------------------
// Test Health
// ----------------------------------------------------------------------

TEST(TestHealth, Ping) {
    Svc::Health::Tester testerReg(Svc::BL_REGULAR_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerReg.Ping();

    Svc::Health::Tester testerLoop(Svc::BL_LOOPING_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerLoop.Ping();

    Svc::Health::Tester testerBulk(Svc::BL_BULK_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerBulk.Ping();

    Svc::Health::Tester testerDirect(Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC, DIRECT_CHUNK_SIZE);
    testerDirect.Ping();
}

int main(int argc, char **argv) {
::testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
