// ----------------------------------------------------------------------
// Main.cpp
// ----------------------------------------------------------------------

#include "Tester.hpp"
#include <stdlib.h>
#include <SnapdragonFlight/DspRpcAllocator/DspRpcAllocator.hpp>

//TEST(Test, NominalTlm) {
//  Svc::Tester tester;
//  tester.nominalTlm();
//}

void usage(char* prog) {
    printf("Usage: %s <i2c idx> <slave addr> <bytes to read>\n",prog);
}

int main(int argc, char **argv) {

    if (argc != 4) {
        usage(argv[0]);
        return -1;
    }

    int i2c = atoi(argv[1]);
    int slave = atoi(argv[2]);
    int readSize = atoi(argv[3]);

    SnapdragonFlight::Tester tester;

    printf("Testing I2C %d WriteRead to slave %d\n", i2c, slave);
    tester.openAndConfig(i2c, slave, 400000);
    U8 buffer[2] = { 0, 0 };
    tester.testWriteRead(buffer, sizeof(buffer), readSize);
}
