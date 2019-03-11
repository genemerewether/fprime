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

int main(int argc, char **argv) {

    SnapdragonFlight::Tester tester;

    U8 buffer[argc-1];

    // scan args for bytes

    for (NATIVE_INT_TYPE byte = 0; byte < argc-1; byte++) {
        buffer[byte] = strtol(argv[1+byte],0,0);
    }

    tester.sendBuffer(buffer,sizeof(buffer));
}
