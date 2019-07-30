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
    printf("Usage: %s <pwmdev> <gpio> \n",prog);
}

int main(int argc, char **argv) {

    if (argc != 3) {
        usage(argv[0]);
        return -1;
    }

    int pwm = atoi(argv[1]);
    int gpio = atoi(argv[2]);

    SnapdragonFlight::Tester tester;

    tester.testPwm(pwm, gpio, 10, 0.15);
}
