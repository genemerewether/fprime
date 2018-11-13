

#include <Tester.hpp>

extern "C" {
    void fsw_main(void);
}

void fsw_main(void)
{
    printf("UarDrv test\n");

#if 0
    {
        printf("Test SCI1\n");
        R5::Tester tester(0);
        tester.toDo();
    }

    {
        printf("Test SCI2\n");
        R5::Tester tester(1);
        tester.toDo();
    }

    {
        printf("Test SCI3\n");
        R5::Tester tester(2);
        tester.toDo();
    }
#else

    {
        printf("Test data flow\n");
        R5::Tester tester(0);
        tester.r5FlowTester();
    }
#endif

    printf("UarDrv test completed\n");
}

#ifndef BUILD_TIR5
int main(int argc, char* argv[]) {
    fsw_main();
}

#endif
