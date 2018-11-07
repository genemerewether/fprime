

#include <Tester.hpp>

extern "C" {
    void fsw_main(void);
}

void fsw_main(void)
{
printf("SpiMaster test\n");
    R5::Tester tester;
    tester.toDo();
}

#ifndef BUILD_TIR5
int main(int argc, char* argv[]) {
    fsw_main();
}

#endif
