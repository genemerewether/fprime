
#define PRIVATE  public

#include <R5/R5Mem/R5DmaAllocator.hpp>
#include <R5/R5Mem/R5Mem.hpp>

#include <stdio.h>  // for printf


extern "C" {
    void fsw_main(void);
}

// void rti_test(void);

void fsw_main(void)
{
    printf("R5Mem test\n");

    R5DmaAllocator allocator;

    void* ptr = allocator.allocate(0, 16);
    if(r5MemSharedRam != ptr) {
        printf("!!!! Failed first memory allocation ptr=0x%x expected=0x%x\n", ptr, r5MemSharedRam);
    }
    if((r5MemSharedRam + R5_CACHE_LINE_SIZE) != allocator.r5MemSharedRamPtr) {
        printf("!!!! Failed internal allocation accounting r5MemSharedRamPtr=0x%x expected=0x%x\n", allocator.r5MemSharedRamPtr, (r5MemSharedRam + R5_CACHE_LINE_SIZE));
    }

    ptr = allocator.allocate(0, 255);
    if((r5MemSharedRam + R5_CACHE_LINE_SIZE) != ptr) {
        printf("!!!! Failed memory allocation ptr=0x%x expected=0x%x\n", ptr, (r5MemSharedRam + R5_CACHE_LINE_SIZE));
    }

    ptr = allocator.allocate(0, 513);
    if((r5MemSharedRam + (9 * R5_CACHE_LINE_SIZE)) != ptr) {
        printf("!!!! Failed memory allocation ptr=0x%x expected=0x%x\n", ptr, (r5MemSharedRam + (9 * R5_CACHE_LINE_SIZE)));
    }
    if((r5MemSharedRam + (26 * R5_CACHE_LINE_SIZE)) != allocator.r5MemSharedRamPtr) {
        printf("!!!! Failed internal allocation accounting r5MemSharedRamPtr=0x%x expected=0x%x\n", allocator.r5MemSharedRamPtr, (r5MemSharedRam + (26 * R5_CACHE_LINE_SIZE)));
    }

    ptr = allocator.allocate(0, (256 * 13));
    if(NULL != ptr) {
        printf("!!!! Failed attempt to allocate too much memory ptr=0x%x expected=0x0\n", ptr);
    }
    if((r5MemSharedRam + (26 * R5_CACHE_LINE_SIZE)) != allocator.r5MemSharedRamPtr) {
        printf("!!!! Failed internal allocation accounting r5MemSharedRamPtr=0x%x expected=0x%x\n", allocator.r5MemSharedRamPtr, (r5MemSharedRam + (26 * R5_CACHE_LINE_SIZE)));
    }

    printf("Test completed\n");

// rti_test();

    // The end
    for(;;);
}


// #include "HL_rti.h"
//
// void rti_test(void)
// {
//     printf("RTI test\n");
//
//     printf("get counters\n");
//     RtiTime rtiTime;
//     uint32 medResTimestamp = rtiGetMedResTimestamp();
//     uint32 highResTimestamp = rtiGetHighResTimestamp();
//     rtiGetHighResTime(&rtiTime);
//     printf("MedResTimestamp=%u HighResTimestamp=%u\n", medResTimestamp, highResTimestamp);
//     printf("HighResTime sec=%u subsec=%u\n", rtiTime.seconds, rtiTime.subseconds);
//
//     printf("call rtiSetBothCounters() while they are running GCTRL=0x%x\n", rtiREG1->GCTRL);
//     uint32 ret = rtiSetBothCounters(1, 1);
//     if(0 == ret) {
//         printf("!!!! Failed test rtiSetBothCounters() while they are running\n");
//     }
//
//     printf("stop both counters\n");
//     unsigned sum = 0;
//     rtiStopCounter(rtiREG1, rtiCOUNTER_BLOCK0);
//     rtiStopCounter(rtiREG1, rtiCOUNTER_BLOCK1);
//     highResTimestamp = rtiGetHighResTimestamp();
//     for(unsigned i = 0; i < 1000; ++i) {
//         sum += i;
//     }
//     ret = rtiGetHighResTimestamp();
//     if(ret != highResTimestamp) {
//         printf("!!!! Failed stop counters GCTRL=0x%x\n", rtiREG1->GCTRL);
//         printf("!!!! ret=%u highResTimestamp=%u sum=%u\n", ret, highResTimestamp, sum);
//     }
//
//     printf("call rtiSetBothCounters()\n");
//     ret = rtiSetBothCounters(28, 196);
//     if(0 != ret) {
//         printf("!!!! Failed call to rtiSetBothCounters()\n");
//     }
//     medResTimestamp = rtiGetMedResTimestamp();
//     highResTimestamp = rtiGetHighResTimestamp();
//     rtiGetHighResTime(&rtiTime);
//     printf("MedResTimestamp=%u HighResTimestamp=%u 196*75=%u\n", medResTimestamp, highResTimestamp, (196 * 75));
//     printf("HighResTime sec=%u subsec=%u\n", rtiTime.seconds, rtiTime.subseconds);
//
//     printf("start both counters\n");
//     rtiStartBothCounters();
//     sum = 0;
//     for(unsigned i = 0; i < 10000; ++i) {
//         sum += i;
//     }
//     medResTimestamp = rtiGetMedResTimestamp();
//     highResTimestamp = rtiGetHighResTimestamp();
//     rtiGetHighResTime(&rtiTime);
//     printf("MedResTimestamp=%u HighResTimestamp=%u sum=%u\n", medResTimestamp, highResTimestamp, sum);
//     printf("HighResTime sec=%u subsec=%u\n", rtiTime.seconds, rtiTime.subseconds);
//
//     printf("RTI Test completed\n");
// }

#ifndef BUILD_TIR5
int main(int argc, char* argv[]) {
    fsw_main();
}
#endif
