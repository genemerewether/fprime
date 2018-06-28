/*
 * AllocatorTester.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: tcanham
 */

#include <SnapdragonFlight/DspRpcAllocator/DspRpcAllocator.hpp>
#include <cstdio>

void NominalTest(void) {

    SnapdragonFlight::DspRpcAllocator allocator;
    void *ptr = allocator.allocate(10,100);
    if ((void*)0 == ptr) {
        printf("NULL pointer!\n");
        return;
    }
    printf("Pointer value: %p\n",ptr);
    allocator.deallocate(10,ptr);

}

int main(int argc, char* argv[]) {
    NominalTest();
}
