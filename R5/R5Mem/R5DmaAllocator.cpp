/*
 * R5DmaAllocator.cpp
 *
 *  Created on: Mar 10, 2017
 *      Author: tcanham
 */

#include <R5/R5Mem/R5DmaAllocator.hpp>
#include <R5/R5Mem/R5Mem.hpp>

// #include <stdio.h>  // for printf

R5DmaAllocator::R5DmaAllocator()
{
    r5MemSharedRamPtr = &r5MemSharedRam[0];
}

R5DmaAllocator::~R5DmaAllocator() {
}

void* R5DmaAllocator::allocate(NATIVE_UINT_TYPE identifier, NATIVE_UINT_TYPE size)
{
    unsigned availSize = &r5MemSharedRam[R5_MEM_SHARED_RAM_SIZE] - r5MemSharedRamPtr;

    // All allocations are done in cache line multiples to ensure buffers chache line alignment
    size += (R5_CACHE_LINE_SIZE - 1);
    size &= (U32)(-R5_CACHE_LINE_SIZE);

    if(size <= availSize) {
        U8* alocAddr = r5MemSharedRamPtr;
        r5MemSharedRamPtr += size;

        return alocAddr;
    }
    else {
        return 0;
    }
}

void R5DmaAllocator::deallocate(NATIVE_UINT_TYPE identifier, void* ptr)
{
}


