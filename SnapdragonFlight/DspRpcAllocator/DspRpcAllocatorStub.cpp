/*
 * DspRpcAllocatorStub.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Gorang Gandhi
 */

#include <SnapdragonFlight/DspRpcAllocator/DspRpcAllocator.hpp>
#include <stdlib.h>
#include <Fw/Types/Assert.hpp>

namespace SnapdragonFlight {

    bool DspRpcAllocator::s_initialized = false;

    DspRpcAllocator::DspRpcAllocator() {
        if (not DspRpcAllocator::s_initialized) {
            DspRpcAllocator::s_initialized = true;
        }
    }

    DspRpcAllocator::~DspRpcAllocator() {
    }

    void *DspRpcAllocator::allocate(NATIVE_UINT_TYPE identifier, NATIVE_UINT_TYPE size) {
        FW_ASSERT(DspRpcAllocator::s_initialized);
        return malloc(size);
    }

    void DspRpcAllocator::deallocate(NATIVE_UINT_TYPE identifier, void* ptr) {
        FW_ASSERT(DspRpcAllocator::s_initialized);
        free(ptr);
    }

}
