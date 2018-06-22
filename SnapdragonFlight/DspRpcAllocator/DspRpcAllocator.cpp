/*
 * DspRpcAllocator.cpp
 *
 *  Created on: Dec 6, 2016
 *      Author: tcanham
 */

#include <SnapdragonFlight/DspRpcAllocator/DspRpcAllocator.hpp>
#include <rpcmem.h>
#include <Fw/Types/Assert.hpp>

namespace SnapdragonFlight {

    bool DspRpcAllocator::s_initialized = false;

    DspRpcAllocator::DspRpcAllocator() {
        if (not DspRpcAllocator::s_initialized) {
            rpcmem_init();
            DspRpcAllocator::s_initialized = true;
        }
    }

    DspRpcAllocator::~DspRpcAllocator() {
    }

    void *DspRpcAllocator::allocate(NATIVE_UINT_TYPE identifier, NATIVE_UINT_TYPE size) {
        FW_ASSERT(DspRpcAllocator::s_initialized);
        return rpcmem_alloc(RPCMEM_DEFAULT_HEAP, RPCMEM_DEFAULT_FLAGS, size);
    }

    void DspRpcAllocator::deallocate(NATIVE_UINT_TYPE identifier, void* ptr) {
        FW_ASSERT(DspRpcAllocator::s_initialized);
        rpcmem_free(ptr);
    }

}
