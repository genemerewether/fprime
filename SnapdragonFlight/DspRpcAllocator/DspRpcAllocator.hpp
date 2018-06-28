/*
 * DspRpcAllocator.hpp
 *
 *  Created on: Dec 6, 2016
 *      Author: tcanham
 */

#ifndef DSPRPCALLOCATOR_DSPRPCALLOCATOR_HPP_
#define DSPRPCALLOCATOR_DSPRPCALLOCATOR_HPP_

#include <Fw/Types/MemAllocator.hpp>

namespace SnapdragonFlight {

    class DspRpcAllocator: public Fw::MemAllocator {
        public:
            DspRpcAllocator();
            virtual ~DspRpcAllocator();
            void *allocate(NATIVE_UINT_TYPE identifier, NATIVE_UINT_TYPE size);
            void deallocate(NATIVE_UINT_TYPE identifier, void* ptr);
        PRIVATE:
            static bool s_initialized;
    };

}

#endif /* DSPRPCALLOCATOR_DSPRPCALLOCATOR_HPP_ */
