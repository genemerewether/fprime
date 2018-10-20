/*
 * R5DmaAllocator.hpp
 *
 *  Created on: Mar 10, 2017
 *      Author: tcanham
 */

#ifndef R5MEM_R5DMAALLOCATOR_HPP_
#define R5MEM_R5DMAALLOCATOR_HPP_

#include <Fw/Types/MemAllocator.hpp>

class R5DmaAllocator: public Fw::MemAllocator {
    public:
        R5DmaAllocator();
        virtual ~R5DmaAllocator();
        void *allocate(NATIVE_UINT_TYPE identifier, NATIVE_UINT_TYPE size);
        void deallocate(NATIVE_UINT_TYPE identifier, void* ptr);

    PRIVATE:
        U8* r5MemSharedRamPtr;
};

#endif /* R5MEM_R5DMAALLOCATOR_HPP_ */
