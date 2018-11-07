// ======================================================================
// \title  R5Mem.h
// \author Igor Uchenik
// \brief  R5 DMA Memory Allocator
//
// \copyright
// Copyright 2009-2017, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
//
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ======================================================================

#ifndef R5MEM_HPP_
#define R5MEM_HPP_

#include <Fw/Types/BasicTypes.hpp>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

#define R5_MEM_SHARED_RAM_SIZE  4096

#define R5_CACHE_LINE_SIZE  32

extern U8 r5MemSharedRam[R5_MEM_SHARED_RAM_SIZE];

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // R5MEM_HPP_
