/*==============================================================================
  Copyright (c) 2012-2013 Qualcomm Technologies, Inc.
  All rights reserved. Qualcomm Proprietary and Confidential.
==============================================================================*/

#include "rpcmem.h"

#include <stdlib.h>
#include <stdio.h>

void rpcmem_init()
{
}

void rpcmem_deinit()
{
}

int rpcmem_to_fd(void* po) {
   return (int)(uintptr_t)po;
}

void* rpcmem_alloc(int heapid, uint32 flags, int size)
{
   (void)heapid;
   (void)flags;

   return malloc(size);
}

void rpcmem_free(void* po)
{
   free(po);
}
