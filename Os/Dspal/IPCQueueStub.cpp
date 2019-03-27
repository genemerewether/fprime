#include <Fw/Types/Assert.hpp>
#include <Os/Queue.hpp>
#include <Os/IPCQueue.hpp>

#ifdef TGT_OS_TYPE_VXWORKS
    #include <vxWorks.h>
#endif

#ifdef TGT_OS_TYPE_LINUX
    #include <sys/types.h>
    #include <unistd.h>
#endif

#include <mqueue.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h>

#define IPC_QUEUE_TIMEOUT_SEC (1)

namespace Os {

    IPCQueue::IPCQueue() : Queue() {
    }

    Queue::QueueStatus IPCQueue::create(const Fw::StringBase &name, NATIVE_INT_TYPE depth, NATIVE_INT_TYPE msgSize) {
        return QUEUE_UNINITIALIZED;
    }

    IPCQueue::~IPCQueue() {
    }

    Queue::QueueStatus IPCQueue::send(const U8* buffer, NATIVE_INT_TYPE size, NATIVE_INT_TYPE priority, QueueBlocking block) {
        return QUEUE_UNINITIALIZED;
    }

    Queue::QueueStatus IPCQueue::receive(U8* buffer, NATIVE_INT_TYPE capacity, NATIVE_INT_TYPE &actualSize, NATIVE_INT_TYPE &priority, QueueBlocking block) {
        return QUEUE_UNINITIALIZED;
    }

    NATIVE_INT_TYPE IPCQueue::getNumMsgs(void) const {
        return 0;
    }

    NATIVE_INT_TYPE IPCQueue::getMaxMsgs(void) const {
        return 0;
    }

    NATIVE_INT_TYPE IPCQueue::getQueueSize(void) const {
        return 0;
    }

    NATIVE_INT_TYPE IPCQueue::getMsgSize(void) const {
        return 0;
    }

}
