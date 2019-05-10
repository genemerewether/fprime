#include <Fw/Types/Assert.hpp>
#include <Os/Queue.hpp>

#ifdef TGT_OS_TYPE_LINUX
    #include <sys/types.h>
    #include <unistd.h>
#endif

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <semaphore.h>

namespace Os {

    class QueueHandle {
        public:
        QueueHandle(U8* data, NATIVE_INT_TYPE depth, NATIVE_INT_TYPE msgSize) {
            // Initialize the handle:
            int ret;
            // depth # buffers are empty to begin with
            ret = sem_init(&this->queueEmpty, 0, depth);
            FW_ASSERT(ret == 0, ret); // If this fails, something horrible happened.
            // 0 buffers are full to begin with
            ret = sem_init(&this->queueFull, 0, 0);
            FW_ASSERT(ret == 0, ret); // If this fails, something horrible happened.
            // this is acting as a mutex - no one holds it to start
            ret = sem_init(&this->queueLock, 0, 1);
            FW_ASSERT(ret == 0, ret); // If this fails, something horrible happened.

            FW_ASSERT(data);
            this->data = data;
            this->depth = depth;
            this->msgSize = msgSize;
            fill = 0;
            use = 0;
        }
        ~QueueHandle() { 
            // Destroy the handle:
            (void) sem_destroy(&this->queueEmpty);
            (void) sem_destroy(&this->queueFull);
            (void) sem_destroy(&this->queueLock);
        }
      
        NATIVE_UINT_TYPE getBufferIndex(NATIVE_INT_TYPE index) {
            return (index % this->depth) * (sizeof(NATIVE_INT_TYPE) + this->msgSize);
        }

        // NOTE(Mereweth) - MUST be done with mutex held
        void put(const U8* buffer, NATIVE_INT_TYPE size) {
            NATIVE_UINT_TYPE index = this->getBufferIndex(this->fill);
            this->fill = (this->fill + 1) % this->depth;
            // Copy size of buffer onto queue:
            void* dest = &data[index];
            void* ptr = memcpy(dest, &size, sizeof(size));
            FW_ASSERT(ptr == dest);

            // Copy buffer onto queue:
            index += sizeof(size);
            dest = &data[index];
            ptr = memcpy(dest, buffer, size);
            FW_ASSERT(ptr == dest);
        }

        // NOTE(Mereweth) - MUST be done with mutex held
        Queue::QueueStatus get(U8* buffer, NATIVE_INT_TYPE& size) {
            NATIVE_UINT_TYPE index = this->getBufferIndex(this->use);
            this->use = (this->use + 1) % this->depth;
            // Copy size of buffer from queue:
            NATIVE_UINT_TYPE storedSize;
            void* source = &data[index];
            void* ptr = memcpy(&storedSize, source, sizeof(size));
            FW_ASSERT(ptr == &storedSize);

            // If the buffer passed in is not big
            // enough, return false, and pass out
            // the size of the messsage:
            if(storedSize > size){
                size = storedSize;
                return Queue::QUEUE_SIZE_MISMATCH;
            }
            size = storedSize;

            // Copy buffer from queue:
            index += sizeof(size);
            source = &data[index];
            ptr = memcpy(buffer, source, storedSize);
            FW_ASSERT(ptr == buffer);
            return Queue::QUEUE_OK;
        }
      
        U8* data;
        NATIVE_UINT_TYPE depth;
        NATIVE_UINT_TYPE msgSize;
      
        NATIVE_UINT_TYPE fill;
        NATIVE_UINT_TYPE use;
      
        sem_t queueEmpty;
        sem_t queueFull;
        sem_t queueLock;
    };

    Queue::Queue() :
        m_handle(-1) {
    }

    Queue::QueueStatus Queue::create(const Fw::StringBase &name, NATIVE_INT_TYPE depth, NATIVE_INT_TYPE msgSize) {
        U8* data = new U8[depth*(sizeof(msgSize) + msgSize)];  
        if (NULL == data) {
            return QUEUE_UNINITIALIZED;
        }
        
        // Set up queue handle:
        QueueHandle* queueHandle = new QueueHandle(data, depth, msgSize);
        if (NULL == queueHandle) {
            return QUEUE_UNINITIALIZED;
        }
        this->m_handle = (POINTER_CAST) queueHandle;
        
        Queue::s_numQueues++;

        return QUEUE_OK;
    }

    Queue::~Queue() {
        QueueHandle* queueHandle = (QueueHandle*) this->m_handle;
        delete queueHandle->data;
        delete queueHandle;
    }

    Queue::QueueStatus Queue::send(const U8* buffer, NATIVE_INT_TYPE size, NATIVE_INT_TYPE priority, QueueBlocking block) {

        QueueHandle* queueHandle = (QueueHandle*) this->m_handle;
        U8* data = queueHandle->data;
        sem_t* queueEmpty = &queueHandle->queueEmpty;
        sem_t* queueFull = &queueHandle->queueFull;
        sem_t* queueLock = &queueHandle->queueLock;

        if (NULL == data) {
            return QUEUE_UNINITIALIZED;
        }
        
        if (NULL == buffer) {
            return QUEUE_EMPTY_BUFFER;
        }

        int res = 0;
        if (block == QUEUE_NONBLOCKING) {
            res = sem_trywait(queueEmpty);   
        }
        else { // blocking
            res = sem_wait(queueEmpty);
        }
        
        if (-1 == res) {
            switch (errno) {
                case EINTR:
                    return QUEUE_SEND_ERROR;
                case EINVAL:
                    return QUEUE_UNINITIALIZED;
                case EAGAIN: // only for trywait
                    return QUEUE_FULL;
                default:
                    return QUEUE_UNKNOWN_ERROR;
            }
        }
        
        res = sem_wait(queueLock);
        if (-1 == res) {
            switch (errno) {
                case EINTR:
                    return QUEUE_SEND_ERROR;
                case EINVAL:
                    return QUEUE_UNINITIALIZED;
                default:
                    return QUEUE_UNKNOWN_ERROR;
            }
        }
        // if we got here, we can push onto queue
        queueHandle->put(buffer, size);

        res = sem_post(queueLock);
        if ((-1 == res) && (EINVAL == errno)) {
            return QUEUE_UNINITIALIZED;
        }

        res = sem_post(queueFull);
        if ((-1 == res) && (EINVAL == errno)) {
            return QUEUE_UNINITIALIZED;
        }
        
        return QUEUE_OK;
    }

    Queue::QueueStatus Queue::receive(U8* buffer, NATIVE_INT_TYPE capacity, NATIVE_INT_TYPE &actualSize, NATIVE_INT_TYPE &priority, QueueBlocking block) {


        QueueHandle* queueHandle = (QueueHandle*) this->m_handle;
        U8* data = queueHandle->data;
        sem_t* queueEmpty = &queueHandle->queueEmpty;
        sem_t* queueFull = &queueHandle->queueFull;
        sem_t* queueLock = &queueHandle->queueLock;

        if (NULL == data) {
            return QUEUE_UNINITIALIZED;
        }
        
        if (NULL == buffer) {
            return QUEUE_EMPTY_BUFFER;
        }

        int res = 0;
        if (block == QUEUE_NONBLOCKING) {
            res = sem_trywait(queueFull);   
        }
        else { // blocking
            res = sem_wait(queueFull);
        }
        
        if (-1 == res) {
            switch (errno) {
                case EINTR:
                    return QUEUE_SEND_ERROR;
                case EINVAL:
                    return QUEUE_UNINITIALIZED;
                case EAGAIN: // only for trywait
                    return QUEUE_NO_MORE_MSGS;
                default:
                    return QUEUE_UNKNOWN_ERROR;
            }
        }
        
        res = sem_wait(queueLock);
        if (-1 == res) {
            switch (errno) {
                case EINTR:
                    return QUEUE_SEND_ERROR;
                case EINVAL:
                    return QUEUE_UNINITIALIZED;
                default:
                    return QUEUE_UNKNOWN_ERROR;
            }
        }
        // if we got here, we can pull off queue

        Queue::QueueStatus qStat = queueHandle->get(buffer, actualSize);

        res = sem_post(queueLock);
        if ((-1 == res) && (EINVAL == errno)) {
            return QUEUE_UNINITIALIZED;
        }

        res = sem_post(queueEmpty);
        if ((-1 == res) && (EINVAL == errno)) {
            return QUEUE_UNINITIALIZED;
        }

        return qStat;
    }

    NATIVE_INT_TYPE Queue::getNumMsgs(void) const {
        QueueHandle* queueHandle = (QueueHandle*) this->m_handle;
        return 0;
    }

    NATIVE_INT_TYPE Queue::getMaxMsgs(void) const {
        QueueHandle* queueHandle = (QueueHandle*) this->m_handle;
        return 0;
    }

    NATIVE_INT_TYPE Queue::getQueueSize(void) const {
        QueueHandle* queueHandle = (QueueHandle*) this->m_handle;
        return queueHandle->depth;
    }

    NATIVE_INT_TYPE Queue::getMsgSize(void) const {
        QueueHandle* queueHandle = (QueueHandle*) this->m_handle;
        return queueHandle->msgSize;
    }

}

