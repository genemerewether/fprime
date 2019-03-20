#include <Fw/Comp/IPCQueuedComponentBase.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Types/EightyCharString.hpp>
#include <Fw/Cfg/Config.hpp>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>

#if defined TGT_OS_TYPE_LINUX
#include <signal.h>
#include <sys/prctl.h>
#endif

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Fw {

    class IPCQueuedComponentExitSerializableBuffer : public Fw::SerializeBufferBase {

        public:
            NATIVE_UINT_TYPE getBuffCapacity(void) const {
                return sizeof(m_buff);
            }

            U8* getBuffAddr(void) {
                return m_buff;
            }

            const U8* getBuffAddr(void) const {
                return m_buff;
            }

        private:

            U8 m_buff[sizeof(IPCQueuedComponentBase::IPCQUEUED_COMPONENT_EXIT)];

    };
  
#if FW_OBJECT_NAMES
    IPCQueuedComponentBase::IPCQueuedComponentBase(const char* name) : PassiveComponentBase(name),m_msgsDropped(0),m_pid(-1) {

    }
#else
    IPCQueuedComponentBase::IPCQueuedComponentBase() : PassiveComponentBase(),m_msgsDropped(0),m_pid(-1) {

    }
#endif
    IPCQueuedComponentBase::~IPCQueuedComponentBase() {
        DEBUG_PRINT("IPCQueuedComponent %s destructor, pid %d.\n",this->getObjName(),this->getPID());
    }

    void IPCQueuedComponentBase::init(NATIVE_INT_TYPE instance) {
        PassiveComponentBase::init(instance);
    }
  
    void IPCQueuedComponentBase::exit(void) {
        IPCQueuedComponentExitSerializableBuffer exitBuff;
        SerializeStatus stat = exitBuff.serialize((I32)IPCQUEUED_COMPONENT_EXIT);
        FW_ASSERT(FW_SERIALIZE_OK == stat,static_cast<NATIVE_INT_TYPE>(stat));
        Os::Queue::QueueStatus qStat = this->m_queue.send(exitBuff,0,Os::Queue::QUEUE_NONBLOCKING);
        FW_ASSERT(Os::Queue::QUEUE_OK == qStat,static_cast<NATIVE_INT_TYPE>(qStat));
    }

    int IPCQueuedComponentBase::getPID() {
        return this->m_pid;
    }

    int IPCQueuedComponentBase::spawnChild() {
        this->m_pid = fork();
        if (this->m_pid == 0) { // child process
#if defined TGT_OS_TYPE_LINUX // try to kill the child when the parent exits
            prctl(PR_SET_PDEATHSIG, SIGTERM);
#endif
        }
        return this->m_pid;
    }

#if FW_OBJECT_TO_STRING == 1 && FW_OBJECT_NAMES == 1
    void IPCQueuedComponentBase::toString(char* buffer, NATIVE_INT_TYPE size) {
        (void)snprintf(buffer, size,"QueueComp: %s", this->m_objName);
        buffer[size-1] = 0;
    }
#endif

    Os::Queue::QueueStatus IPCQueuedComponentBase::createQueue(NATIVE_INT_TYPE depth, NATIVE_INT_TYPE msgSize) {

        Fw::EightyCharString queueName;
#if FW_OBJECT_NAMES == 1
        queueName = this->m_objName;
#else
        char queueNameChar[FW_QUEUE_NAME_MAX_SIZE];
        (void)snprintf(queueNameChar,sizeof(queueNameChar),"CompQ_%d",Os::Queue::getNumQueues());
        queueName = queueNameChar;
#endif
    	return this->m_queue.create(queueName, depth, msgSize);
    }

    NATIVE_INT_TYPE IPCQueuedComponentBase::getNumMsgsDropped(void) {
        return this->m_msgsDropped;
    }

    void IPCQueuedComponentBase::incNumMsgDropped(void) {
        this->m_msgsDropped++;
    }

}
