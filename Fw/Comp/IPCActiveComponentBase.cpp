#include <Fw/Cfg/Config.hpp>
#include <Fw/Comp/IPCActiveComponentBase.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Types/EightyCharString.hpp>
#include <stdio.h>

#include <stdio.h> // TODO(mereweth@jpl.nasa.gov) - remove the debug prints
#include <sys/time.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Fw {

#if FW_OBJECT_NAMES == 1
    IPCActiveComponentBase::IPCActiveComponentBase(const char* name) : IPCQueuedComponentBase(name) {

    }
#else
    IPCActiveComponentBase::IPCActiveComponentBase() : IPCQueuedComponentBase() {

    }
#endif
    IPCActiveComponentBase::~IPCActiveComponentBase() {
        DEBUG_PRINT("IPCActiveComponent %s destructor, pid %d.\n",this->getObjName(),this->getPID());
    }

    void IPCActiveComponentBase::init(NATIVE_INT_TYPE instance) {
        IPCQueuedComponentBase::init(instance);
    }

#if FW_OBJECT_TO_STRING == 1 && FW_OBJECT_NAMES == 1
    void IPCActiveComponentBase::toString(char* buffer, NATIVE_INT_TYPE size) {
        (void)snprintf(buffer, size, "ActComp: %s", this->m_objName);
        buffer[size-1] = 0;
    }
#endif

    void IPCActiveComponentBase::start(NATIVE_INT_TYPE identifier, NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize, NATIVE_INT_TYPE cpuAffinity) {

        Fw::EightyCharString taskName;

#if FW_OBJECT_NAMES == 1
        taskName = this->getObjName();
#else
        char taskNameChar[FW_TASK_NAME_MAX_SIZE];
        (void)snprintf(taskNameChar,sizeof(taskNameChar),"ActComp_%d",Os::Task::getNumTasks());
        taskName = taskNameChar;
#endif

    	Os::Task::TaskStatus status = this->m_task.start(taskName, identifier, priority, stackSize, this->s_baseTask,this, cpuAffinity);
    	FW_ASSERT(status == Os::Task::TASK_OK,(NATIVE_INT_TYPE)status);
    }

    Os::Task::TaskStatus IPCActiveComponentBase::join(void **value_ptr) {
        DEBUG_PRINT("join %s\n", this->getObjName());
        return this->m_task.join(value_ptr);
    }

    void IPCActiveComponentBase::s_baseTask(void* ptr) {
        // cast void* back to active component
        IPCActiveComponentBase* comp = static_cast<IPCActiveComponentBase*> (ptr);
        // indicated that task is started
        comp->m_task.setStarted(true);
        // print out message when task is started
        DEBUG_PRINT("Active Component %s task started, pid %d.\n",comp->getObjName(),comp->getPID());
        // call preamble
        comp->preamble();
        // call main task loop until exit or error
        comp->loop();
        // if main loop exits, call finalizer
        comp->finalizer();
    }

    void IPCActiveComponentBase::loop(void) {

        bool quitLoop = false;
        while (!quitLoop) {
            MsgDispatchStatus loopStatus = this->doDispatch();
            switch (loopStatus) {
                case MSG_DISPATCH_OK: // if normal message processing, continue
                    break;
                case MSG_DISPATCH_EXIT:
                    DEBUG_PRINT("Active Component %s task exiting, pid %d.\n",this->getObjName(),this->getPID());
                    quitLoop = true;
                    break;
                default:
                    FW_ASSERT(0,(NATIVE_INT_TYPE)loopStatus);
            }
        }

    }

    void IPCActiveComponentBase::preamble(void) {
    }

    void IPCActiveComponentBase::finalizer(void) {
    }

}
