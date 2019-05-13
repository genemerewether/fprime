#include <Fw/Cfg/Config.hpp>
#include <Fw/Comp/ActiveComponentBase.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Types/EightyCharString.hpp>
#include <stdio.h>

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#include <stdio.h>
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

#undef DEBUG_PRINT
#define DEBUG_PRINT(x,...)

namespace Fw {

#if FW_OBJECT_NAMES == 1
    ActiveComponentBase::ActiveComponentBase(const char* name) : QueuedComponentBase(name) {

    }
#else
    ActiveComponentBase::ActiveComponentBase() : QueuedComponentBase() {

    }
#endif
    ActiveComponentBase::~ActiveComponentBase() {
        DEBUG_PRINT("ActiveComponent %s destructor.\n",this->getObjName());
    }

    void ActiveComponentBase::init(NATIVE_INT_TYPE instance) {
        QueuedComponentBase::init(instance);
    }

#if FW_OBJECT_TO_STRING == 1 && FW_OBJECT_NAMES == 1
    void ActiveComponentBase::toString(char* buffer, NATIVE_INT_TYPE size) {
        (void)snprintf(buffer, size, "ActComp: %s", this->m_objName);
        buffer[size-1] = 0;
    }
#endif

    void ActiveComponentBase::start(NATIVE_INT_TYPE identifier, NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize, NATIVE_INT_TYPE cpuAffinity) {

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

    Os::Task::TaskStatus ActiveComponentBase::join(void **value_ptr) {
        DEBUG_PRINT("join %s\n", this->getObjName());
        return this->m_task.join(value_ptr);
    }

    void ActiveComponentBase::s_baseTask(void* ptr) {
        // cast void* back to active component
        ActiveComponentBase* comp = static_cast<ActiveComponentBase*> (ptr);
        // indicated that task is started
        comp->m_task.setStarted(true);
        // print out message when task is started
#if FW_OBJECT_NAMES == 1
        DEBUG_PRINT("after task start for name %s\n",
                    comp->getObjName());
#endif
        // printf("Active Component %s task started.\n",comp->getObjName());
        // call preamble
        comp->preamble();
        // call main task loop until exit or error
        comp->loop();
        // if main loop exits, call finalizer
        comp->finalizer();
    }

    void ActiveComponentBase::loop(void) {

        bool quitLoop = false;
        while (!quitLoop) {
            MsgDispatchStatus loopStatus = this->doDispatch();
            switch (loopStatus) {
                case MSG_DISPATCH_OK: // if normal message processing, continue
                    break;
                case MSG_DISPATCH_EXIT:
                    quitLoop = true;
                    break;
                default:
                    FW_ASSERT(0,(NATIVE_INT_TYPE)loopStatus);
            }
        }

    }

    void ActiveComponentBase::preamble(void) {
    }

    void ActiveComponentBase::finalizer(void) {
    }

}
