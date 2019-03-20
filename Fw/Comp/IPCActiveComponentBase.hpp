/*
 * IPCActiveComponentBase.hpp
 *
 *  Created on: June 19, 2018
 *      Author: mereweth
 */

/*
 * Description:
 */
#ifndef FW_IPCACTIVE_COMPONENT_BASE_HPP
#define FW_IPCACTIVE_COMPONENT_BASE_HPP

#include <Fw/Comp/IPCQueuedComponentBase.hpp>
#include <Os/Task.hpp>
#include <Fw/Cfg/Config.hpp>


namespace Fw {
    class IPCActiveComponentBase : public IPCQueuedComponentBase {
        public:
            void start(NATIVE_INT_TYPE identifier, NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize, NATIVE_INT_TYPE cpuAffinity = -1); //!< called by instantiator when task is to be started
            Os::Task::TaskStatus join(void **value_ptr); //!< provide return value of thread if value_ptr is not NULL

        PROTECTED:
#if FW_OBJECT_NAMES == 1
            IPCActiveComponentBase(const char* name); //!< Constructor
#else
            IPCActiveComponentBase(); //!< Constructor
#endif
            virtual ~IPCActiveComponentBase(); //!< Destructor
            void init(NATIVE_INT_TYPE instance); //!< initialization code
            virtual void preamble(void); //!< A function that will be called before the event loop is entered
            virtual void loop(void); //!< The function that will loop dispatching messages
            virtual void finalizer(void); //!< A function that will be called after exiting the loop
            Os::Task m_task; //!< task object for active component
#if FW_OBJECT_TO_STRING == 1
            virtual void toString(char* str, NATIVE_INT_TYPE size); //!< create string description of component
#endif
        PRIVATE:
            static void s_baseTask(void*); //!< function provided to task class for new thread.
    };

}
#endif
