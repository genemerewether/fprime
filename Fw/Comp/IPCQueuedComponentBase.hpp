/*
 * IPCIPCQueuedComponentBase.hpp
 *
 *  Created on: June 19, 2018
 *      Author: mereweth
 */

/*
 * Description:
 */
#ifndef FW_IPCQUEUED_COMPONENT_BASE_HPP
#define FW_IPCQUEUED_COMPONENT_BASE_HPP

#include <Fw/Comp/PassiveComponentBase.hpp>
#include <Os/IPCQueue.hpp>
#include <Os/Task.hpp>
#include <Fw/Cfg/Config.hpp>


namespace Fw {
    class IPCQueuedComponentBase : public PassiveComponentBase {
        public:
            void exit(void); //!< exit task in active/queued component
      
            enum {
                IPCQUEUED_COMPONENT_EXIT //!< message to exit queued component loop
            };
      
            // FIXME: Had to make MsgDispatchStatus public for LLVM. Think LLVM is wrong.
            typedef enum {
                MSG_DISPATCH_OK, //!< Dispatch was normal
                MSG_DISPATCH_EMPTY, //!< No more messages in the queue
                MSG_DISPATCH_ERROR, //!< Errors dispatching messages
                MSG_DISPATCH_EXIT //!< A message was sent requesting an exit of the loop
            } MsgDispatchStatus;

            int getPID(); //!< returns stored PID

        PROTECTED:

#if FW_OBJECT_NAMES == 1
            IPCQueuedComponentBase(const char* name); //!< Constructor
#else
            IPCQueuedComponentBase(); //!< Constructor
#endif
            virtual ~IPCQueuedComponentBase(); //!< Destructor
            void init(NATIVE_INT_TYPE instance); //!< initialization function
            int spawnChild(); //!< forks another process
            Os::IPCQueue m_queue; //!< queue object for active component
            Os::Queue::QueueStatus createQueue(NATIVE_INT_TYPE depth, NATIVE_INT_TYPE msgSize);
            virtual MsgDispatchStatus doDispatch(void)=0; //!< method to dispatch a single message in the queue.
#if FW_OBJECT_TO_STRING == 1
            virtual void toString(char* str, NATIVE_INT_TYPE size); //!< dump string representation of component
#endif
            NATIVE_INT_TYPE getNumMsgsDropped(void); //!< return number of messages dropped
            void incNumMsgDropped(void); //!< increment the number of messages dropped
        PRIVATE:
            NATIVE_INT_TYPE m_msgsDropped; //!< number of messages dropped from full queue
            NATIVE_INT_TYPE m_pid; //!< store the result of the fork call
    };

}
#endif
