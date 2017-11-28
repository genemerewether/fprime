#include <Os/Task.hpp>
#include <Fw/Types/Assert.hpp>

#include <sys/types.h>
#include <unistd.h>

#include <pthread.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

typedef void* (*pthread_func_ptr)(void*);

namespace Os {
    Task::Task() : m_handle(0), m_identifier(0), m_affinity(-1), m_started(false), m_suspendedOnPurpose(false) {
    }

    Task::TaskStatus Task::start(const Fw::StringBase &name, NATIVE_INT_TYPE identifier, NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize, taskRoutine routine, void* arg, NATIVE_INT_TYPE cpuAffinity) {

    	// for linux, task names can only be of length 15, so just setting it to the name:
    	this->m_name = name;
        this->m_identifier = identifier;

        Task::TaskStatus tStat = TASK_OK;

        pthread_attr_t att;
        // clear att; can cause issues
        memset(&att,0,sizeof(att));

        I32 stat = pthread_attr_init(&att);
        if (stat != 0) {
            DEBUG_PRINT("pthread_attr_init: (%d)(%d): %s\n",stat,errno,strerror(stat));
        	return TASK_INVALID_PARAMS;
        }

        stat = pthread_attr_setstacksize(&att,stackSize);
        if (stat != 0) {
            return TASK_INVALID_STACK;
        }

        sched_param schedParam;
        memset(&schedParam,0,sizeof(sched_param));
        schedParam.sched_priority = priority;
        stat = pthread_attr_setschedparam(&att,&schedParam);
        if (stat != 0) {
            DEBUG_PRINT("pthread_attr_setschedparam: %s\n",strerror(errno));
        	return TASK_INVALID_PARAMS;
        }

	stat = pthread_attr_setthreadname(&att,(char*)this->m_name.toChar());
	if (stat != 0) {
	  DEBUG_PRINT("pthread_attr_setthreadname_np: %s %s\n",this->m_name.toChar(),strerror(stat));
	  return TASK_INVALID_PARAMS;
	}

        // Set affinity before creating thread:
        if (cpuAffinity != -1) {

	  //TODO(mereweth) - does this have any effect for DSPAL?
	  cpu_set_t cpuset = 1 << cpuAffinity;

            stat = pthread_attr_setaffinity_np(&att, sizeof(cpu_set_t), &cpuset);
            if (stat != 0) {
                DEBUG_PRINT("pthread_attr_setaffinity_np: %i %s\n",cpuAffinity,strerror(stat));
                return TASK_INVALID_PARAMS;
            }
        }

        // If a registry has been registered, register task
        if (Task::s_taskRegistry) {
            Task::s_taskRegistry->addTask(this);
        }

        pthread_t* tid = new pthread_t;

        stat = pthread_create(tid,&att,(pthread_func_ptr)routine,arg);
	
        switch (stat) {
            case 0:
                this->m_handle = (POINTER_CAST)tid;
                Task::s_numTasks++;
                break;
            case EINVAL:
                delete tid;
                DEBUG_PRINT("pthread_create: %s\n",strerror(errno));
                tStat = TASK_INVALID_PARAMS;
                break;
            default:
                delete tid;
                tStat = TASK_UNKNOWN_ERROR;
                break;
        }

        (void)pthread_attr_destroy(&att);

        return tStat;
    }

    Task::TaskStatus Task::delay(NATIVE_UINT_TYPE milliseconds)
    {
      NATIVE_INT_TYPE stat = 0;
      stat = usleep(milliseconds * 1000);

      if (stat != 0) {
	DEBUG_PRINT("delay: %d %s\n",milliseconds,strerror(errno));
	return TASK_DELAY_ERROR;
      }
      else {
	return TASK_OK;
      }

      return TASK_OK; // for coverage analysis

    }


    Task::~Task() {
    	if (this->m_handle) {
    		delete (pthread_t*)this->m_handle;
    	}
        // If a registry has been registered, remove task
        if (Task::s_taskRegistry) {
            Task::s_taskRegistry->removeTask(this);
        }

    }

    // FIXME: Need to find out how to do this for Posix threads

    void Task::suspend(bool onPurpose) {
        FW_ASSERT(0);
    }

    void Task::resume(void) {
        FW_ASSERT(0);
    }

    bool Task::isSuspended(void) {
        FW_ASSERT(0);
        return false;
    }

}
