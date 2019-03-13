SRC = 			TaskCommon.cpp \
				TaskString.cpp \
				QueueCommon.cpp \
				QueueString.cpp \
				IPCQueueCommon.cpp \
				SimpleQueueRegistry.cpp \
				MemCommon.cpp \
				ValidatedFile.cpp \
				ValidateFileCommon.cpp 

HDR = 			Queue.hpp \
				IPCQueue.hpp \
				QueueString.hpp \
				SimpleQueueRegistry.hpp \
				Task.hpp \
				TaskString.hpp \
				InterruptLock.hpp \
				IntervalTimer.hpp \
				WatchdogTimer.hpp \
				Mutex.hpp \
				File.hpp \
				ValidateFile.hpp \
				ValidatedFile.hpp \
				FileSystem.hpp \
				LocklessQueue.hpp

SRC_LINUX=      Posix/IPCQueue.cpp \
		Pthreads/Queue.cpp \
		Pthreads/BufferQueueCommon.cpp \
                Pthreads/PriorityBufferQueue.cpp \
                Pthreads/MaxHeap/MaxHeap.cpp \
				Linux/File.cpp \
				Posix/Task.cpp \
				LogPrintf.cpp \
				Linux/InterruptLock.cpp \
				Linux/WatchdogTimer.cpp \
				X86/IntervalTimer.cpp \
				Linux/IntervalTimer.cpp \
				Posix/Mutex.cpp \
				Linux/FileSystem.cpp \
				Posix/LocklessQueue.cpp

SRC_DARWIN =   	MacOs/IPCQueueStub.cpp \ # NOTE(mereweth) - provide a stub that only works in single-process, not IPC
		Pthreads/Queue.cpp \
                Pthreads/BufferQueueCommon.cpp \
                Pthreads/PriorityBufferQueue.cpp \
                Pthreads/MaxHeap/MaxHeap.cpp \
				Linux/File.cpp \
				Posix/Task.cpp \
				LogPrintf.cpp \
				Linux/WatchdogTimer.cpp \
				Linux/InterruptLock.cpp \
				X86/IntervalTimer.cpp \
				MacOs/IntervalTimer.cpp \
				Posix/Mutex.cpp \
				Linux/FileSystem.cpp  \
				Posix/LocklessQueue.cpp

SRC_CYGWIN =    Pthreads/Queue.cpp \
		Pthreads/BufferQueueCommon.cpp \
                Pthreads/PriorityBufferQueue.cpp \
                Pthreads/MaxHeap/MaxHeap.cpp \
				Linux/File.cpp \
				Posix/Task.cpp \
				LogPrintf.cpp \
				Linux/InterruptLock.cpp \
				Linux/WatchdogTimer.cpp \
				X86/IntervalTimer.cpp \
				Linux/IntervalTimer.cpp \
				Posix/Mutex.cpp \
				Linux/FileSystem.cpp \
				Posix/LocklessQueue.cpp

SRC_SDFLIGHT =  Posix/IPCQueue.cpp \
		Pthreads/Queue.cpp \
		Pthreads/BufferQueueCommon.cpp \
                Pthreads/PriorityBufferQueue.cpp \
                Pthreads/MaxHeap/MaxHeap.cpp \
				Linux/File.cpp \
				Posix/TaskRoot.cpp \
				LogPrintf.cpp \
				Linux/InterruptLock.cpp \
				Linux/WatchdogTimer.cpp \
				X86/IntervalTimer.cpp \
				Linux/IntervalTimer.cpp \
				Posix/Mutex.cpp \
				Posix/LocklessQueue.cpp \
				Linux/FileSystem.cpp	

SRC_DSPAL =  Pthreads/Queue.cpp \
	 	Pthreads/BufferQueueCommon.cpp \
                Pthreads/PriorityBufferQueue.cpp \
                Pthreads/MaxHeap/MaxHeap.cpp \
				Dspal/FileStub.cpp \
				Dspal/IPCQueueStub.cpp \
				Dspal/Task.cpp \
				LogPrintf.cpp \
				Linux/WatchdogTimer.cpp \
				X86/IntervalTimer.cpp \
				Dspal/IntervalTimer.cpp \
				Posix/Mutex.cpp \
				Posix/LocklessQueue.cpp \
				Dspal/FileSystem.cpp

SRC_LINUXRT =   Posix/IPCQueue.cpp \
		Pthreads/Queue.cpp \
		Pthreads/BufferQueueCommon.cpp \
                Pthreads/PriorityBufferQueue.cpp \
                Pthreads/MaxHeap/MaxHeap.cpp \
				Linux/File.cpp \
				Posix/TaskRoot.cpp \
				LogPrintf.cpp \
				Linux/InterruptLock.cpp \
				Linux/WatchdogTimer.cpp \
				X86/IntervalTimer.cpp \
				Linux/IntervalTimer.cpp \
				Posix/Mutex.cpp \
				Linux/FileSystem.cpp \
				Posix/LocklessQueue.cpp

SRC_TIR5 =	TIR5/IntervalTimer.cpp \
		LogPrintf.cpp \
		MutexStub.cpp

SUBDIRS = test

# to use Pthread priority queue include:
#        Pthreads/Queue.cpp \
#        Pthreads/BufferQueueCommon.cpp \
#        Pthreads/PriorityBufferQueue.cpp \
#        Pthreads/MaxHeap/MaxHeap.cpp \


# to use Pthread fifo queue include:
#        Pthreads/Queue.cpp \
#        Pthreads/BufferQueueCommon.cpp \
#        Pthreads/FIFOBufferQueue.cpp \

