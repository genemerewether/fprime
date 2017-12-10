// ====================================================================== 
// \title  HexRouterImpl.cpp
// \author mereweth
// \brief  cpp file for HexRouter component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
// 
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ====================================================================== 


#include <SnapdragonFlight/HexRouter/HexRouterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/EightyCharString.hpp"

#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  HexRouterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    HexRouterComponentImpl(
        const char *const compName
    ) :
      HexRouterComponentBase(compName)
#else
    HexRouterImpl(void)
#endif
  {
      // Initialize memory buffer objects
      for (NATIVE_UINT_TYPE buff = 0; buff < RECEIVE_BUFFER_POOL_SIZE; buff++) {
	  this->m_inputBuffObj[buff].setdata((U64)this->m_inputBuff[buff]);
	  this->m_inputBuffObj[buff].setsize(RECEIVE_BUFFER_SIZE);

	  // After creation, buffer set ordering can change as some consumers are slower to process and return
	  this->m_buffSet[buff].readBuffer = this->m_inputBuffObj[buff];
	  this->m_buffSet[buff].available = true;
      }
  }

  void HexRouterComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE msgSize,
        const NATIVE_INT_TYPE instance
    ) 
  {
    HexRouterComponentBase::init(queueDepth, msgSize, instance);
  }

  HexRouterComponentImpl ::
    ~HexRouterComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------
    void HexRouterComponentImpl ::
    readBufferRecv_handler(
	const NATIVE_INT_TYPE portNum,
	Fw::Buffer Buffer
    )
  {
      this->m_readBuffMutex.lock();
      bool found = false;

      // search for open entry
      for (NATIVE_UINT_TYPE entry = 0; entry < RECEIVE_BUFFER_POOL_SIZE; entry++) {
	  // Look for slots to fill. "Available" is from
	  // the perspective of the driver thread looking for
	  // buffers to fill, so add the buffer and make it available.
	  if (not this->m_buffSet[entry].available) {
	      this->m_buffSet[entry].readBuffer = Buffer;
	      this->m_buffSet[entry].available = true;
	      found = true;
	      break;
	  }
      }
      this->m_readBuffMutex.unLock();
      FW_ASSERT(found,Buffer.getbufferID(),Buffer.getmanagerID());

  }
  
  void HexRouterComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO telemetry
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void HexRouterComponentImpl ::
    KraitPortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {   
        unsigned char* data =
                reinterpret_cast<unsigned char*>(Buffer.getBuffAddr());
        NATIVE_INT_TYPE xferSize = Buffer.getBuffLength();

        for (NATIVE_INT_TYPE chunk = 0; chunk < xferSize; chunk +=
                WRITE_BUFF_SIZE) {

            NATIVE_INT_TYPE thisSize = FW_MIN(WRITE_BUFF_SIZE,
                    xferSize - chunk);
            NATIVE_INT_TYPE stat = rpc_relay_write(portNum, data + chunk, thisSize);
            if (-1 == stat) {
                this->log_WARNING_HI_HR_WriteError(stat);
                return;
            }
        }
  }

  // ----------------------------------------------------------------------
  // Implementations of class methods
  // ----------------------------------------------------------------------
  
    void HexRouterComponentImpl::startPortReadThread(
            NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize,
            NATIVE_INT_TYPE cpuAffinity) {

        Fw::EightyCharString task("HexPortReader");
	// TODO(mereweth) - may want to start multiple read threads depending on FastRPC call latency
        Os::Task::TaskStatus stat = this->m_portReadTask.start(task, 0, priority, stackSize,
							       hexPortReadTaskEntry, this, cpuAffinity);
        FW_ASSERT(stat == Os::Task::TASK_OK, stat);
    }

    void HexRouterComponentImpl::hexPortReadTaskEntry(void * ptr) {
        HexRouterComponentImpl* comp = static_cast<HexRouterComponentImpl*>(ptr);

	uint8_t buff[READ_PORT_SIZE];
	Fw::ExternalSerializeBuffer portBuff(buff, READ_PORT_SIZE);
	
	int stat = rpc_relay_port_allocate(READ_PORT_SIZE);
        if (-1 == stat) {
            comp->log_WARNING_HI_HR_MemoryError(HR_HEX_ALLOC);
            return;
        }

        while (1) {
            // wait for data
            int sizeRead = 0;
	    int portNum = 0;

//          timespec stime;
//          (void)clock_gettime(CLOCK_REALTIME,&stime);
//          printf("<<< Calling rpc_relay_port_read() at %d %d\n", stime.tv_sec, stime.tv_nsec);

            bool waiting = true;
            int stat = 0;
	    
            while (waiting) {
                stat = rpc_relay_port_read(&portNum,
					   reinterpret_cast<unsigned char*>(buff),
					   READ_PORT_SIZE, &sizeRead);

                // check for timeout
                if (1 == stat) {
                    if (comp->m_quitReadThread) {
                        return;
                    }
                } else { // quit if other error or if data received
                    waiting = false;
                }
            }

            if (comp->m_quitReadThread) {
                return;
            }

            if (stat != 0) {
	        comp->log_WARNING_HI_HR_DataReceiveError(HR_RELAY_READ_ERR, stat);
            } else {
	      /*stat = portBuff.deserialize(buff, sizeRead, true);
		if (stat != Fw::FW_SERIALIZE_OK) {
		    DEBUG_PRINT("Decode data error\n");
		    comp->tlmWrite_HR_NumDecodeErrors(++comp->m_numDecodeErrors);
		    return;
		}
		// set buffer to size of data*/
		stat = portBuff.setBuffLen(sizeRead);
		if (stat != Fw::FW_SERIALIZE_OK) {
		    DEBUG_PRINT("Set setBuffLen error\n");
		    comp->tlmWrite_HR_NumDecodeErrors(++comp->m_numDecodeErrors);
		    return;
		}

//              (void)clock_gettime(CLOCK_REALTIME,&stime);
//              printf("<!<! Sending data to port %u size %u at %d %d\n", portNum, buff.getsize(), stime.tv_sec, stime.tv_nsec);
		// call output port
		DEBUG_PRINT("Calling port %d with %d bytes.\n", portNum, sizeRead);
		if (comp->isConnected_HexPortsOut_OutputPort(portNum)) {
		    Fw::SerializeStatus stat = comp->HexPortsOut_out(portNum, portBuff);

		    // If had issues deserializing the data, then report it:
		    if (stat != Fw::FW_SERIALIZE_OK) {
		      DEBUG_PRINT("HexPortsOut_out() serialize status error\n");
		      comp->tlmWrite_HR_NumBadSerialPortCalls(++comp->m_numBadSerialPortCalls);
		      comp->log_WARNING_HI_HR_BadSerialPortCall(stat, portNum);
		      return;
		    }
		}
		else {
		    comp->log_WARNING_HI_HR_InvalidPortNum(HR_BUFF_SEND, portNum);
		}
            }
        }
    }
    
    void HexRouterComponentImpl::startBuffReadThread(
            NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize,
            NATIVE_INT_TYPE cpuAffinity) {

        Fw::EightyCharString task("HexBuffReader");
        Os::Task::TaskStatus stat = this->m_buffReadTask.start(task, 0, priority, stackSize,
							       hexBuffReadTaskEntry, this, cpuAffinity);
        FW_ASSERT(stat == Os::Task::TASK_OK, stat);
    }

    void HexRouterComponentImpl::hexBuffReadTaskEntry(void * ptr) {
        HexRouterComponentImpl* comp = static_cast<HexRouterComponentImpl*>(ptr);

	int stat = rpc_relay_buff_allocate(READ_BUFF_SIZE);
        if (-1 == stat) {
            comp->log_WARNING_HI_HR_MemoryError(HR_HEX_ALLOC);
            return;
        }

        Fw::Buffer buff;

        while (1) {
            // wait for data
            int sizeRead = 0;
	    int portNum = 0;

            // find open buffer

            comp->m_readBuffMutex.lock();
            // search for open entry
            NATIVE_INT_TYPE entryFound = false;

	    NATIVE_INT_TYPE entry = 0;
            for (entry = 0; entry < RECEIVE_BUFFER_POOL_SIZE; entry++) {
                if (comp->m_buffSet[entry].available) {
                    comp->m_buffSet[entry].available = false;
                    buff = comp->m_buffSet[entry].readBuffer;
                    entryFound = true;
                    break;
                }
            }
            comp->m_readBuffMutex.unLock();

            if (not entryFound) {
	        comp->log_WARNING_HI_HR_DataReceiveError(HR_NO_BUFFERS, 0);
                // to avoid spinning, wait 50 ms
                Os::Task::delay(50);
                continue;
            }

//          timespec stime;
//          (void)clock_gettime(CLOCK_REALTIME,&stime);
//          printf("<<< Calling rpc_relay_buff_read() at %d %d\n", stime.tv_sec, stime.tv_nsec);

            bool waiting = true;
            int stat = 0;

	    
            while (waiting) {

                stat = rpc_relay_buff_read(&portNum,
					   reinterpret_cast<unsigned char*>(buff.getdata()),
					   buff.getsize(), &sizeRead);

                // check for timeout
                if (1 == stat) {
                    if (comp->m_quitReadThread) {
                        return;
                    }
                } else { // quit if other error or if data received
                    waiting = false;
                }
            }

            if (comp->m_quitReadThread) {
                return;
            }

            if (stat != 0) {
	        comp->log_WARNING_HI_HR_DataReceiveError(HR_RELAY_READ_ERR, stat);
		// TODO(mereweth) send buffer to destination with error flag instead of recycling
		comp->m_readBuffMutex.lock();
		comp->m_buffSet[entry].available = true;
		comp->m_readBuffMutex.unLock();
            } else {
//              (void)clock_gettime(CLOCK_REALTIME,&stime);
//              printf("<!<! Sending data to port %u size %u at %d %d\n", portNum, buff.getsize(), stime.tv_sec, stime.tv_nsec);
                buff.setsize(sizeRead);
		if (comp->isConnected_readBufferSend_OutputPort(portNum)) {
		    comp->readBufferSend_out(portNum, buff);
		}
		else {
		    comp->log_WARNING_HI_HR_InvalidPortNum(HR_BUFF_SEND, portNum);
		    comp->m_readBuffMutex.lock();
		    comp->m_buffSet[entry].available = true;
		    comp->m_readBuffMutex.unLock();
		}
            }
        }
    }

    void HexRouterComponentImpl::quitReadThread(void) {
        this->m_quitReadThread = true;
        //rpc_relay_quit();
    }
} // end namespace SnapdragonFlight
