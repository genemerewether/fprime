// ======================================================================
// \Title  HexRouterImpl.cpp
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

#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImplCfg.hpp>

#include <SnapdragonFlight/HexRouter/HexRouterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/EightyCharString.hpp"
#include "Fw/Types/SerialBuffer.hpp"

#include <SnapdragonFlight/RpcCommon/wrap_rpc.h>

#ifdef BUILD_SDFLIGHT
#include <rpcmem.h>
#endif // BUILD_SDFLIGHT

#include <time.h>
#include <unistd.h>
#include <stdlib.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    HexRouterComponentImpl ::
      HexRouterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName
#endif
      ) :
        HexRouterComponentBase(
#if FW_OBJECT_NAMES == 1
                               compName
#endif
                               ),
        m_portReadTask(),
        m_buffReadTask(),
        m_buffSet(),
        m_inputBuff(),
        m_inputBuffObj(),
        m_readBuffMutex(),
        m_quitReadThreads(false),
        m_numDecodeErrors(0u),
        m_numBadSerialPortCalls(0u),
        m_numPackets(0u),
        m_numInvalidPorts(0u),
        m_rpcPortWriteBuff(NULL),
        m_localMsgSize(0u),
        m_allocatorId(0)
    {
        // Initialize memory buffer objects
        // TODO(mereweth) switch to rpcmem_alloc if actually using these
        for (NATIVE_UINT_TYPE buff = 0; buff < HR_RECEIVE_BUFFER_POOL_SIZE; buff++) {
            this->m_inputBuffObj[buff].setdata((U64)this->m_inputBuff[buff]);
            this->m_inputBuffObj[buff].setsize(HR_RECEIVE_BUFFER_SIZE);

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

        m_localMsgSize = msgSize + 2 * sizeof(U32);
#ifdef BUILD_SDFLIGHT
        // TODO(mereweth) - switch to DspRpcAllocator?
        rpcmem_init();
        this->m_rpcPortWriteBuff = (uint8_t*)rpcmem_alloc(22, 0, m_localMsgSize);
#else
        this->m_rpcPortWriteBuff = (uint8_t*)malloc(m_localMsgSize);
#endif
    }

    HexRouterComponentImpl ::
      ~HexRouterComponentImpl(void)
    {
        if (m_rpcPortWriteBuff) {
#ifdef BUILD_SDFLIGHT
            rpcmem_free(m_rpcPortWriteBuff);
#else
            free(m_rpcPortWriteBuff);
#endif
        }
        m_rpcPortWriteBuff = NULL;
        m_localMsgSize = 0;
    }

    // TODO(mereweth) - allocate buffer pools - change name to allocateBuffers
    void HexRouterComponentImpl ::
      allocateBuffer(NATIVE_INT_TYPE identifier, Fw::MemAllocator& allocator, NATIVE_UINT_TYPE bytes) {
        this->m_allocatorId = identifier;
        //static_cast<U8*>(allocator.allocate(identifier,bytes))
    }

    void HexRouterComponentImpl ::
      deallocateBuffer(Fw::MemAllocator& allocator) {
        //allocator.deallocate(this->m_allocatorId,this->m_seqBuffer.getBuffAddr());
    }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------
    void HexRouterComponentImpl ::
      readBufferRecv_handler(const NATIVE_INT_TYPE portNum,
                             Fw::Buffer Buffer)
    {
        this->m_readBuffMutex.lock();
        bool found = false;

        // search for open entry
        for (NATIVE_UINT_TYPE entry = 0; entry < HR_RECEIVE_BUFFER_POOL_SIZE; entry++) {
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
        DEBUG_PRINT("KraitPortsIn_handler for port %d with %d bytes\n",
                    portNum, Buffer.getBuffLength());

        Fw::SerialBuffer sendBuff(m_rpcPortWriteBuff, m_localMsgSize);
        Fw::SerializeStatus serStat = sendBuff.serialize(portNum);
        FW_ASSERT(serStat == Fw::FW_SERIALIZE_OK, serStat);
        serStat = sendBuff.serialize(Buffer);
        FW_ASSERT(serStat == Fw::FW_SERIALIZE_OK, serStat);

        NATIVE_INT_TYPE stat = rpc_relay_port_write(m_rpcPortWriteBuff,
                                                    sendBuff.getBuffLength());
        // TODO(mereweth) - write error codes
        if (KR_RTN_OK != stat) {
            this->log_WARNING_HI_HR_WriteError(stat);
            return;
        }
    }

  // ----------------------------------------------------------------------
  // Implementations of class methods
  // ----------------------------------------------------------------------

    void HexRouterComponentImpl::startPortReadThread(
            NATIVE_INT_TYPE priority, NATIVE_INT_TYPE stackSize,
            NATIVE_INT_TYPE cpuAffinity) {

        Fw::EightyCharString task("HexPortReader");
        /* TODO(mereweth) - may want to start multiple read threads
         * depending on FastRPC call latency - have to handle the concurrency
         * issue in KraitRouter then
         */
        Os::Task::TaskStatus stat =
                    this->m_portReadTask.start(task, 0,
                                               priority, stackSize,
                                               hexPortReadTaskEntry, this,
                                               cpuAffinity);
        FW_ASSERT(stat == Os::Task::TASK_OK, stat);
    }

    void HexRouterComponentImpl::hexPortReadTaskEntry(void * ptr) {
        HexRouterComponentImpl* comp =
                  static_cast<HexRouterComponentImpl*>(ptr);

#ifdef BUILD_SDFLIGHT
        // TODO(mereweth) - switch to DspRpcAllocator?
        rpcmem_init();
        uint8_t* buff = (uint8_t*)rpcmem_alloc(22, 0, HR_RPC_READ_SIZE);
#else
        uint8_t* buff = (uint8_t*)malloc(HR_RPC_READ_SIZE);
#endif
        FW_ASSERT(buff != NULL, 0);
        Fw::SerialBuffer rpcCallBuff(buff, HR_RPC_READ_SIZE);

        while (!comp->m_quitReadThreads) {
            // wait for data
            int sizeRead = 0;
            unsigned int portNum = 0;
            bool waiting = true;

            timespec stime;
            (void)clock_gettime(CLOCK_REALTIME,&stime);
            DEBUG_PRINT("Calling rpc_relay_port_read() at %d %d\n",
                        stime.tv_sec, stime.tv_nsec);

            int stat = 0;

            while (waiting) {
                stat = rpc_relay_port_read(reinterpret_cast<unsigned char*>(buff),
                                           HR_RPC_READ_SIZE, &sizeRead);
                // setBuffLen to sizeRead called below

                timespec stime;
                (void)clock_gettime(CLOCK_REALTIME,&stime);
                DEBUG_PRINT("After rpc_relay_port_read() at %d %d; quit? %d\n",
                            stime.tv_sec, stime.tv_nsec, comp->m_quitReadThreads);

                // TODO(mereweth) - is this the best strategy for performance? Favor the DSP
                // check for timeout
                if ((stat == KR_RTN_OK) &&
                    (sizeRead == 0))     {
                    if (comp->m_quitReadThreads) {
                        waiting = false;
                        break;
                    }
                    usleep(HR_NO_MSG_SLEEP_US);
                } else { // quit if other error or if data received
                    waiting = false;
                }
            }

            if (comp->m_quitReadThreads) {
#ifdef BUILD_SDFLIGHT
                rpcmem_free(buff);
#else
                free(buff);
#endif
                return;
            }

            // TODO(mereweth) - make EVRs for exit
            switch (stat) {
                case KR_RTN_QUIT_PREINIT:
                case KR_RTN_QUIT:
                    comp->m_quitReadThreads = true;
                    break;
                case KR_RTN_OK:
                {
                    Fw::SerializeStatus serStat = Fw::FW_SERIALIZE_OK;
                    // set buffer to total size of data*/
                    serStat = rpcCallBuff.setBuffLen(sizeRead);
                    rpcCallBuff.resetDeser();
                    if (serStat != Fw::FW_SERIALIZE_OK) {
                        DEBUG_PRINT("Set setBuffLen error\n");
                        comp->tlmWrite_HR_NumDecodeErrors(++comp->m_numDecodeErrors);
                        break;
                    }
                    while (serStat == Fw::FW_SERIALIZE_OK) {
                        serStat = rpcCallBuff.deserialize(portNum);
                        // first read into loop; could be empty here
                        if (serStat == Fw::FW_DESERIALIZE_BUFFER_EMPTY) {
                            DEBUG_PRINT("first read into loop - empty here\n");
                            break;
                        }
                        
                        if (serStat != Fw::FW_SERIALIZE_OK) {
                            DEBUG_PRINT("deserialize portNum error\n");
                            comp->tlmWrite_HR_NumDecodeErrors(++comp->m_numDecodeErrors);
                            break;
                        }

                        // pull out one port call's worth
                        Fw::ExternalSerializeBuffer portBuff;
                        serStat = rpcCallBuff.deserializeNoCopy(portBuff);
                        if (serStat != Fw::FW_SERIALIZE_OK) {
                            DEBUG_PRINT("deserialize portBuff error %d, port %d\n", serStat, portNum);
                            comp->tlmWrite_HR_NumDecodeErrors(++comp->m_numDecodeErrors);
                            break;
                        }

                        // (void)clock_gettime(CLOCK_REALTIME,&stime);
                        // printf("<!<! Sending data to port %u size %u at %d %d\n", portNum, buff.getsize(), stime.tv_sec, stime.tv_nsec);

                        // call output port
                        if (portNum >= comp->getNum_HexPortsOut_OutputPorts()) {
                            DEBUG_PRINT("portNum %d too big\n", portNum, comp->getNum_HexPortsOut_OutputPorts());
                            comp->tlmWrite_HR_NumDecodeErrors(++comp->m_numDecodeErrors);
                            break;
                        }

                        if (comp->isConnected_HexPortsOut_OutputPort(portNum)) {
                            DEBUG_PRINT("Calling port %d with %d bytes.\n",
                                        portNum, portBuff.getBuffLength());
                            Fw::SerializeStatus serStat = comp->HexPortsOut_out(portNum,
                                                                                portBuff);

                            // If had issues deserializing the data, then report it:
                            if (serStat != Fw::FW_SERIALIZE_OK) {
                                DEBUG_PRINT("HexPortsOut_out() serialize status error %d\n", serStat);
                                comp->tlmWrite_HR_NumBadSerialPortCalls(++comp->m_numBadSerialPortCalls);
                                comp->log_WARNING_HI_HR_BadSerialPortCall(serStat,
                                                                          portNum);
                                break;
                            }
                        }
                        else {
                            comp->log_WARNING_HI_HR_InvalidPortNum(HR_BUFF_SEND, portNum);
                        }
                    }
                    //TODO(mereweth) - EVR that bytes remain
                    if (serStat != Fw::FW_DESERIALIZE_BUFFER_EMPTY) {
                        comp->tlmWrite_HR_NumDecodeErrors(++comp->m_numDecodeErrors);
                        DEBUG_PRINT("%d bytes remain\n", rpcCallBuff.getBuffLeft());
                    }
                }
                    break;
                case KR_RTN_FASTRPC_FAIL:
                default:
                    timespec stime;
                    (void)clock_gettime(CLOCK_REALTIME,&stime);
                    DEBUG_PRINT("rpc_relay_port_read error stat %d at %d:%d\n",
                                stat, stime.tv_sec, stime.tv_nsec);
                    comp->log_WARNING_HI_HR_DataReceiveError(HR_RELAY_READ_ERR,
                                                             stat);
                    break;
            }

            (void)clock_gettime(CLOCK_REALTIME,&stime);
            DEBUG_PRINT("Done with hexPortReadTaskEntry loop at %d %d\n",
                        stime.tv_sec, stime.tv_nsec);
        } // end of outer while loop

        if (comp->m_quitReadThreads) {
#ifdef BUILD_SDFLIGHT
            rpcmem_free(buff);
#else
            free(buff);
#endif
            return;
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
        FW_ASSERT(0); // NOTE(mereweth) - not fully implemented yet

        HexRouterComponentImpl* comp = static_cast<HexRouterComponentImpl*>(ptr);

        Fw::Buffer buff;

        while (1) {
            // wait for data
            int sizeRead = 0;
            unsigned int portNum = 0;

            // find open buffer

            comp->m_readBuffMutex.lock();
            // search for open entry
            NATIVE_INT_TYPE entryFound = false;

            NATIVE_INT_TYPE entry = 0;
            for (entry = 0; entry < HR_RECEIVE_BUFFER_POOL_SIZE; entry++) {
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

            timespec stime;
            (void)clock_gettime(CLOCK_REALTIME,&stime);
            DEBUG_PRINT("<<< Calling rpc_relay_buff_read() at %d %d\n",
                        stime.tv_sec, stime.tv_nsec);

            bool waiting = true;
            int stat = 0;

            while (waiting) {
                // TODO(mereweth) - faster copy with mapped shared region?
                stat = rpc_relay_buff_read(&portNum,
                       reinterpret_cast<unsigned char*>(buff.getdata()),
                       buff.getsize(), &sizeRead);

                // TODO(mereweth) - add KraitRouter timeout return code and
                // check for timeout
                if (1 == stat) {
                    if (comp->m_quitReadThreads) {
                        return;
                    }
                }
                else { // quit if other error or if data received
                    waiting = false;
                }
            }

            if (comp->m_quitReadThreads) {
                return;
            }

            if (stat != 0) {
                timespec stime;
                (void)clock_gettime(CLOCK_REALTIME,&stime);
                DEBUG_PRINT("rpc_relay_buff_read error stat %d at %d:%d\n",
                            stat, stime.tv_sec, stime.tv_nsec);
                // TODO(mereweth) send buffer to destination with error flag
                // instead of recycling?
                comp->m_readBuffMutex.lock();
                comp->m_buffSet[entry].available = true;
                comp->m_readBuffMutex.unLock();
            }
            else {
                // (void)clock_gettime(CLOCK_REALTIME,&stime);
                // printf("<!<! Sending data to port %u size %u at %d %d\n", portNum, buff.getsize(), stime.tv_sec, stime.tv_nsec);
                buff.setsize(sizeRead);
                if (comp->isConnected_readBufferSend_OutputPort(portNum)) {
                    comp->readBufferSend_out(portNum, buff);
                }
                else {
                    comp->log_WARNING_HI_HR_InvalidPortNum(HR_BUFF_SEND,
                                                           portNum);
                    comp->m_readBuffMutex.lock();
                    comp->m_buffSet[entry].available = true;
                    comp->m_readBuffMutex.unLock();
                }
            }
        }
    }

    void HexRouterComponentImpl::quitReadThreads(void) {
        this->m_quitReadThreads = true;
        this->m_portReadTask.join(NULL);
        DEBUG_PRINT("HexRouter quitReadThreads after portRead join\n");
        this->m_buffReadTask.join(NULL);
        DEBUG_PRINT("HexRouter quitReadThreads after buffRead join\n");
        //rpc_relay_quit();
    }
} // end namespace SnapdragonFlight
