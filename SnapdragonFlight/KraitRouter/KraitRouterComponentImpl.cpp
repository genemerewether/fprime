// ======================================================================
// \title  KraitRouterImpl.cpp
// \author vagrant
// \brief  cpp file for KraitRouter component implementation class
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


#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "Fw/Types/SerialBuffer.hpp"
#include <unistd.h>

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

#undef DEBUG_PRINT
#define DEBUG_PRINT(x,...)

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    KraitRouterComponentImpl ::
      KraitRouterComponentImpl(
    #if FW_OBJECT_NAMES == 1
          const char *const compName
    #endif
      ) :
        KraitRouterComponentBase(
    #if FW_OBJECT_NAMES == 1
                                 compName
    #endif
                                 ),
        m_localMsgSize(0u),
        m_initialized(KR_STATE_PREINIT),
        m_recvQueue(),
        m_sendQueue()
    {

    }

    void KraitRouterComponentImpl ::
      init(
          const NATIVE_INT_TYPE queueDepth,
          const NATIVE_INT_TYPE msgSize,
          const NATIVE_INT_TYPE instance
      )
    {
        KraitRouterComponentBase::init(instance);

        m_localMsgSize = msgSize + sizeof(U32);

        Fw::EightyCharString queueName;
#if FW_OBJECT_NAMES == 1
        queueName = this->m_objName;
#else
        char queueNameChar[FW_QUEUE_NAME_MAX_SIZE];
        (void)snprintf(queueNameChar,sizeof(queueNameChar),"CompQ_%d",Os::Queue::getNumQueues());
        queueName = queueNameChar;
#endif
        m_recvQueue.create(queueName, queueDepth, m_localMsgSize);
        m_sendQueue.create(queueName, queueDepth, m_localMsgSize);

        m_initialized = KR_STATE_INIT;
    }

    void KraitRouterComponentImpl ::
      quit(void)
    {
        m_initialized = KR_STATE_QUIT;
    }
  
    KraitRouterComponentImpl ::
      ~KraitRouterComponentImpl(void)
    {
    }

    int KraitRouterComponentImpl::buffRead(unsigned int* port,
                                           unsigned char* buff,
                                           int buffLen,
                                           int* bytes) {
        FW_ASSERT(0); // TODO(mereweth)
        DEBUG_PRINT("buffRead called on object 0x%X, init %d\n",
                    (unsigned long) this, this->m_initialized);
        while (this->m_initialized == KR_STATE_PREINIT) {
            // queue isn't initialized yet so we have no choice but to sleep.
            usleep(KR_PREINIT_SLEEP_US);
        }
        // TODO(mereweth) - add mechanism for force quit
        if (this->m_initialized == KR_STATE_QUIT_PREINIT) {
            return KR_RTN_QUIT_PREINIT;
        }
        *port = 0;
        *bytes = 0;
        return 1;
    }

    int KraitRouterComponentImpl::portRead(unsigned char* buff,
                                           int buffLen,
                                           int* bytes) {
        DEBUG_PRINT("portRead called on object 0x%X, init %d\n",
                    (unsigned long) this, this->m_initialized);
        while (this->m_initialized == KR_STATE_PREINIT) {
            // queue isn't initialized yet so we have no choice but to sleep.
            usleep(KR_PREINIT_SLEEP_US);
        }
        // TODO(mereweth) - add mechanism for force quit
        if (this->m_initialized == KR_STATE_QUIT_PREINIT) {
            return KR_RTN_QUIT_PREINIT;
        }

        if (buffLen < 0) {
            return KR_RTN_FASTRPC_FAIL;
        }

        Fw::ExternalSerializeBuffer msg(buff, buffLen);
        NATIVE_INT_TYPE priority;

        // NOTE(mereweth) - BLOCKING!!!
        while ((msg.getBuffCapacity() - msg.getBuffLength())
               >= this->m_localMsgSize) {
            if (KR_STATE_QUIT == this->m_initialized) {
                return KR_RTN_QUIT;
            }
            // TODO(mereweth) - probably should push the quit code through the queue
            
            Os::Queue::QueueStatus msgStatus = this->m_recvQueue.receive(msg,
                                            priority, Os::Queue::QUEUE_BLOCKING);
            DEBUG_PRINT("portRead object 0x%X, init %d, buffLeft %d\n",
                        (unsigned long) this, this->m_initialized,
                        msg.getBuffCapacity() - msg.getBuffLength());
            FW_ASSERT(
                msgStatus == Os::Queue::QUEUE_OK,
                static_cast<AssertArg>(msgStatus)
            );
        }

        *bytes = msg.getBuffLength();

        DEBUG_PRINT("portRead object 0x%X, init %d, buffLeft %d, returning %d bytes\n",
                    (unsigned long) this, this->m_initialized,
                    msg.getBuffCapacity() - msg.getBuffLength(),
                    *bytes);

        return KR_RTN_OK;
    }


    int KraitRouterComponentImpl::buffWrite(unsigned int port,
                                            const unsigned char* buff,
                                            int buffLen) {
        FW_ASSERT(0); // TODO(mereweth)
        return 0;
    }

    int KraitRouterComponentImpl::portWrite(const unsigned char* buff,
                                            int buffLen) {
        DEBUG_PRINT("write called on object 0x%X, init %d\n",
                    (unsigned long) this, this->m_initialized);

        if (buffLen > this->m_localMsgSize) {
            return KR_RTN_SEND_TOO_BIG;
        }
        FW_ASSERT(buffLen <= this->m_localMsgSize, buffLen, this->m_localMsgSize);

        Os::Queue::QueueStatus qStatus =
          this->m_sendQueue.send(buff, buffLen, 0, Os::Queue::QUEUE_NONBLOCKING);
        if (Os::Queue::QUEUE_FULL == qStatus) {
            return KR_RTN_SEND_FULL;
        }
        FW_ASSERT(
            qStatus == Os::Queue::QUEUE_OK,
            static_cast<AssertArg>(qStatus)
        );

        return KR_RTN_OK;
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void KraitRouterComponentImpl ::
      Sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      )
    {
        for (int i = 0; i < KR_SCHED_MAX_ITER; i++) {
            DEBUG_PRINT("sched iter %d called on object 0x%X, init %d\n",
                        i, (unsigned long) this, this->m_initialized);
            U8 msgBuff[this->m_localMsgSize];
            Fw::ExternalSerializeBuffer msg(msgBuff,this->m_localMsgSize);
            NATIVE_INT_TYPE priority;

            Os::Queue::QueueStatus msgStatus = this->m_sendQueue.receive(msg,
                                            priority, Os::Queue::QUEUE_NONBLOCKING);
            if (Os::Queue::QUEUE_NO_MORE_MSGS == msgStatus) {
              return;
            } else {
              FW_ASSERT(
                  msgStatus == Os::Queue::QUEUE_OK,
                  static_cast<AssertArg>(msgStatus)
              );
            }

            // Reset to beginning of buffer
            msg.resetDeser();

            NATIVE_INT_TYPE outPortNum;
            Fw::SerializeStatus deserStatus = msg.deserialize(outPortNum);
            FW_ASSERT(
                deserStatus == Fw::FW_SERIALIZE_OK,
                static_cast<AssertArg>(deserStatus)
            );
            // at this point, msg deserialization pointer points at length & buffer

            if (outPortNum >= this->getNum_KraitPortsOut_OutputPorts()) {
                DEBUG_PRINT("portNum %d too big\n", outPortNum, this->getNum_KraitPortsOut_OutputPorts());
                //this->tlmWrite_HR_NumDecodeErrors(++this->m_numDecodeErrors);
                continue;
            }

            // if connected, call output port
            if (this->isConnected_KraitPortsOut_OutputPort(outPortNum)) {
                // Deserialize serialized buffer into new buffer
                Fw::ExternalSerializeBuffer serHandBuff;
                deserStatus = msg.deserializeNoCopy(serHandBuff);
                FW_ASSERT(
                    deserStatus == Fw::FW_SERIALIZE_OK,
                    static_cast<AssertArg>(deserStatus)
                );

                DEBUG_PRINT("Calling port %d from KR sched\n", outPortNum);
                Fw::SerializeStatus stat = this->KraitPortsOut_out(outPortNum, serHandBuff);
                if (stat != Fw::FW_SERIALIZE_OK) {
                    DEBUG_PRINT("KraitPortsOut_out() serialize status error\n");
                    //this->log_WARNING_HI_KR_BadSerialPortCall(stat, port);
                    //this->tlmWrite_HR_NumBadSerialPortCalls(++this->m_numBadSerialPortCalls);
                    // TODO(mereweth) - status codes
                }
            }
        }
    }

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined serial input ports
    // ----------------------------------------------------------------------

    void KraitRouterComponentImpl ::
      HexPortsIn_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      )
    {
        // NOTE(mereweth) - called from DSP spawned thread; read from FastRPC thread
        DEBUG_PRINT("HexPortsIn_handler for port %d with %d bytes\n",
                    portNum, Buffer.getBuffLength());

        if (Buffer.getBuffLength() > this->m_localMsgSize) {
            DEBUG_PRINT("HexPort greater than max msg size\n");
            return;
        }

        U8 msgBuff[this->m_localMsgSize];
        Fw::ExternalSerializeBuffer msgSerBuff(msgBuff,this->m_localMsgSize);

        Fw::SerializeStatus serStat = msgSerBuff.serialize(portNum);
        FW_ASSERT(serStat == Fw::FW_SERIALIZE_OK, serStat);
        serStat = msgSerBuff.serialize(Buffer);
        FW_ASSERT(serStat == Fw::FW_SERIALIZE_OK, serStat);
        
        Os::Queue::QueueStatus qStatus =
          this->m_recvQueue.send(msgSerBuff, 0, Os::Queue::QUEUE_NONBLOCKING);
        if (qStatus == Os::Queue::QUEUE_FULL) {
            DEBUG_PRINT("hexports full, dropping\n");
            return;
        }
        FW_ASSERT(Os::Queue::QUEUE_OK == qStatus, qStatus);
    
        DEBUG_PRINT("HexPortsIn_handler done port %d with %d bytes\n",
                    portNum, Buffer.getBuffLength());
    }

} // end namespace SnapdragonFlight
