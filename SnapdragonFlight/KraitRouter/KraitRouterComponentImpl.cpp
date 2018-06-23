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
        m_tempBuff(),
        m_maxReadPerDispatch(0u),
        m_initialized(KR_STATE_PREINIT)
    {

    }

    void KraitRouterComponentImpl ::
      init(
          const NATIVE_INT_TYPE queueDepth,
          const NATIVE_INT_TYPE msgSize,
          const NATIVE_INT_TYPE instance
      )
    {
        KraitRouterComponentBase::init(queueDepth, msgSize, instance);

        m_maxReadPerDispatch = msgSize + 2 * sizeof(U32);
        m_initialized = KR_STATE_INIT;
    }

    KraitRouterComponentImpl ::
      ~KraitRouterComponentImpl(void)
    {
        m_initialized = KR_STATE_QUIT;
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
        this->m_tempBuff.setExtBuffer(buff,
                                      static_cast<unsigned int>(buffLen));
        this->m_tempBuff.resetSer();

        Fw::QueuedComponentBase::MsgDispatchStatus msgStat = Fw::QueuedComponentBase::MSG_DISPATCH_OK;
        // dequeue any pending messages
        DEBUG_PRINT("portRead object 0x%X, init %d, pre-loop buffLeft %d, msgStat %d\n",
                    (unsigned long) this, this->m_initialized,
                    this->m_tempBuff.getBuffCapacity() - this->m_tempBuff.getBuffLength(), msgStat);
        while ((msgStat != Fw::QueuedComponentBase::MSG_DISPATCH_EMPTY) &&
               ((this->m_tempBuff.getBuffCapacity() - this->m_tempBuff.getBuffLength())
                >= this->m_maxReadPerDispatch)) {
            DEBUG_PRINT("portRead object 0x%X, init %d, buffLeft %d, msgStat %d\n",
                        (unsigned long) this, this->m_initialized,
                        this->m_tempBuff.getBuffCapacity() - this->m_tempBuff.getBuffLength(), msgStat);
            msgStat = this->doDispatch();
            if (msgStat == Fw::QueuedComponentBase::MSG_DISPATCH_EXIT) {
                return KR_RTN_QUIT;
            }
        }

        *bytes = this->m_tempBuff.getBuffLength();

        DEBUG_PRINT("portRead object 0x%X, init %d, buffLeft %d, msgStat %d, returning %d bytes\n",
                    (unsigned long) this, this->m_initialized,
                    this->m_tempBuff.getBuffCapacity() - this->m_tempBuff.getBuffLength(), msgStat,
                    *bytes);

        return KR_RTN_OK;
    }


    int KraitRouterComponentImpl::buffWrite(unsigned int port,
                                            const unsigned char* buff,
                                            int buffLen) {
        FW_ASSERT(0); // TODO(mereweth)
        return 0;
    }

    int KraitRouterComponentImpl::portWrite(unsigned int port,
                                            const unsigned char* buff,
                                            int buffLen) {
        DEBUG_PRINT("write called on object 0x%X, port %d, init %d\n",
                    (unsigned long) this, port, this->m_initialized);
        FW_ASSERT(0); // TODO(mereweth) - fill write queue
        /*while (!this->m_initialized) {
            if (this->m_quit) {
                DEBUG_PRINT("quitting write preinit in object 0x%X\n",
                            (unsigned long) this);
                return -10;
            }

            usleep(KR_PREINIT_SLEEP_US);
        }

        if (!this->m_sendPortBuffers[m_sendPortBuffInsert].available) {
            DEBUG_PRINT("FastRPC write tried to overwrite a port buffer at %d for port %d\n",
                        m_sendPortBuffInsert, port);
            //this->log_WARNING_HI_KR_BadSerialPortCall(stat, port);
            //this->tlmWrite_KR_NumBadSerialPortCalls();
            return -5;
        }

        NATIVE_UINT_TYPE sendPortBuffInsert = m_sendPortBuffInsert;

        m_sendPortBuffers[sendPortBuffInsert].available = false;
        m_sendPortBuffers[sendPortBuffInsert].portNum = port;
*/
        /* NOTE(mereweth) - this memory will be copied again in the Sched call
         * for the output port invocation. This should be sufficiently fast for
         * these port calls.
         */
        /*unsigned int sendBuffSize =
           FW_NUM_ARRAY_ELEMENTS(m_sendPortBuffers[sendPortBuffInsert].buff);
        if (buffLen >= sendBuffSize) {
            DEBUG_PRINT("FastRPC write data %d too big: actual %d, avail %d\n",
                        port, buffLen, sendBuffSize);
            //this->log_WARNING_HI_KR_BadSerialPortCall(stat, port);
            //this->tlmWrite_KR_NumBadSerialPortCalls();
            return -3;
        }
        memcpy(m_sendPortBuffers[sendPortBuffInsert].buff,
               buff, buffLen);
        m_sendPortBuffers[sendPortBuffInsert].buffLen = buffLen;

        m_sendPortBuffInsert++;
        if (m_sendPortBuffInsert >= FW_NUM_ARRAY_ELEMENTS(m_sendPortBuffers)) {
            m_sendPortBuffInsert = 0;
        }*/

        return 0;
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
        return; // TODO(mereweth) - fill write queue
        /*
            // if connected, call output port
            if (this->isConnected_KraitPortsOut_OutputPort(port)) {
                Fw::ExternalSerializeBuffer portBuff(buff, buffLen);
                portBuff.setBuffLen(buffLen);

                DEBUG_PRINT("Calling port %d from buf %d with %d bytes.\n",
                            port, m_sendPortBuffRemove, buffLen);
                Fw::SerializeStatus stat = this->KraitPortsOut_out(port, portBuff);
                if (stat != Fw::FW_SERIALIZE_OK) {
                    DEBUG_PRINT("KraitPortsOut_out() serialize status error\n");
                    //this->log_WARNING_HI_KR_BadSerialPortCall(stat, port);
                    //this->tlmWrite_HR_NumBadSerialPortCalls(++this->m_numBadSerialPortCalls);
                    // TODO(mereweth) - status codes
                }
            }
        }*/
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
        // NOTE(mereweth) - called from FastRPC portRead thread
        DEBUG_PRINT("HexPortsIn_handler for port %d with %d bytes\n",
                    portNum, Buffer.getBuffLength());

        Fw::SerializeStatus stat = this->m_tempBuff.serialize(portNum);
        FW_ASSERT(stat == Fw::FW_SERIALIZE_OK);
        // serializes length and copies contents
        stat = this->m_tempBuff.serialize(Buffer);
        FW_ASSERT(stat == Fw::FW_SERIALIZE_OK);

        DEBUG_PRINT("HexPortsIn_handler done port %d with %d bytes\n",
                    portNum, Buffer.getBuffLength());
    }

} // end namespace SnapdragonFlight
