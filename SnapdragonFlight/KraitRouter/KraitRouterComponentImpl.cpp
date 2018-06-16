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

//#undef DEBUG_PRINT
//#define DEBUG_PRINT(x,...)

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
        m_quit(false),
        m_recvPortBuffers(),
        m_recvPortBuffInsert(0),
        m_recvPortBuffRemove(0),
        m_sendPortBuffers(),
        m_sendPortBuffInsert(0),
        m_sendPortBuffRemove(0),
        m_initialized(false)
    {
        for (int i = 0; i < KR_NUM_RECV_PORT_BUFFS; i++) {
            m_recvPortBuffers[i].available = true;
        }
        for (int i = 0; i < KR_NUM_SEND_PORT_BUFFS; i++) {
            m_sendPortBuffers[i].available = true;
        }
    }

    void KraitRouterComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
        KraitRouterComponentBase::init(instance);
        m_initialized = true;
    }

    KraitRouterComponentImpl ::
      ~KraitRouterComponentImpl(void)
    {
        m_initialized = false;
    }

    int KraitRouterComponentImpl::buffRead(unsigned int* port,
                                           unsigned char* buff,
                                           int buffLen,
                                           int* bytes) {
        DEBUG_PRINT("buffRead called on object 0x%X, init %d\n",
                    (unsigned long) this, this->m_initialized);
        while (!this->m_initialized) {
            if (this->m_quit) {
                return -10;
            }

            usleep(KR_PREINIT_SLEEP_US);
        }
        *port = 0;
        *bytes = 0;
        return 1;
    }

    int KraitRouterComponentImpl::portRead(unsigned int* port,
                                           unsigned char* buff,
                                           int buffLen,
                                           int* bytes) {
        DEBUG_PRINT("portRead called on object 0x%X, init %d\n",
                    (unsigned long) this, this->m_initialized);
        while (!this->m_initialized) {
            if (this->m_quit) {
                DEBUG_PRINT("quitting portRead preinit in object 0x%X\n",
                            (unsigned long) this);
                return -10;
            }

            usleep(KR_PREINIT_SLEEP_US);
        }

        /* TODO(mereweth) - handle concurrency issue here if we ever invoke
         * this from multiple contexts
         */

        /* NOTE(mereweth) - if portBuffer entry is available, that means no data
         * is stored there. If m_recvPortBuffInsert and m_recvPortBuffRemove are
         * equal, we are all caught up and waiting for a new input port call.
         */
        while (this->m_recvPortBuffers[m_recvPortBuffRemove].available == true ||
               m_recvPortBuffInsert == m_recvPortBuffRemove) {
            DEBUG_PRINT("waiting for portBuff at %d in object 0x%X\n",
                        m_recvPortBuffRemove, (unsigned long) this);
            if (this->m_quit) {
                DEBUG_PRINT("quitting portRead postinit in object 0x%X\n",
                            (unsigned long) this);
                return -10;
            }
            usleep(KR_NOPORT_SLEEP_US);
        }

        if (buffLen < m_recvPortBuffers[m_recvPortBuffRemove].buffLen) {
            return -1;
            /* TODO(mereweth) - error - FastRPC call didn't provide enough space
             * to copy in port
             */
        }
        *port = m_recvPortBuffers[m_recvPortBuffRemove].portNum;
        DEBUG_PRINT("before memcpy of buff idx %d in portRead in object 0x%X, port %d\n",
                    m_recvPortBuffRemove, (unsigned long) this, *port);
        memcpy(buff, m_recvPortBuffers[m_recvPortBuffRemove].buff,
               m_recvPortBuffers[m_recvPortBuffRemove].buffLen);
        *bytes = m_recvPortBuffers[m_recvPortBuffRemove].buffLen;

        // was false, meaning had data. set to available so it can be filled again
        m_recvPortBuffers[m_recvPortBuffRemove].available = true;

        m_recvPortBuffRemove++;
        if (m_recvPortBuffRemove >= FW_NUM_ARRAY_ELEMENTS(m_recvPortBuffers)) {
            m_recvPortBuffRemove = 0;
        }

        /* TODO(mereweth) - handle concurrency issue here if we ever invoke
         * this from multiple contexts
         */

        return 0;
    }

    int KraitRouterComponentImpl::write(unsigned int port,
                                        const unsigned char* buff,
                                        int buffLen) {
        DEBUG_PRINT("write called on object 0x%X, port %d, init %d\n",
                    (unsigned long) this, port, this->m_initialized);
        while (!this->m_initialized) {
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

        /* NOTE(mereweth) - this memory will be copied again in the Sched call
         * for the output port invocation. This should be sufficiently fast for
         * these port calls.
         */
        unsigned int sendBuffSize =
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
        }

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
        // until writeRemove catches up to writeInsert
        while (!this->m_sendPortBuffers[m_sendPortBuffRemove].available &&
               m_sendPortBuffRemove != m_sendPortBuffInsert) {
            unsigned int port = m_sendPortBuffers[m_sendPortBuffRemove].portNum;
            unsigned char* buff = m_sendPortBuffers[m_sendPortBuffRemove].buff;
            unsigned int buffLen = m_sendPortBuffers[m_sendPortBuffRemove].buffLen;
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

            m_sendPortBuffers[m_sendPortBuffRemove].available = true;

            m_sendPortBuffRemove++;
            if (m_sendPortBuffRemove >= FW_NUM_ARRAY_ELEMENTS(m_sendPortBuffers)) {
                m_sendPortBuffRemove = 0;
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
        /* NOTE(mereweth) - single (thread of) producer, so we don't lock here
         * All the Hexagon QUEST code runs in a single QuRT thread, using PassiveRateGroup
         */

        DEBUG_PRINT("HexPortsIn_handler for port %d with %d bytes\n",
                    portNum, Buffer.getBuffLength());

        if (Buffer.getBuffLength() == 0) {
            DEBUG_PRINT("HexPortsIn_handler serialized port %d empty\n",
                        portNum);
            //this->log_WARNING_HI_KR_BadSerialPortCall(stat, port);
            //this->tlmWrite_KR_NumBadSerialPortCalls();
            return;
        }

        if (!this->m_recvPortBuffers[m_recvPortBuffInsert].available) {
            DEBUG_PRINT("HexPortsIn_handler tried to overwrite a port buffer at %d for port %d\n",
                        m_recvPortBuffInsert, portNum);
            //this->log_WARNING_HI_KR_BadSerialPortCall(stat, port);
            //this->tlmWrite_KR_NumBadSerialPortCalls();
            return;
        }

        m_recvPortBuffers[m_recvPortBuffInsert].available = false;
        m_recvPortBuffers[m_recvPortBuffInsert].portNum = portNum;

        /* NOTE(mereweth) - this memory gets copied again into the buffer
         * provided by FastRPC read call. This should be sufficiently fast for
         * these port calls
         */
        unsigned int recvBuffSize =
            FW_NUM_ARRAY_ELEMENTS(m_recvPortBuffers[m_recvPortBuffInsert].buff);
        if (Buffer.getBuffLength() >= recvBuffSize) {
            DEBUG_PRINT("HexPortsIn_handler serialized port %d too big: actual %d, avail %d\n",
                        portNum, Buffer.getBuffLength(), recvBuffSize);
            //this->log_WARNING_HI_KR_BadSerialPortCall(stat, port);
            //this->tlmWrite_KR_NumBadSerialPortCalls();
            m_recvPortBuffers[m_recvPortBuffInsert].available = true;
            return;
        }
        memcpy(m_recvPortBuffers[m_recvPortBuffInsert].buff,
               Buffer.getBuffAddr(), Buffer.getBuffLength());
        m_recvPortBuffers[m_recvPortBuffInsert].buffLen = Buffer.getBuffLength();

        m_recvPortBuffInsert++;
        if (m_recvPortBuffInsert >= FW_NUM_ARRAY_ELEMENTS(m_recvPortBuffers)) {
            m_recvPortBuffInsert = 0;
        }
    }

} // end namespace SnapdragonFlight
