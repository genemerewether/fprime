// ====================================================================== 
// \title  ZmqPubImpl.hpp
// \author tcanham
// \brief  hpp file for ZmqPub component implementation class
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

#ifndef ZmqPub_HPP
#define ZmqPub_HPP

#include "fprime-zmq/zmq-pub/ZmqPubComponentAc.hpp"
#include <fprime-zmq/zmq-pub/ZmqPubComponentImplCfg.hpp>
#include <fprime-zmq/zmq/include/zmq.h>

namespace Zmq {

  class ZmqPubComponentImpl :
    public ZmqPubComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ZmqPub
      //!
      ZmqPubComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ZmqPub
      //!
      void init(
          NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          NATIVE_INT_TYPE msgSize, /*!< The message size*/
          NATIVE_INT_TYPE instance /*!< The instance number*/
      );
      //! Open the connection
      //!
      void open(
              const char* port /*!< port for connection, client or server */
              );

      //! Destroy object ZmqPub
      //!
      ~ZmqPubComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Sched
      //!
      void Sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for PortsIn
      //!
      void PortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );

      //! Preamble override to set up zmq message queue
      void preamble(void);
      //! Finalizer override to clean up zmq message queue
      void finalizer(void);

      class ZmqSerialBuffer :
        public Fw::SerializeBufferBase
      {

        public:

#ifdef BUILD_UT
          void operator=(const ZmqSerialBuffer& other);
          ZmqSerialBuffer(const Fw::SerializeBufferBase& other);
          ZmqSerialBuffer(const ZmqSerialBuffer& other);
          ZmqSerialBuffer();
#endif

          NATIVE_UINT_TYPE getBuffCapacity(void) const {
            return sizeof(m_buff);
          }

          // Get the max number of bytes that can be serialized
          NATIVE_UINT_TYPE getBuffSerLeft(void) const {

            const NATIVE_UINT_TYPE size  = getBuffCapacity();
            const NATIVE_UINT_TYPE loc = getBuffLength();

            if (loc >= (size-1) ) {
                return 0;
            }
            else {
                return (size - loc - 1);
            }
          }

          U8* getBuffAddr(void) {
            return m_buff;
          }

          const U8* getBuffAddr(void) const {
            return m_buff;
          }

        private:
          // Should be the max of all the input ports serialized sizes...
          U8 m_buff[ZMQ_PUB_MSG_SIZE];

      };

      ZmqSerialBuffer m_netBuffer; // Buffer to decode port call in network buffer

      //! decode network packet
      void decodePacket(U8* packet, NATIVE_UINT_TYPE size);

      //! zmq error helper - quit loop if true
      bool zmqError(const char* from);

      // Initialization variables

      char m_port[ZMQ_PUB_ENDPOINT_NAME_SIZE]; /*!< port for server connection */

      // zmq variables

      //  Prepare our context and socket

      void *m_context; //!< zmq context
      void *m_pubSocket; //!< zmp socket for outbound buffers to poll task

      U32 m_packetsSent; //!< number of packets sent

      // buffer used to send port data
      ZmqSerialBuffer m_sendBuff;



    };

} // end namespace Zmq

#endif
