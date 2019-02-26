// ====================================================================== 
// \title  ZmqAdapterImpl.hpp
// \author tcanham
// \brief  hpp file for ZmqAdapter component implementation class
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

#ifndef ZmqAdapter_HPP
#define ZmqAdapter_HPP

#include "fprime-zmq/zmq-adapter/ZmqAdapterComponentAc.hpp"
#include <fprime-zmq/zmq-adapter/ZmqAdapterComponentImplCfg.hpp>
#include <fprime-zmq/zmq/include/zmq.h>

namespace Zmq {

  class ZmqAdapterComponentImpl :
    public ZmqAdapterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ZmqAdapter
      //!
      ZmqAdapterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ZmqAdapter
      //!
      void init(
          NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          NATIVE_INT_TYPE msgSize, /*!< The message size*/
          NATIVE_INT_TYPE instance /*!< The instance number*/
      );
      //! Open the connection
      //!
      void open(
              bool server,  /*!< if this node is the server */
              const char* addr,  /*!< if client, the server address, not used for server */
              const char* port, /*!< port for connection, client or server */
              NATIVE_UINT_TYPE priority, /*!< read task priority */
              NATIVE_UINT_TYPE stackSize, /*!< stack size */
              NATIVE_UINT_TYPE affinity /*!< cpu affinity */
              );

      //! Destroy object ZmqAdapter
      //!
      ~ZmqAdapterComponentImpl(void);

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
          U8 m_buff[ZMQ_ADAPTER_MSG_SIZE];

      };

      ZmqSerialBuffer m_netBuffer; // Buffer to decode port call in network buffer
      ZmqSerialBuffer m_ipcBuffer; // Buffer to create port call buffer to send on network

      //! decode network packet
      void decodePacket(U8* packet, NATIVE_UINT_TYPE size);

      //! zmq error helper - quit loop if true
      bool zmqError(const char* from);

      // Initialization variables

      bool m_server;  /*!< if this node is the server */
      char m_addr[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE];  /*!< if client, the server address, not used for server */
      char m_port[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE]; /*!< port for connection, client or server */

      // zmq variables

      //  Prepare our context and socket

      void *m_context; //!< zmq context
      void *m_ipcSocket; //!< zmp socket for outbound buffers to poll task

      static void workerTask(void* ptr); //!< worker task entry point
      Os::Task m_socketTask;

    };

} // end namespace Zmq

#endif
