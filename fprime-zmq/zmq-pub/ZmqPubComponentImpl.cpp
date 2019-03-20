// ====================================================================== 
// \title  ZmqPubImpl.cpp
// \author tcanham
// \brief  cpp file for ZmqPub component implementation class
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


#include <fprime-zmq/zmq-pub/ZmqPubComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/EightyCharString.hpp>
#include <Os/Task.hpp>
#include <errno.h>
#include <string.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#define DEBUG_PRINT(x,...)

namespace Zmq {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ZmqPubComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ZmqPubComponentImpl(
        const char *const compName
    ) :
      ZmqPubComponentBase(compName)
#else
    ZmqPubImpl(void)
#endif
    ,m_context(0)
    ,m_pubSocket(0)
    ,m_packetsSent(0)
  {

  }

  void ZmqPubComponentImpl ::
    init(
      NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
      NATIVE_INT_TYPE msgSize, /*!< The message size*/
      NATIVE_INT_TYPE instance /*!< The instance number*/
   ) {
    ZmqPubComponentBase::init(queueDepth, msgSize, instance);
  }

  ZmqPubComponentImpl ::
    ~ZmqPubComponentImpl(void)
  {

      // clean up zmq state
      if (this->m_context) {
          zmq_ctx_destroy (this->m_context);
      }
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ZmqPubComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      // update channel
      this->tlmWrite_ZP_PacketsSent(this->m_packetsSent);
  }

  void ZmqPubComponentImpl::preamble(void) {

      // ZMQ requires that a socket be created, used, and destroyed on the same thread,
      // so we create it in the preamble. Only do it if the context was successfully created.
      if (this->m_context) {
          this->m_pubSocket = zmq_socket (this->m_context, ZMQ_PUB);
          if (not this->m_pubSocket) {
              Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
              this->log_WARNING_HI_ZP_SocketError(errArg);
              return;
          }

          // bind the server to the port
          char endpoint[ZMQ_PUB_ENDPOINT_NAME_SIZE];
          (void)snprintf(endpoint,ZMQ_PUB_ENDPOINT_NAME_SIZE,"tcp://*:%s",this->m_port);
          // null terminate
          endpoint[ZMQ_PUB_ENDPOINT_NAME_SIZE-1] = 0;

          NATIVE_INT_TYPE stat = zmq_bind(this->m_pubSocket,endpoint);
          if (-1 == stat) {
              Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
              this->log_WARNING_HI_ZP_BindError(errArg);
              return;
          } else {
              this->log_ACTIVITY_HI_ZP_PublishConnectionOpened();
          }
      }
  }

  void ZmqPubComponentImpl::finalizer(void) {

      // ZMQ requires that a socket be created, used, and destroyed on the same thread,
      // so we close it in the finalizer
      DEBUG_PRINT("Finalizing\n");
      if (this->m_pubSocket) {
          zmq_close(this->m_pubSocket);
      }
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void ZmqPubComponentImpl ::
    PortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      // return if we never successfully created the socket
      if (not this->m_pubSocket) {
          return;
      }

      DEBUG_PRINT("PortsIn_handler: %d\n",portNum);
      Fw::SerializeStatus stat;
      m_sendBuff.resetSer();

      // for ZMQ publish, need a subscription
      const U8 sub[] = "ZP"; // for "ZeroMQ Ports"
      stat = m_sendBuff.serialize(sub,strlen((const char*)sub),true);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

      // serialize port call
      stat = m_sendBuff.serialize(static_cast<U8>(portNum));
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      stat = m_sendBuff.serialize(Buffer);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      // send on zmq socket
      while (true) {
          DEBUG_PRINT("Sending %d bytes\n",m_sendBuff.getBuffLength());
          int zstat = zmq_send(this->m_pubSocket,m_sendBuff.getBuffAddr(),m_sendBuff.getBuffLength(),0);
          if (-1 == zstat) {
              if (this->zmqError("port zmq_send")) {
                  Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
                  this->log_WARNING_HI_ZP_SendError(errArg);
                  break;
              }
          } else {
              FW_ASSERT((int)m_sendBuff.getBuffLength() == zstat,zstat);
              this->m_packetsSent++;
              break;
          }
      }

  }

  void ZmqPubComponentImpl::open(
          const char* port /*!< port for connection, client or server */
          ) {

      // store values for worker thread
      strncpy(this->m_port,port,ZMQ_PUB_ENDPOINT_NAME_SIZE);
      this->m_port[ZMQ_PUB_ENDPOINT_NAME_SIZE-1] = 0;

      // create zmq context
      this->m_context = zmq_ctx_new ();
      if (not this->m_context) {
          Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
          this->log_WARNING_HI_ZP_ContextError(errArg);
      }

  }

  bool ZmqPubComponentImpl::zmqError(const char* from) {
      switch (zmq_errno()) {
          case ETERM:
              DEBUG_PRINT("%s: ZMQ terminate\n",from);
              return true;
          case EINTR:
              DEBUG_PRINT("%s: ZMQ EINTR\n",from);
              return false;
          default:
              DEBUG_PRINT("%s: ZMQ error: %s\n",from,zmq_strerror(zmq_errno()));
              return true;
      }
  }

#ifdef BUILD_UT
  void ZmqPubComponentImpl::ZmqSerialBuffer::operator=(const ZmqSerialBuffer& other) {
      this->resetSer();
      this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
  }

  ZmqPubComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(
          const Fw::SerializeBufferBase& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.getBuffAddr(),other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  ZmqPubComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(
          const ZmqPubComponentImpl::ZmqSerialBuffer& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.m_buff,other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  ZmqPubComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(): Fw::SerializeBufferBase() {

  }

#endif


} // end namespace Zmq
