// ====================================================================== 
// \title  ZmqSubImpl.cpp
// \author tcanham
// \brief  cpp file for ZmqSub component implementation class
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


#include <fprime-zmq/zmq-sub/ZmqSubComponentImpl.hpp>
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

  ZmqSubComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ZmqSubComponentImpl(
        const char *const compName
    ) :
      ZmqSubComponentBase(compName)
#else
    ZmqSubImpl(void)
#endif
    ,m_context(0)
  {

  }

  void ZmqSubComponentImpl ::
    init(
      NATIVE_INT_TYPE instance /*!< The instance number*/
   ) {
    ZmqSubComponentBase::init(instance);
  }

  ZmqSubComponentImpl ::
    ~ZmqSubComponentImpl(void)
  {

      // clean up zmq state
      if (this->m_context) {
          zmq_ctx_destroy (this->m_context);
      }
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ZmqSubComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    this->tlmWrite_ZS_PacketsReceived(this->m_packetsReceived);
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void ZmqSubComponentImpl::open(
          const char* addr,  /*!< if client, the server address, not used for server */
          const char* port, /*!< port for connection, client or server */
          NATIVE_UINT_TYPE priority, /*!< read task priority */
          NATIVE_UINT_TYPE stackSize, /*!< stack size */
          NATIVE_UINT_TYPE affinity /*!< cpu affinity */
          ) {

      // store values for worker thread
      strncpy(this->m_addr,addr,ZMQ_SUB_ENDPOINT_NAME_SIZE);
      this->m_addr[ZMQ_SUB_ENDPOINT_NAME_SIZE-1] = 0;
      strncpy(this->m_port,port,ZMQ_SUB_ENDPOINT_NAME_SIZE);
      this->m_port[ZMQ_SUB_ENDPOINT_NAME_SIZE-1] = 0;

      // create zmq context
      this->m_context = zmq_ctx_new ();
      if (not this->m_context) {
          Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
          this->log_WARNING_HI_ZS_ContextError(errArg);
      }

      // start task
      Fw::EightyCharString taskName;
      taskName.format("zmq_pub%d",this->getInstance());
      Os::Task::TaskStatus tstat = this->m_socketTask.start(taskName,0,priority,stackSize,this->workerTask,this,affinity);
      FW_ASSERT(Os::Task::TASK_OK == tstat,tstat);

  }

  void ZmqSubComponentImpl::workerTask(void* ptr) {

      // quit if the context wasn't created

      DEBUG_PRINT("Worker task started\n");
      FW_ASSERT(ptr);
      ZmqSubComponentImpl* compPtr = static_cast<ZmqSubComponentImpl*>(ptr);

      if (not compPtr->m_context) {
          return;
      }
      // create network socket depending on whether we are client or server

      DEBUG_PRINT("Creating ZMQ SUB socket\n");
      int stat;
      void* networkSocket = 0;
      networkSocket = zmq_socket (compPtr->m_context, ZMQ_SUB);
      if (not networkSocket) {
          Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
          compPtr->log_WARNING_HI_ZS_SocketError(errArg);
          return;
      }

      char endpoint[ZMQ_SUB_ENDPOINT_NAME_SIZE];
      (void)snprintf(endpoint,ZMQ_SUB_ENDPOINT_NAME_SIZE,"tcp://%s:%s",compPtr->m_addr,compPtr->m_port);
      // null terminate
      endpoint[ZMQ_SUB_ENDPOINT_NAME_SIZE-1] = 0;

      DEBUG_PRINT("Connecting to server %s port %s (%s)\n",compPtr->m_addr,compPtr->m_port,endpoint);
      stat = zmq_connect(networkSocket, endpoint);
      if (-1 == stat) {
          Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
          compPtr->log_WARNING_HI_ZS_ConnectError(errArg);
          return;
      }

      // subscribe to "ZeroMQ Port" packets
      const char filter[] = "ZP";
      stat = zmq_setsockopt (networkSocket, ZMQ_SUBSCRIBE,
                           filter, strlen(filter));
      if (-1 == stat) {
          Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
          compPtr->log_WARNING_HI_ZS_SockOptsError(errArg);
          return;
      } else {
          compPtr->log_ACTIVITY_HI_ZS_SubscribeConnectionOpened();
      }

      // wait on network or local requests
      char msg[ZMQ_SUB_MSG_SIZE];
      while (true) {
          int size = zmq_recv (networkSocket, msg, ZMQ_SUB_MSG_SIZE, 0);
          if (size != -1) {
              // decode packet into port call
              DEBUG_PRINT("Received network packet\n");
              compPtr->decodePacket((U8*)msg,size);
              compPtr->m_packetsReceived++;
              continue;
          } else {
              if (compPtr->zmqError("network zmq_recv")) {
                  Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
                  compPtr->log_WARNING_HI_ZS_ReceiveError(errArg);
                  break;
              } else {
                  continue;
              }
          }
      }

      zmq_close(networkSocket);

  }

  void ZmqSubComponentImpl::decodePacket(U8* packet, NATIVE_UINT_TYPE size) {

      FW_ASSERT(packet);
      FW_ASSERT(size < ZMQ_SUB_MSG_SIZE,size);
      Fw::SerializeStatus stat;
      Fw::ExternalSerializeBuffer buff(packet,size);
      buff.setBuffLen(size);
      buff.resetDeser();

      DEBUG_PRINT("Decoding packet of size %d\n",size);

      // deserialize subscription header
      const char subHdr[] = "ZP";
      U8 subHdrRd[sizeof(subHdr)];
      NATIVE_UINT_TYPE len = strlen(subHdr);
      stat = buff.deserialize(subHdrRd,len,true);
      if (stat != Fw::FW_SERIALIZE_OK) {
          DEBUG_PRINT("Decode sub header error %d %d %d\n",stat,buff.getBuffLength(),buff.getBuffLeft());
          //this->tlmWrite_FR_NumDecodeErrors(++this->m_numDecodeErrors);
          return;
      }

      // get port number
      U8 portNum;
      stat = buff.deserialize(portNum);
      // check for deserialization error or port number too high
      if (stat != Fw::FW_SERIALIZE_OK or portNum > this->getNum_PortsOut_OutputPorts()) {
          DEBUG_PRINT("Decode portnum error %d %d\n",stat,this->getNum_PortsOut_OutputPorts());
          //this->tlmWrite_FR_NumDecodeErrors(++this->m_numDecodeErrors);
          return;
      }
      // get buffer for port

      stat = buff.deserialize(this->m_netBuffer);
      if (stat != Fw::FW_SERIALIZE_OK) {
          DEBUG_PRINT("Decode data error\n");
          //this->tlmWrite_FR_NumDecodeErrors(++this->m_numDecodeErrors);
          return;
      }
      // call output port
      DEBUG_PRINT("Calling port %d with %d bytes.\n",portNum,this->m_netBuffer.getBuffLength());
      if (this->isConnected_PortsOut_OutputPort(portNum)) {

          Fw::SerializeStatus stat = this->PortsOut_out(portNum,this->m_netBuffer);

          // If had issues deserializing the data, then report it:
          if (stat != Fw::FW_SERIALIZE_OK) {

              DEBUG_PRINT("PortsOut_out() serialize status error\n");
              //this->tlmWrite_FR_NumBadSerialPortCalls(++this->m_numBadSerialPortCalls);
              //this->log_WARNING_HI_FR_BadSerialPortCall(stat,portNum);
              return;
          }

      }
      else {
      }

  }

  bool ZmqSubComponentImpl::zmqError(const char* from) {
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
  void ZmqSubComponentImpl::ZmqSerialBuffer::operator=(const ZmqSerialBuffer& other) {
      this->resetSer();
      this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
  }

  ZmqSubComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(
          const Fw::SerializeBufferBase& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.getBuffAddr(),other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  ZmqSubComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(
          const ZmqSubComponentImpl::ZmqSerialBuffer& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.m_buff,other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  ZmqSubComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(): Fw::SerializeBufferBase() {

  }

#endif


} // end namespace Zmq
