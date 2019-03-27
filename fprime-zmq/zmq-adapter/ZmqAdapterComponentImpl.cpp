// ====================================================================== 
// \title  ZmqAdapterImpl.cpp
// \author tcanham
// \brief  cpp file for ZmqAdapter component implementation class
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


#include <fprime-zmq/zmq-adapter/ZmqAdapterComponentImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/EightyCharString.hpp>
#include <Os/Task.hpp>
#include <errno.h>
#include <string.h>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
//#define DEBUG_PRINT(x,...)

namespace Zmq {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ZmqAdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ZmqAdapterComponentImpl(
        const char *const compName
    ) :
      ZmqAdapterComponentBase(compName)
#else
    ZmqAdapterImpl(void)
#endif
    ,m_context(0)
  {

  }

  void ZmqAdapterComponentImpl ::
    init(
      NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
      NATIVE_INT_TYPE msgSize, /*!< The message size*/
      NATIVE_INT_TYPE instance /*!< The instance number*/
   ) {
    ZmqAdapterComponentBase::init(queueDepth, msgSize, instance);
  }

  ZmqAdapterComponentImpl ::
    ~ZmqAdapterComponentImpl(void)
  {

      // clean up zmq state
      zmq_ctx_destroy (this->m_context);
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ZmqAdapterComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void ZmqAdapterComponentImpl::preamble(void) {

      // ZMQ requires that a socket be created, used, and destroyed on the same thread,
      // so we create it in the preamble
      this->m_ipcSocket = zmq_socket (this->m_context, ZMQ_PAIR);
      char qname[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE];
      (void)snprintf(qname,ZMQ_ADAPTER_ENDPOINT_NAME_SIZE,"inproc://zmq_rtr_q%d",this->getInstance());
      qname[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE-1] = 0;
      zmq_bind (this->m_ipcSocket, qname);
  }

  void ZmqAdapterComponentImpl::finalizer(void) {

      // ZMQ requires that a socket be created, used, and destroyed on the same thread,
      // so we close it in the finalizer
      DEBUG_PRINT("Finalizing\n");
      zmq_close(this->m_ipcSocket);
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void ZmqAdapterComponentImpl ::
    PortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      DEBUG_PRINT("PortsIn_handler: %d\n",portNum);
      ZmqSerialBuffer buff;
      Fw::SerializeStatus stat;

      // serialize port call
      stat = buff.serialize(portNum);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      stat = buff.serialize(Buffer);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      // send on zmq socket
      bool done = false;
      while (not done) {
          int zstat = zmq_send(this->m_ipcSocket,buff.getBuffAddr(),buff.getBuffLength(),0);
          if (-1 == zstat) {
              if (this->zmqError("port zmq_send")) {
                  done = true;
              }
          } else {
              FW_ASSERT((int)buff.getBuffLength() == zstat,zstat);
              done = true;
          }
      }

  }

  void ZmqAdapterComponentImpl::open(
          bool server,  /*!< if this node is the server */
          const char* addr,  /*!< if client, the server address, not used for server */
          const char* port, /*!< port for connection, client or server */
          NATIVE_UINT_TYPE priority, /*!< read task priority */
          NATIVE_UINT_TYPE stackSize, /*!< stack size */
          NATIVE_UINT_TYPE affinity /*!< cpu affinity */
          ) {

      // store values for worker thread
      this->m_server = server;
      strncpy(this->m_addr,addr,ZMQ_ADAPTER_ENDPOINT_NAME_SIZE);
      this->m_addr[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE-1] = 0;
      strncpy(this->m_port,port,ZMQ_ADAPTER_ENDPOINT_NAME_SIZE);
      this->m_port[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE-1] = 0;

      // create zmq context
      this->m_context = zmq_ctx_new ();
      FW_ASSERT(this->m_context);

      // start task
      Fw::EightyCharString taskName;
      taskName.format("zmq_rtr%d",this->getInstance());
      Os::Task::TaskStatus tstat = this->m_socketTask.start(taskName,0,priority,stackSize,this->workerTask,this,affinity);
      FW_ASSERT(Os::Task::TASK_OK == tstat,tstat);

  }

  void ZmqAdapterComponentImpl::workerTask(void* ptr) {

      DEBUG_PRINT("Worker task started\n");
      FW_ASSERT(ptr);
      ZmqAdapterComponentImpl* compPtr = static_cast<ZmqAdapterComponentImpl*>(ptr);

      // create network socket depending on whether we are client or server

      int stat;
      void* networkSocket = 0;
      char endpoint[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE];
      if (compPtr->m_server) {
          networkSocket = zmq_socket (compPtr->m_context, ZMQ_REP);
          FW_ASSERT(networkSocket);
          (void)snprintf(endpoint,ZMQ_ADAPTER_ENDPOINT_NAME_SIZE,"tcp://*:%s",compPtr->m_port);
          // null terminate
          endpoint[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE-1] = 0;
          stat = zmq_bind(networkSocket, endpoint);
      } else {
          networkSocket = zmq_socket (compPtr->m_context, ZMQ_REQ);
          FW_ASSERT(networkSocket);
          (void)snprintf(endpoint,ZMQ_ADAPTER_ENDPOINT_NAME_SIZE,"tcp://%s:%s",compPtr->m_addr,compPtr->m_port);
          // null terminate
          endpoint[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE-1] = 0;
          stat = zmq_connect(networkSocket, endpoint);
      }
      FW_ASSERT (0 == stat,stat);

      // create message queue listener
      void *ipcSocket = zmq_socket (compPtr->m_context, ZMQ_PAIR);
      char qname[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE];
      (void)snprintf(qname,ZMQ_ADAPTER_ENDPOINT_NAME_SIZE,"inproc://zmq_rtr_q%d",compPtr->getInstance());
      qname[ZMQ_ADAPTER_ENDPOINT_NAME_SIZE-1] = 0;
      stat = zmq_connect (ipcSocket, qname);
      FW_ASSERT (0 == stat,stat);

      // set up polling for either message queue or network connection
      zmq_pollitem_t items [] = {
          { ipcSocket,   0, ZMQ_POLLIN, 0 },
          { networkSocket, 0, ZMQ_POLLIN, 0 }
      };

      // wait on network or local requests
      char msg[ZMQ_ADAPTER_MSG_SIZE];
      while (true) {
          DEBUG_PRINT("poll\n");
          if (zmq_poll (items, 2, -1) == -1) {
              // if error or close, quit loop
              if (compPtr->zmqError("zmq_poll")) {
                  break;
              } else {
                  continue;
              }
          }

          // messages from port calls are sent on network
          if (items [0].revents & ZMQ_POLLIN) {
              DEBUG_PRINT("Received ipc packet\n");
              while (true) {
                  int size = zmq_recv (ipcSocket, msg, ZMQ_ADAPTER_MSG_SIZE, 0);
                  if (size != -1) {
                      // send packet on socket to recipient
                      while (true) {
                          size = zmq_send (networkSocket,msg,size,0);
                          if (size == -1) {
                              if (compPtr->zmqError("zmq_send")) {
                                  break; // if there is another error, quit trying to receive. Message lost?
                              } else {
                                  continue;
                              }
                          } else {
                              break;
                          }
                      }
                  } else {
                      if (compPtr->zmqError("ipc zmq_recv")) {
                          break;
                      } else {
                          continue;
                      }
                  }
              }
          }
          // messages from network are decoded into port calls
          if (items [1].revents & ZMQ_POLLIN) {
              DEBUG_PRINT("Received network packet\n");
              while (true) {
                  int size = zmq_recv (networkSocket, msg, ZMQ_ADAPTER_MSG_SIZE, 0);
                  if (size != -1) {
                      // decode packet into port call
                      compPtr->decodePacket((U8*)msg,size);
                      break;
                  } else {
                      if (compPtr->zmqError("network zmq_recv")) {
                          break;
                      } else {
                          continue;
                      }
                  }
              }
          }
      }

      zmq_close(networkSocket);
      zmq_close(ipcSocket);

  }

  void ZmqAdapterComponentImpl::decodePacket(U8* packet, NATIVE_UINT_TYPE size) {

      FW_ASSERT(packet);
      FW_ASSERT(size < ZMQ_ADAPTER_MSG_SIZE,size);
      Fw::SerializeStatus stat;
      Fw::ExternalSerializeBuffer buff(packet,size);
      buff.resetDeser();

      DEBUG_PRINT("Decoding packet of size %d\n",size);

      // get port number
      U8 portNum;
      stat = buff.deserialize(portNum);
      // check for deserialization error or port number too high
      if (stat != Fw::FW_SERIALIZE_OK or portNum > this->getNum_PortsOut_OutputPorts()) {
          DEBUG_PRINT("Decode portnum error %d %d\n",stat,this->getNum_PortsOut_OutputPorts());
          //this->tlmWrite_FR_NumDecodeErrors(++this->m_numDecodeErrors);
          return;
      }
      // get size
      U8 entrySize;
      stat = buff.deserialize(entrySize);
      if (stat != Fw::FW_SERIALIZE_OK) {
          DEBUG_PRINT("Decode size error\n");
          //this->tlmWrite_FR_NumDecodeErrors(++this->m_numDecodeErrors);
          return;
      }
      // get buffer for port

      NATIVE_UINT_TYPE decodeSize = entrySize;
      stat = buff.deserialize(this->m_netBuffer.getBuffAddr(),decodeSize,true);
      if (stat != Fw::FW_SERIALIZE_OK) {
          DEBUG_PRINT("Decode data error\n");
          //this->tlmWrite_FR_NumDecodeErrors(++this->m_numDecodeErrors);
          return;
      }
      // set buffer to size of data
      stat = this->m_netBuffer.setBuffLen(entrySize);
      if (stat != Fw::FW_SERIALIZE_OK) {
          DEBUG_PRINT("Set setBuffLen error\n");
          //this->tlmWrite_FR_NumDecodeErrors(++this->m_numDecodeErrors);
          return;
      }
      // call output port
      DEBUG_PRINT("Calling port %d with %d bytes.\n",portNum,entrySize);
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

  bool ZmqAdapterComponentImpl::zmqError(const char* from) {
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
  void ZmqAdapterComponentImpl::ZmqSerialBuffer::operator=(const ZmqSerialBuffer& other) {
      this->resetSer();
      this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
  }

  ZmqAdapterComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(
          const Fw::SerializeBufferBase& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.getBuffAddr(),other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  ZmqAdapterComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(
          const ZmqAdapterComponentImpl::ZmqSerialBuffer& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.m_buff,other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  ZmqAdapterComponentImpl::ZmqSerialBuffer::ZmqSerialBuffer(): Fw::SerializeBufferBase() {

  }

#endif


} // end namespace Zmq
