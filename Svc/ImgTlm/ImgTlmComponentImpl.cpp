// ======================================================================
// \title  ImgTlmImpl.cpp
// \author mereweth
// \brief  cpp file for ImgTlm component implementation class
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


#include <Svc/ImgTlm/ImgTlmComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Os/Task.hpp>
#include <errno.h>
#include <string.h>

#include <fprime-zmq/zmq/include/zmq.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#define DEBUG_PRINT(x,...)

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ImgTlmComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ImgTlmComponentImpl(
        const char *const compName
    ) :
      ImgTlmComponentBase(compName)
#else
    ImgTlmImpl(void)
#endif
    ,m_context(0)
    ,m_pubSocket(0)
    ,m_packetsSent(0)
  {

  }

  void ImgTlmComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    ImgTlmComponentBase::init(queueDepth, instance);

  }

  ImgTlmComponentImpl ::
    ~ImgTlmComponentImpl(void)
  {
      // clean up zmq state
      if (this->m_context) {
          zmq_ctx_destroy (this->m_context);
      }
  }

  void ImgTlmComponentImpl::preamble(void) {

      // ZMQ requires that a socket be created, used, and destroyed on the same thread,
      // so we create it in the preamble. Only do it if the context was successfully created.
      if (this->m_context) {
          this->m_pubSocket = zmq_socket (this->m_context, ZMQ_PUB);
          if (not this->m_pubSocket) {
            DEBUG_PRINT("Failed to create PUB socket\n");
            /*Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
              this->log_WARNING_HI_ZP_SocketError(errArg);*/
              return;
          }

          zmq_setsockopt(this->m_pubSocket, ZMQ_SNDHWM, &IMG_TLM_HWM, sizeof(IMG_TLM_HWM));
          zmq_setsockopt(this->m_pubSocket, ZMQ_LINGER, &IMG_TLM_LINGER, sizeof(IMG_TLM_LINGER));
          zmq_setsockopt(this->m_pubSocket, ZMQ_SNDTIMEO, &IMG_TLM_SNDTIMEO, sizeof(IMG_TLM_SNDTIMEO));
          //zmq_setsockopt(this->m_pubSocket, ZMQ_SWAP, &IMG_TLM_SWAP, sizeof(IMG_TLM_SWAP));

          NATIVE_INT_TYPE stat;

          // connect to the subscriber
          stat = zmq_connect(this->m_pubSocket, m_endpoint);
          if (0 != stat) {
            DEBUG_PRINT("Failed to connect to sub %s\n", m_endpoint);
            /*Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
              this->log_WARNING_HI_ZP_BindError(errArg);*/
              return;
          } else {
            DEBUG_PRINT("Connected to sub %s\n", m_endpoint);
            //this->log_ACTIVITY_HI_ZP_PublishConnectionOpened();
          }
      }
  }

  void ImgTlmComponentImpl::finalizer(void) {

      // ZMQ requires that a socket be created, used, and destroyed on the same thread,
      // so we close it in the finalizer
      DEBUG_PRINT("Finalizing\n");
      if (this->m_pubSocket) {
          zmq_close(this->m_pubSocket);
      }
  }

  void ImgTlmComponentImpl::open(
              const char* endpoint
          ) {

      // store values for worker thread
      (void)snprintf(m_endpoint,IMG_TLM_ENDPOINT_NAME_SIZE,"tcp://%s",endpoint);

      // null terminate
      m_endpoint[IMG_TLM_ENDPOINT_NAME_SIZE-1] = 0;

      DEBUG_PRINT("Storing endpoint %s\n", m_endpoint);

      // create zmq context
      m_context = zmq_ctx_new ();
      if (not m_context) {
        DEBUG_PRINT("Failed to create zmq context\n");
        /*Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
          this->log_WARNING_HI_ZP_ContextError(errArg);*/
      }

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ImgTlmComponentImpl ::
    ImageRecv_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &fwBuffer
    )
  {
      // return if we never successfully created the socket
      if (not this->m_pubSocket) {
          this->ImageForward_out(portNum, fwBuffer);
          return;
      }

      zmq_msg_t msg;
      NATIVE_INT_TYPE stat;

      stat = zmq_msg_init_size(&msg, 640*480+2);
      if (0 != stat) {
        DEBUG_PRINT("Failed to init msg\n");
          return;
      } else {
        DEBUG_PRINT("Init-ed message\n");
      }
      strncpy((char*)zmq_msg_data(&msg), "ZP", 2);
      memcpy(((uint8_t*)zmq_msg_data(&msg)) + 2, (void*) fwBuffer.getdata(), 640*480);
      stat = zmq_msg_send(&msg, m_pubSocket, 0);
      if (-1 == stat) {
        DEBUG_PRINT("Failed to send msg; error %d\n", errno);
      } else {
        DEBUG_PRINT("Sent msg\n");
      }

      DEBUG_PRINT("Closing message\n");
      zmq_msg_close(&msg);

      // If some UDP-transport based socket supports multipart messages at some point
      /*uint8_t* ptr = framePtr->data;
      for (int i = 0; i < 480; i++) {
        stat = zmq_msg_init_size(&msg, 640);//*480);
        if (0 != stat) {
          DEBUG_PRINT("Failed to init msg for part %d\n", i);
            framePtr->releaseRef();
            return;
        } else {
          DEBUG_PRINT("Init-ed message for part %d\n", i);
        }

        memcpy(zmq_msg_data(&msg), ptr, 640);//*480);

        stat = zmq_msg_set_group(&msg, "ImgTlm");
        if (0 != stat) {
          DEBUG_PRINT("Failed to set msg group for part %d\n", i);
            framePtr->releaseRef();
            return;
        } else {
          DEBUG_PRINT("Set msg group for part %d\n", i);
        }

        if (i != 479) {
          stat = zmq_msg_send(&msg, m_pubSocket, ZMQ_SNDMORE);
        }
        else {
          stat = zmq_msg_send(&msg, m_pubSocket, 0);
        }
        if (-1 == stat) {
          DEBUG_PRINT("Failed to send msg; error %d; part %d\n", errno, i);
        } else {
          DEBUG_PRINT("Sent msg part %d\n", i);
        }

        DEBUG_PRINT("Closing message part %d\n", i);
        zmq_msg_close(&msg);

        ptr += 640;
      }*/

      this->ImageForward_out(portNum, fwBuffer);
  }

} // end namespace Svc
