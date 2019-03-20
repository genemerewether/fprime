// ======================================================================
// \title  ImgTlmImpl.hpp
// \author mereweth
// \brief  hpp file for ImgTlm component implementation class
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

#ifndef ImgTlm_HPP
#define ImgTlm_HPP

#include "Svc/ImgTlm/ImgTlmComponentAc.hpp"
#include "Svc/ImgTlm/ImgTlmComponentImplCfg.hpp"

#include <fprime-zmq/zmq/include/zmq.h>

namespace Svc {

  class ImgTlmComponentImpl :
    public ImgTlmComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ImgTlm
      //!
      ImgTlmComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ImgTlm
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Open the connection
      //!
      void open(
              const char* endpoint
              );

      //! Destroy object ImgTlm
      //!
      ~ImgTlmComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for ImageRecv
      //!
      void ImageRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer
      );

      //! Preamble override to set up zmq message queue
      void preamble(void);
      //! Finalizer override to clean up zmq message queue
      void finalizer(void);

      char m_endpoint[IMG_TLM_ENDPOINT_NAME_SIZE]; /*!< endpoint for pub connection */

      // zmq variables

      //  Prepare our context and socket

      void *m_context; //!< zmq context
      void *m_pubSocket; //!< zmp socket for outbound buffers to poll task

      U32 m_packetsSent; //!< number of packets sent
    };

} // end namespace Svc

#endif
