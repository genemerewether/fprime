// ======================================================================
// \title  ImgCompImpl.hpp
// \author mereweth
// \brief  hpp file for ImgComp component implementation class
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

#ifndef ImgComp_HPP
#define ImgComp_HPP

#include "Svc/ImgComp/ImgCompComponentAc.hpp"
#include "Svc/ImgComp/ImgCompComponentImplCfg.hpp"

#include "turbojpeg.h"

namespace Svc {

  class ImgCompComponentImpl :
    public ImgCompComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ImgComp
      //!
      ImgCompComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ImgComp
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ImgComp
      //!
      ~ImgCompComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Declarations of methods inherited from base class
      // ----------------------------------------------------------------------

      //! Override preamble function 
      //!
      void preamble(void);

      //! Override finalizer function
      //!
      void finalizer(void);

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for uncompressedIn
      //!
      void uncompressedIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer
      );

      //! Handler implementation for pingIn
      //!
      void pingIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Handler implementation for schedIn
      //!
      void schedIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    PRIVATE:

    void pushEmptyBuffStorage(U32 portNum, BufferOffsetSkipType skip);

    void pushEmptyBuffXmit(U32 portNum, BufferOffsetSkipType skip);

    void getBuffAndPushStorage(U32 portNum,
                               U32 metaSize, void* metaPtr,
                               U32 thumbSize, void* thumbPtr,
                               U32 imgSize, void* imgPtr);

    void getBuffAndPushXmit(U32 portNum,
                            U32 metaSize, void* metaPtr,
                            U32 thumbSize, void* thumbPtr,
                            U32 imgSize, void* imgPtr);

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for IMGCOMP_NoOp command handler
      //!
      void IMGCOMP_NoOp_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      // ----------------------------------------------------------------------
      // Member variables
      // ----------------------------------------------------------------------

        //! TurboJpeg handles (one per compression size)
        //!
        tjhandle m_jpegHandle;

        //! Number of buffers that ImgComp has handled
        //!
        U32 m_buffersHandled;

        //! Temporary buffer for deinterleaving Cb and Cr
        BYTE* m_deinterBuffer;
        U32 m_deinterBufferSize;

        //! Temporary buffer to hold the output of the compression
        //!
        BYTE* m_tempCompBuffer;
        U32 m_tempCompBufferSize;

        //! Temporary buffer to hold the output of the compression
        //!
        BYTE* m_thumbCompBuffer;
        U32 m_thumbCompBufferSize;

        NATIVE_INT_TYPE m_setenvRet;
    };

} // end namespace Svc

#endif
