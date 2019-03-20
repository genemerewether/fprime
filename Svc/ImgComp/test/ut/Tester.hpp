// ======================================================================
// \title  ImgComp/test/ut/Tester.hpp
// \author mereweth
// \brief  hpp file for ImgComp test harness implementation class
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

#ifndef TESTER_HPP
#define TESTER_HPP

#include "GTestBase.hpp"
#include "Svc/ImgComp/ImgCompComponentImpl.hpp"

namespace Svc {

  class Tester :
    public ImgCompGTestBase
  {

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

    public:

      //! Construct object Tester
      //!
      Tester(void);

      //! Destroy object Tester
      //!
      ~Tester(void);

    public:

      // ----------------------------------------------------------------------
      // Test nominal operations
      // ----------------------------------------------------------------------

      void bufGetAllOk(void);

      // ----------------------------------------------------------------------
      // Test off-nominal buffer get calls
      // ----------------------------------------------------------------------

      void bufGetNoMeta(void);

      void bufGetNoThumb(void);

      void bufGetNoImage(void);

      void bufGetNoMetaNoCount(void);

      void bufGetNoThumbNoCount(void);

      void bufGetNoImageNoCount(void);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_compressedOutStorage
      //!
      void from_compressedOutStorage_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Handler for from_compressedOutXmit
      //!
      void from_compressedOutXmit_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Handler for from_compressedGetStorage
      //!
      Fw::Buffer from_compressedGetStorage_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      );

      //! Handler for from_compressedGetXmit
      //!
      Fw::Buffer from_compressedGetXmit_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      );

      //! Handler for from_uncompressedReturn
      //!
      void from_uncompressedReturn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Handler for from_pingOut
      //!
      void from_pingOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

    private:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      Fw::Buffer bufferGetHelper(U32 size);

      //! Connect ports
      //!
      void connectPorts(void);

      //! Initialize components
      //!
      void initComponents(void);

    private:

      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      //!
      ImgCompComponentImpl component;

      //! How to handle BufferGet calls
      //!
      enum BufGetMode {
          METADATA_BUF,
          THUMB_BUF,
          IMAGE_BUF
      } m_bufGetMode;

      char * m_metadataBuf;
      U32 m_metadataBufSize;
      char * m_thumbBuf;
      U32 m_thumbBufSize;
      char * m_imageBuf;
      U32 m_imageBufSize;

      enum BufGetFailMode {
          GET_BUF_SUCCEED,
          FAIL_GET_METADATA_BUF,
          FAIL_GET_THUMB_BUF,
          FAIL_GET_IMAGE_BUF,
          FAIL_GET_ALL,
      } m_bufGetFailMode;

      BufGetFailMode m_bufGetCountFailMode;

      //! Buffer Manager ID to use in subsequent BufferGet calls
      //!
      U32 m_bufMgrID;

      //! Buffer Manager ID to use in subsequent BufferGet calls (increments every call)
      //!
      U32 m_bufID;

  };

} // end namespace Svc

#endif
