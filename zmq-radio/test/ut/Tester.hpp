// ====================================================================== 
// \title  ZmqRadio/test/ut/Tester.hpp
// \author dkooi
// \brief  hpp file for ZmqRadio test harness implementation class
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
#include "fprime-zmq/zmq-radio/ZmqRadioComponentImpl.hpp"

namespace Zmq {

  class Tester :
    public ZmqRadioGTestBase
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
      // Tests
      // ----------------------------------------------------------------------
      //! To do
      //!
      void testConnection(void);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_fileUplinkBufferSendOut
      //!
      void from_fileUplinkBufferSendOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer fwBuffer 
      );

      //! Handler for from_uplinkPort
      //!
      void from_uplinkPort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::ComBuffer &data, /*!< Buffer containing packet data*/
          U32 context /*!< Call context value; meaning chosen by user*/
      );

      //! Handler for from_fileDownlinkBufferSendOut
      //!
      void from_fileDownlinkBufferSendOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer fwBuffer 
      );

      //! Handler for from_fileUplinkBufferGet
      //!
      Fw::Buffer from_fileUplinkBufferGet_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      );

    private:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------
      //! Dispatch internal messages
      //!
      void dispatchAll(void);

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
      ZmqRadioComponentImpl component;

  };

} // end namespace Zmq

#endif
