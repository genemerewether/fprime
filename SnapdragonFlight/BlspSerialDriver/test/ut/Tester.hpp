// ====================================================================== 
// \title  BlspSerialDriver/test/ut/Tester.hpp
// \author tcanham
// \brief  hpp file for BlspSerialDriver test harness implementation class
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

#include "TesterBase.hpp"
#include "SnapdragonFlight/BlspSerialDriver/BlspSerialDriverComponentImpl.hpp"

namespace SnapdragonFlight {

  class Tester :
    public BlspSerialDriverTesterBase
  {

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

    public:

      //! Construct object Tester
      //!
      Tester(NATIVE_INT_TYPE device, NATIVE_INT_TYPE numReadBuffers, NATIVE_INT_TYPE bufferSize,  BlspSerialDriverComponentImpl::UartFlowControl flow);

      //! Destroy object Tester
      //!
      ~Tester(void);

    public:

      // ---------------------------------------------------------------------- 
      // Tests
      // ---------------------------------------------------------------------- 

      //! Send serial data
      //!
      void sendSerial(BYTE* data, NATIVE_INT_TYPE size);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_serialRecv
      //!
      void from_serialRecv_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus& status /*!< Status of read*/
      );

    private:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

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
      BlspSerialDriverComponentImpl component;

      NATIVE_INT_TYPE m_numBuffers;
      NATIVE_INT_TYPE m_bufferSize;
      Fw::Buffer m_readBuffer[100];
      BYTE* m_readData[FW_NUM_ARRAY_ELEMENTS(m_readBuffer)];

      void textLogIn(
                const FwEventIdType id, //!< The event ID
                Fw::Time& timeTag, //!< The time
                const Fw::TextLogSeverity severity, //!< The severity
                const Fw::TextLogString& text //!< The event string
            );

  };

} // end namespace Snapdragon

#endif
