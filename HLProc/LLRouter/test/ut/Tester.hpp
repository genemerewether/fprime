// ====================================================================== 
// \title  LLRouter/test/ut/Tester.hpp
// \author tcanham
// \brief  hpp file for LLRouter test harness implementation class
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
#include "HLProc/LLRouter/LLRouterComponentImpl.hpp"

namespace HLProc {

  class Tester :
    public LLRouterGTestBase
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

      //! Nominal cases
      //!
      void nominalInit(void);

      void nominalOneFrame(void);
      void nominalTwoFrames(void);
      void nominalOneSplit(void);
      void nominalOneSplitTwice(void);
      void nominalTwoFramesOneBuffer(void);
      void nominalSyncOnly(void);
      void nominalSyncPlusSize(void);

      void nominalSendPackets(void);

      //! Off-nominal cases
      void offNomResyncOnePacket(void);
      void offNomPktResyncPkt(void);
      void offNomDecodeError(void);
      void offNomSerialError(void);
      void offNomCrcError(void);
      void offNomInvalidPort(void);
      void offNomBadSerialPortCall(void);
      void offNomOutpuBufferOverflow(void);
      void offNomZeroPacketSize(void);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_SerWritePort
      //!
      void from_SerWritePort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      );

      //! Handler for from_SerialBufferSend
      //!
      void from_SerialBufferSend_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

    private:

      // ----------------------------------------------------------------------
      // Handlers for serial from ports
      // ----------------------------------------------------------------------

      //! Handler for from_LLPortsOut
      //!
      void from_LLPortsOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );

      //! Handler for from_pingOut
      //!
      void from_pingOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Handler for from_PolyGet
      //!
      void from_PolyGet_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
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

      //! Dump test buffer
      void dumpBuff(U8* buff, NATIVE_UINT_TYPE bytes);

      void verifyNoErrors(U32 idx_num_pkts = 0, U32 idx_num_good_pkts = 0, U32 num_good_pkts = 1);

    private:

      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      //!
      LLRouterComponentImpl component;

      Fw::Buffer m_serBufferObject; // Buffer container for serial data
      U8 m_serBuffer[SERIAL_BUFFER_SIZE]; // Raw buffer for serial data
      bool m_serPortWritten; // flag indicating port call happened
      U32 m_numPacketsSent; // Number of packets sent over serial from tester

      // accumulate test buffers
      LLRouterComponentImpl::SerialDriverBuffer m_testBuffer;

      // starts a test buffer - add sync and size
      void startBuffer(void);
      // add a port call to a buffer
      void addPortCall(U8 portNum, BYTE* buffer, U8 bytes);
      // finish the buffer - add CRC and update size
      void finishBuffer(void);
      // fill buffer part of port call
      void fillBuffer(U8* buff, NATIVE_UINT_TYPE bytes);
      // reset the data buffer back to zero
      void resetBuffer(void);
      // add the current test buffer to the data buffer
      // start = -1 = beginning, size = -1 = rest of buffer
      void addBuffer(NATIVE_INT_TYPE start = -1, NATIVE_INT_TYPE size = -1);

      // insert some bytes for resync testing
      void addOffBytes(NATIVE_UINT_TYPE bytes);

      // verify a port call has been made
      void verifyPortCall(NATIVE_INT_TYPE port, NATIVE_UINT_TYPE bytes, NATIVE_UINT_TYPE order);

      typedef struct {
          NATIVE_INT_TYPE port;
          LLRouterComponentImpl::SerialDriverBuffer buff;
      } serPortHistory;

      History<serPortHistory>
              *m_serPortHistory;

      U32 m_fromSerPortHistorySize;

      // clear port call history
      void clearLLPort(void);

      // send serial buffer
      void sendBuffer(void);

      // values for PolyDb call
      Svc::MeasurementStatus m_polyStatus;
      Fw::PolyType m_polyVal;

  };

} // end namespace HLProc

#endif
