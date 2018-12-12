// ====================================================================== 
// \title  HLRouterImpl.hpp
// \author tcanham
// \brief  hpp file for HLRouter component implementation class
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

#ifndef HLRouter_HPP
#define HLRouter_HPP

#include "LLProc/HLRouter/HLRouterComponentAc.hpp"
#include "LLProc/HLRouter/HLRouterComponentImplCfg.hpp"
#include "Fw/Buffer/BufferSerializableAc.hpp"

#include <Fw/Types/MemAllocator.hpp>

namespace LLProc {

  class HLRouterComponentImpl :
    public HLRouterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object HLRouter
      //!
      HLRouterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object HLRouter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object HLRouter
      //!
      ~HLRouterComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for Sched
      //!
      void Sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined serial input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for LLPortsIn
      //!
      void LLPortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );

      void processSerialBuffer(Fw::Buffer& buff);

      // helper to reset output buffer
      void resetSerOutBuff(void);

      // Calculates 16 bit CRC for the passed buffer
      static U16 calculate_crc_chksum(const U8 *buff, U16 size);

      // CRC size:
      static const U8 CRC_SIZE = sizeof(U16);

      // Serializable buffer used for driver

      class SerialDriverBuffer :
        public Fw::SerializeBufferBase
      {

        public:

#ifdef BUILD_UT
    	  void operator=(const Fw::SerializeBufferBase& other);
    	  SerialDriverBuffer(const Fw::SerializeBufferBase& other);
    	  SerialDriverBuffer(const SerialDriverBuffer& other);
    	  SerialDriverBuffer();
#endif

          NATIVE_UINT_TYPE getBuffCapacity(void) const {
            return sizeof(m_buff);
          }

          // Get the max number of bytes that can be serialized
          NATIVE_UINT_TYPE getBuffSerLeft(void) const {

            const NATIVE_UINT_TYPE size  = getBuffCapacity();
            const NATIVE_UINT_TYPE loc = getBuffLength();

            if (loc >= (size-1) ) {
                return 0;
            }
            else {
                return (size - loc - 1);
            }
          }

          U8* getBuffAddr(void) {
            return m_buff;
          }

          const U8* getBuffAddr(void) const {
            return m_buff;
          }

        private:
          // Should be the max of all the input ports serialized sizes...
          U8 m_buff[SERIAL_BUFFER_SIZE];

      };

      // input buffers
      Fw::Buffer m_inputBuffObj; //!< input buffer object
      BYTE m_inputBuff[SERIAL_BUFFER_SIZE]; //!< raw buffer for serial driver
      SerialDriverBuffer m_inputAccumulator; //!< input buffer for accumulating UART transactions

      enum TransState {
          TRAN_WAITING, //!< waiting for the beginning of a transaction
          TRAN_SIZE, //!< waiting for size of transaction
          TRAN_SIZE2, //!< waiting for 2nd byte of size of transaction
          TRAN_PROCESSING //!< processing a transaction
      } m_transState;

      U8 m_tempSize;

      NATIVE_UINT_TYPE m_tranLeft; //!< number of bytes left in transaction
      NATIVE_UINT_TYPE m_buffOffset; //!< location in current receive buffer
      // output buffers
      SerialDriverBuffer m_outputBuffer; //!< output buffer for serial port
      Fw::Buffer m_outputBuffObj; //!< output buffer object
      // extract packet (or partial) after sync byte
      void extractPacket(Fw::Buffer& buff);
      void extractPacket2(Fw::Buffer& buff);

      // decode a received packet
      void decodePacket(void);
      // decode a port buffer
      SerialDriverBuffer m_portBuff;

      // some stats
      U32 m_numResyncs; //!< number of searches for sync pattern
      U32 m_numDecodeErrors; //!< number of buffer decoder errors
      U32 m_numPackets; //!< number of received packets
      U32 m_numCrcErrors; //!< number of buffer decoder errors
      U32 m_numGoodPackets; //!< number of good packets, ie passed CRC check
      U32 m_numInvalidPorts; //!< number of invalid ports
      U32 m_numBadSerialPortCalls;  //<! number of bad Serial port calls, ie bad serialize status returned
      U32 m_numOutputBufferOverflows; //<! number of output buffer overflows
      U32 m_numZeroPktSize; //<! number of packets with zero size
      U32 m_numTooBigPktSize; //<! number of packets with greater than max size
      U32 m_maxSerialSendSize; //<! max size of serial packet sent

    };

} // end namespace LLProc

#endif
