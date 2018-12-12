// ====================================================================== 
// \title  LLRouterImpl.hpp
// \author tcanham
// \brief  hpp file for LLRouter component implementation class
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

#ifndef LLRouter_HPP
#define LLRouter_HPP

#include "HLProc/LLRouter/LLRouterComponentAc.hpp"
#include "HLProc/LLRouter/LLRouterComponentImplCfg.hpp"
#include "Fw/Buffer/BufferSerializableAc.hpp"
#include "Os/IntervalTimer.hpp"
#include "Os/File.hpp"

#include <Fw/Types/MemAllocator.hpp>
#include <limits.h>

namespace HLProc {

  class LLRouterComponentImpl :
    public LLRouterComponentBase
  {

    public:

      // Number of sched cycles to wait for a response from R5
      static const U32 PATCH_TIMEOUT = 15;

      static const char PATCH_ACK_CHAR = '$';

      static const U32 DEFAULT_R5_START_ADDR = 0x00200000;

      static const U32 PATCH_FLAG_FORCEBOOT = 0x1;

      static const BYTE START_BYTE = 0xBE;

      static const U32 HEADER_LENGTH = 3;

      static const U32 MAX_DATA_PACKET_LENGTH = 32;

      static const U32 MAX_TOTAL_PACKET_LENGTH = 1 + HEADER_LENGTH + MAX_DATA_PACKET_LENGTH + 1;

      const char* BOOTLOADER_MAGIC_STRING = "LeoBoot!";

      static const U32 METADATA_FLAGS_FORCEBOOT = (1 << 0);

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object LLRouter
      //!
      LLRouterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object LLRouter
      //!
      void init(
          NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          NATIVE_INT_TYPE msgSize, /*!< The message size*/
          NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );
      //! Destroy object LLRouter
      //!
      ~LLRouterComponentImpl(void);

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

      //! Handler for input port CycleDone
      //
      void CycleDone_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handlers to implement
      // ----------------------------------------------------------------------

      //! Handler for command LLR_SET_PATCH_MODE
      /* No-op command */
      void LLR_SET_PATCH_MODE_cmdHandler(
              FwOpcodeType opCode, /*!< The opcode*/
              U32 cmdSeq, /*!< The command sequence number*/
              LLRPatchMode Mode 
      );

      //! Handler for command LLR_LOAD_PATCH_FILE
      /* No-op string command */
      void LLR_LOAD_PATCH_FILE_cmdHandler(
              FwOpcodeType opCode, /*!< The opcode*/
              U32 cmdSeq, /*!< The command sequence number*/
              const Fw::CmdStringArg& file, /*!< Patch file*/
              LL updatingLL, /*!< The LL to update*/
              bool legacyBootloader, /*!< Update LLs with an old bootloader*/
              bool forceboot, /*!< Tell the bootloader to ignore CRC32 checks on startup*/
              U64 disabled_selftests /*!< Bitmask of tests to disable on startup. A 1 indicates that the test should not be run*/
      );

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined serial input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for HLPortsIn
      //!
      void HLPortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );

      void SerReadPort_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

      void run_cycle();

      //! Preamble for loading read buffers in driver
      void preamble(void);

      // helper to reset output buffer
      void resetSerOutBuff(void);

      // Calculates 16 bit CRC for the passed buffer
      static U16 calculate_crc_chksum(const U8 *buff, U16 size);

      // CRC size:
      static const U8 CRC_SIZE = sizeof(U16);

      void setRouterPatchMode(LLRPatchMode newMode);

      void runPatchFSM();

      void writePacket(BYTE* output_buffer, U32 buffer_len);

      // Start a patch update for the LL
      void beginUpdate(LL updatingLL, bool legacyBootloader, bool forceboot, U64 disabled_selftests);

      // Send the address packet
      void sendAddressPacket();

      // Send the next data packet
      // return: Number of bytes written
      int sendDataPacket();

      // Send a metadata packet
      void sendMetadataPacket();

      // Send an End of File packet
      void sendEoFPacket();

      // Cleanup the update process
      void finishUpdate();

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

      void processSerialBuffer(Fw::Buffer& buff);

      // input buffers
      BYTE m_inputBuff[SERIAL_RECEIVE_BUFFER_POOL_SIZE][SERIAL_BUFFER_SIZE]; //!< locally allocated buffer (to serial driver)
      Fw::Buffer m_inputBuffObj[SERIAL_RECEIVE_BUFFER_POOL_SIZE]; //!< input buffer objects

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

      //! Implementation for LLR_CYCLE_SERIAL command handler
      //! Command to have scheduler port send serial data if GNC loop is not running
      void LLR_CYCLE_SERIAL_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          bool on /*!< Turn the sending on or off.*/
      );

      //! Handler implementation for pingIn
      //!
      void pingIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );


//      // packet sequence numbers:
//      U8 m_pktSeqNum; //<! current packet sequence number
//      U8 m_ackPktSeqNum; //<! latest packet sequence number

      // some stats
      U32 m_numResyncs; //!< number of searches for sync pattern
      U32 m_numDecodeErrors; //!< number of buffer decoder errors
      U32 m_numPackets; //!< number of received packets
      U32 m_numSerialErrors; //!< number of serial errors
      U32 m_numCrcErrors; //!< number of CRC errors
      U32 m_numGoodPackets; //!< number of good packets, ie passed CRC check
      U32 m_numInvalidPorts; //!< number of invalid ports
      U32 m_numBadSerialPortCalls;  //<! number of bad Serial port calls, ie bad serialize status returned
      U32 m_numOutputBufferOverflows; //<! number of output buffer overflows
      U32 m_numTooBigPktSize; //<! number of packets with gt than max protect size
      U32 m_numZeroPktSize; //<! number of packets with zero size
//      U32 m_numMissedPktSeqNums; //<! number of missed packet sequence numbers in the ACK
//      U32 m_numRetries; //<! number of critical port retries
      U32 m_numPatchBytesSent;

      Os::IntervalTimer m_cycleTimer;
      U32 m_maxCycleTime;
      U32 m_cycleTime;
      bool m_schedSerial; // have scheduler call send serial data

      enum { 
        MAX_FILENAME_SIZE = NAME_MAX, // as defined in limits.h
        MAX_PATH_SIZE = PATH_MAX
      };

      typedef enum {
          PATCH_STATE_ADDRESS,      // Sending address packet
          PATCH_STATE_DATA,         // Sending data/metadata packets
          PATCH_STATE_EOF,          // Sending EoF packet
          PATCH_STATE_DONE          // Finished patching
      } LLRPatchState;

      Os::File m_patchFile;
      U8 m_patchFilename[MAX_FILENAME_SIZE + MAX_PATH_SIZE];
      U32 m_patchFileSize;
      U32 m_patchTimeoutCounter;

      SerialDriverBuffer m_patchOutputBuffer;
      Fw::Buffer m_patchOutputBuffObj;

      LLRPatchMode m_LLRMode;
      LLRPatchState m_LLRPatchState;

      bool m_seenMagicString;
      U8 m_magicStringBuffer[sizeof(BOOTLOADER_MAGIC_STRING)];

      // _SET modes will set the POR on the next sched handler
      // _CLEAR modes will clear the POR on the next sched handler
      // _SET_PATCH modes will set the patch enable bit on the next sched handler
      typedef enum {
          REBOOT_NONE,              // No reboot necessary
          REBOOT_FOR_PATCHING_SET,  // Reboot the m_patchingLL into patching mode. Hold the other LL in POR
          REBOOT_ALL_SET,           // Reboot both LLs into normal mode
          REBOOT_FOR_PATCHING_CLEAR,
          REBOOT_ALL_CLEAR
      } LLRebootType;

      LL m_patchingLL;
      LL m_primeLL;
      LLRebootType m_LLRebootState;


      FwOpcodeType m_patching_opcode;
      U32 m_patching_cmdSeq;

      bool m_newPacket; //!< flag to indicate a new packet from LL has arrived

      bool m_flowing; //!< if traffic is allowed to flow

      //!< Tell the bootloader to ignore Application CRC errors
      bool m_forcebootFlag;

      //!< Selftests that should be run on startup
      U64 m_selftests;

      //!< Look for an ACK byte before starting update, instead of the magic string
      bool m_legacy_start_byte;

    };

} // end namespace HLProc

#endif
