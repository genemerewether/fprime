// ======================================================================
// \title  LLRouterImpl.cpp
// \author tcanham
// \brief  cpp file for LLRouter component implementation class
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

#include <HLProc/LLRouter/LLRouterComponentImpl.hpp>
#include <Common/Cfg/QuestConstants.hpp>
#include <HLProc/Cfg/HLProcPolyDb.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>
#include <Os/File.hpp>
#include <Os/FileSystem.hpp>
#include <string.h> // memcpy
#include <Fw/Types/SerialBuffer.hpp>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <Utils/Hash/libcrc/lib_crc.h> // borrow CRC

#ifdef __cplusplus
}
#endif // __cplusplus

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#define DEBUG_PRINT(x,...)

#define UART_LOOPBACK 0

namespace HLProc {

  //!< Shouldn't be more loop iterations than the number of bytes in the buffer,
  //!< and it can be higher than 5 b/c of off-nominal conditions
  static const NATIVE_UINT_TYPE MAX_ITERS = SERIAL_BUFFER_SIZE;

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  LLRouterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    LLRouterComponentImpl(
        const char *const compName
    ) :
      LLRouterComponentBase(compName)
#else
    LLRouterImpl(void)
#endif
    ,m_transState(TRAN_WAITING)
    ,m_tempSize(0)
    ,m_tranLeft(0)
    ,m_buffOffset(0) // ,m_pktSeqNum(0) // ,m_ackPktSeqNum(0)
    ,m_numResyncs(0)
    ,m_numDecodeErrors(0)
    ,m_numPackets(0)
    ,m_numSerialErrors(0)
    ,m_numCrcErrors(0)
    ,m_numGoodPackets(0)
    ,m_numInvalidPorts(0)
    ,m_numBadSerialPortCalls(0)
    ,m_numOutputBufferOverflows(0)
    ,m_numTooBigPktSize(0)
    ,m_numZeroPktSize(0)
    ,m_numPatchBytesSent(0)
    ,m_maxCycleTime(0)
//	,m_numMissedPktSeqNums(0)
//	,m_numRetries(0)
	,m_cycleTime(0)
    ,m_schedSerial(true)
    ,m_patchFileSize(0)
    ,m_patchTimeoutCounter(0)
    ,m_LLRMode(LLR_NORMAL)
    ,m_LLRPatchState(PATCH_STATE_DONE)
    ,m_seenMagicString(false)
    ,m_patchingLL(LL_A)
    ,m_primeLL(LL_A)
    ,m_LLRebootState(REBOOT_NONE)
    ,m_patching_opcode(0)
    ,m_patching_cmdSeq(0)
    ,m_newPacket(false)
    ,m_flowing(false)
    ,m_forcebootFlag(false)
    ,m_selftests(0xFFFFFFFFFFFFFFFFlu)
    ,m_legacy_start_byte(false)
  {

  }

  void LLRouterComponentImpl ::
    init(
        NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
        NATIVE_INT_TYPE msgSize, /*!< The message size*/
        NATIVE_INT_TYPE instance /*!< The instance number*/
    ) 
  {
    LLRouterComponentBase::init(queueDepth,msgSize,instance);

    // Initialize memory buffer objects
    for (NATIVE_UINT_TYPE buff = 0; buff < SERIAL_RECEIVE_BUFFER_POOL_SIZE; buff++) {
        this->m_inputBuffObj[buff].setdata((U64)this->m_inputBuff[buff]);
        this->m_inputBuffObj[buff].setsize(SERIAL_BUFFER_SIZE);
    }

    this->m_outputBuffObj.setdata((U64)this->m_outputBuffer.getBuffAddr());
    this->m_outputBuffObj.setsize(this->m_outputBuffer.getBuffLength());
    this->m_outputBuffObj.setmanagerID(0);
    this->m_outputBuffObj.setbufferID(0);
    // set up outgoing serial buffer for the first cycle
    this->resetSerOutBuff();

    this->m_patchOutputBuffObj.setmanagerID(0);
    this->m_patchOutputBuffObj.setbufferID(0);

  }

  LLRouterComponentImpl ::
    ~LLRouterComponentImpl(void)
  {

  }

  // Push buffers to serial port
  void LLRouterComponentImpl :: preamble(void) {
      for (NATIVE_INT_TYPE buff = 0; buff < SERIAL_RECEIVE_BUFFER_POOL_SIZE; buff++) {
          this->SerialBufferSend_out(0,this->m_inputBuffObj[buff]);
      }
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void LLRouterComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {

      // Check for power state
      I32 val;
      Svc::MeasurementStatus status;
      Fw::Time time;
      Fw::PolyType polyVal;
      if (this->isConnected_PolyGet_OutputPort(0)) {
          this->PolyGet_out(0,HLProc::HLPROC_POLYDB_POWER_STATE,status,time,polyVal);
          if (Svc::MEASUREMENT_OK == status) {
              if (not polyVal.isI32()) {
                  this->log_WARNING_HI_LLR_PolyError();
                  return;
              }
              polyVal.get(val);
              HLProcPolyDbPowerState state = (HLProcPolyDbPowerState) val;
              switch (state) {
                  case HLPROC_POLYDB_POWER_AWAKE:
                  case HLPROC_POLYDB_POWER_SLEEP: // fall through
                      this->m_flowing = false;
                      break;
                  case HLPROC_POLYDB_POWER_PREFLIGHT: // fall through
                  case HLPROC_POLYDB_POWER_FLIGHT:
                      this->m_flowing = true;
                      break;
                  default:
                      FW_ASSERT(0,state);
                      break;
              }
          }
      }
      else {
          this->m_flowing = true;
      }
  
      // TODO
      // This may be used by the boot loader to query register values periodically
      // Can also be used for timeouts
      if (this->m_schedSerial) {
          this->run_cycle();
      }

      if (this->m_newPacket) {
          this->tlmWrite_LLR_NumGoodPackets(this->m_numGoodPackets);
          this->tlmWrite_LLR_NumPackets(this->m_numPackets);
          this->tlmWrite_LLR_CycleTime(this->m_cycleTime);
          this->m_newPacket = false;
      }


      // Timeout and return to Normal mode after PATCH_TIMEOUT/10 seconds
      if (this->m_LLRMode == LLR_PATCH && this->m_LLRPatchState != PATCH_STATE_DONE) {

          if (this->m_patchTimeoutCounter > 0) {
              this->m_patchTimeoutCounter--;
          } else {
              DEBUG_PRINT("Bootloader timeout at state: %d\n", this->m_LLRPatchState);


              Fw::LogStringArg emptyLogString("");

              if (this->m_LLRPatchState == PATCH_STATE_ADDRESS) {
                  log_WARNING_HI_LLR_PatchError(LLR_PERR_NO_START_ACK, 0, emptyLogString);
              } else {
                log_WARNING_HI_LLR_PatchError(LLR_PERR_TIMEOUT, 0, emptyLogString);
              }

              this->finishUpdate();
          }
      }

      // We can only send one FPGA command per schedule call or we risk the FPGA
      // not updating properly. This state machine allows one or both of the LLs
      // to be rebooted into normal or patching mode
      //DEBUG_PRINT("Reboot mode: %u\n", this->m_LLRebootState);
      switch (this->m_LLRebootState) {
          case REBOOT_NONE:
              break;

          case REBOOT_FOR_PATCHING_SET:
              DEBUG_PRINT("Rebooting %u for patching set\n", this->m_patchingLL);
              //TODO(mereweth)
              this->m_LLRebootState = REBOOT_FOR_PATCHING_CLEAR;
              break;

          case REBOOT_ALL_SET:
              DEBUG_PRINT("Rebooting ALL normally set\n");
              //TODO(mereweth)
              this->m_LLRebootState = REBOOT_ALL_CLEAR;
              break;

          case REBOOT_FOR_PATCHING_CLEAR:
              DEBUG_PRINT("Rebooting %u for patching clear\n", this->m_patchingLL);
              //TODO(mereweth)
              this->m_LLRebootState = REBOOT_NONE;
              break;

          case REBOOT_ALL_CLEAR:
              DEBUG_PRINT("Rebooting ALL normally clear\n");
              //TODO(mereweth)

              this->m_LLRebootState = REBOOT_NONE;
              break;

          default:
              DEBUG_PRINT("Unknown reboot state!\n");
              this->m_LLRebootState = REBOOT_NONE;
              break;
      }

  } // end Sched_handler

  void LLRouterComponentImpl::run_cycle()
  {
      this->m_cycleTimer.stop();
      this->m_cycleTime = this->m_cycleTimer.getDiffUsec();
      if (this->m_cycleTime > this->m_maxCycleTime) {
          this->m_maxCycleTime = this->m_cycleTime;
          this->tlmWrite_LLR_MaxCycleTime(this->m_cycleTime);
      }
      // Send out serial data if there is data beyond SYNC and size bytes
      if (this->m_outputBuffer.getBuffLength() > 3*sizeof(BYTE)) {
          // set size
          BYTE* ptr = this->m_outputBuffer.getBuffAddr();
          Fw::SerialBuffer tempBuff(&ptr[1], 2);
          // poke in size
          // TODO(mereweth) - check ser status
          // data minus SYNC and size bytes
          tempBuff.serialize(static_cast<U16>(this->m_outputBuffer.getBuffLength()
                                              - 3*sizeof(BYTE)));
          // set CRC - CRC starts after sync and size bytes
          U16 crc = LLRouterComponentImpl::calculate_crc_chksum(&ptr[3*sizeof(BYTE)],this->m_outputBuffer.getBuffLength()-3*sizeof(BYTE));
          Fw::SerializeStatus stat = this->m_outputBuffer.serialize(crc);
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
          // set the serial buffer size
          this->m_outputBuffObj.setsize(this->m_outputBuffer.getBuffLength());
          DEBUG_PRINT("Sending %d bytes\n",this->m_outputBuffer.getBuffLength());
          // send data to driver
          this->SerWritePort_out(0,this->m_outputBuffObj);
          // reset buffer for next RTI
          this->resetSerOutBuff();
      }
  }

  void LLRouterComponentImpl::CycleDone_handler(
      NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context )
  {
      this->run_cycle();
  }

  void LLRouterComponentImpl ::
    resetSerOutBuff(void) {
      // reset buffer
      this->m_outputBuffer.resetSer();
      Fw::SerializeStatus stat;
      // serialize start of frame character for next round of data
      stat = this->m_outputBuffer.serialize(Cfg::UART_SYNC_BYTE);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      // serialize placeholder size - will be filled in when packet is complete
      stat = this->m_outputBuffer.serialize((U16)0);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
  }

  U16 LLRouterComponentImpl ::
      calculate_crc_chksum(const U8 *buff, U16 size) {

      FW_ASSERT(buff != NULL);

      U16 crc = 0xFFFF;

      for (U32 i = 0; i < size; ++i) {
          crc = update_crc_16(crc, buff[i]);
      }

      return crc;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void LLRouterComponentImpl ::
    HLPortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      // Add to outgoing buffer
      DEBUG_PRINT("Port %d in. Size: %d\n",portNum,Buffer.getBuffLength());

      // Check if we can fit the data in the output buffer:
      const NATIVE_UINT_TYPE total_size = sizeof(U8) + 2*sizeof(U8) + Buffer.getBuffLength();

      if (total_size > Buffer.getBuffLength() && // Checking for overflow in addition above
          total_size <= this->m_outputBuffer.getBuffSerLeft()) {

          // Add port number
          Fw::SerializeStatus stat = this->m_outputBuffer.serialize((U8)portNum);
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

          // Add size of buffer
          stat = this->m_outputBuffer.serialize((U16)Buffer.getBuffLength());
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

          // Add data
          stat = this->m_outputBuffer.serialize(Buffer.getBuffAddr(),Buffer.getBuffLength(),true);
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

      }
      else {

          DEBUG_PRINT("Output buffer overflow! Port %u in. Size: %u Left: %u\n",
                      portNum,total_size,this->m_outputBuffer.getBuffSerLeft());
          this->tlmWrite_LLR_NumOuputBufferOverflows(++this->m_numOutputBufferOverflows);
      }

  }

  void LLRouterComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
      this->pingOut_out(0,key);
  }



  void LLRouterComponentImpl ::
    processSerialBuffer(Fw::Buffer& buff) {

      // Process serial data
      // Get amount of data received
      NATIVE_UINT_TYPE buffSize = buff.getsize();
      DEBUG_PRINT("Received %d bytes\n",buffSize);

      this->m_buffOffset = 0; // reset receive buffer offset
      for (NATIVE_UINT_TYPE iter = 0; iter < MAX_ITERS; iter++) {
          DEBUG_PRINT("IT: %d\n",iter);
          if (this->m_buffOffset < buffSize) {
              BYTE* ptr = (BYTE*)buff.getdata();
              DEBUG_PRINT("buffOffset: %d\n",this->m_buffOffset);
              switch (this->m_transState) {
                  case TRAN_WAITING: {
                          // search for sync character
                          NATIVE_UINT_TYPE start = this->m_buffOffset;
                          for (; this->m_buffOffset < buffSize; this->m_buffOffset++) {
                              DEBUG_PRINT("S: %d B: 0x%02X ",this->m_buffOffset,ptr[this->m_buffOffset]);
                              if (Cfg::UART_SYNC_BYTE == ptr[this->m_buffOffset]) {
                                  DEBUG_PRINT("Sync offset %d\n",this->m_buffOffset);
                                  if (this->m_buffOffset > start) {
                                      this->tlmWrite_LLR_ResyncErrors(++this->m_numResyncs);
                                  }
                                  // add offset to buffOffset since we have moved there
                                  this->extractPacket(buff);
                                  break; // quit loop
                              } // end if found sync byte
                              DEBUG_PRINT("\n");
                          } // end search for sync character
                      } // end context for start
                      break;
                  case TRAN_SIZE:
                  case TRAN_SIZE2:
                      // if it gets here, 1st or 2nd byte finishes size
                      this->extractPacket(buff);
                      break;
                  case TRAN_PROCESSING:
                      // get rest of packet
                      this->extractPacket2(buff);
                      break;
                  default:
                      FW_ASSERT(0,this->m_transState);
                      break; // make analyzers happy
              } // end switch m_transState
          } else {
              break; // break out of loop if no data left in buffer
          } // end if buffer data left
      } // end while not max iters

  }

  void LLRouterComponentImpl ::
     decodePacket(void) {

      DEBUG_PRINT("** decodePacket()\n");

      BYTE* ptr = this->m_inputAccumulator.getBuffAddr();
      NATIVE_UINT_TYPE size = this->m_inputAccumulator.getBuffLength();
      // Check in extractPacket() will prevent us from ever trying to decode a packet that doesnt
      // have some data in besides a CRC:
      FW_ASSERT(size > CRC_SIZE, size);
      // CRC is last two bytes
      U16 sentCrc = (ptr[size-2] << 8 | ptr[size-1]);
      DEBUG_PRINT("UnCRCing %d bytes\n",size-CRC_SIZE);
      U16 compCrc = LLRouterComponentImpl::calculate_crc_chksum(ptr,size-CRC_SIZE);
      // check crc
      if (sentCrc != compCrc) {
          this->tlmWrite_LLR_CrcErrors(++this->m_numCrcErrors);
          this->log_WARNING_HI_LLR_DataCrcError(compCrc,sentCrc);
          DEBUG_PRINT("CRC Error! 0x%04X 0x%04X\n",sentCrc,compCrc);
          return;
      }

      ++this->m_numGoodPackets;

      DEBUG_PRINT("Packet of %d bytes received.\n",size);
      DEBUG_PRINT("CRC is: 0x%04X\n",sentCrc);
      for (NATIVE_UINT_TYPE byte = 0; byte < size; byte++) {
          DEBUG_PRINT("[0x%02X] ",ptr[byte]);
      }
      DEBUG_PRINT("\n");

      Fw::SerializeStatus stat;

      // for loop with iteration limit of buffer size
      for (NATIVE_UINT_TYPE iter = 0; iter < SERIAL_BUFFER_SIZE; iter++) {
          // Verify that there is data to decode
          if (this->m_inputAccumulator.getBuffLength() > 1) {
              // get port number
              U8 portNum;
              stat = this->m_inputAccumulator.deserialize(portNum);
              // check for deserialization error or port number too high
              if (stat != Fw::FW_SERIALIZE_OK or portNum >= this->getNum_LLPortsOut_OutputPorts()) {
                  DEBUG_PRINT("Decode portnum error %d %d\n",stat,this->getNum_LLPortsOut_OutputPorts());
                  this->tlmWrite_LLR_NumDecodeErrors(++this->m_numDecodeErrors);
                  return;
              }
              // get size
              U16 entrySize;
              stat = this->m_inputAccumulator.deserialize(entrySize);
              if (stat != Fw::FW_SERIALIZE_OK) {
                  DEBUG_PRINT("Decode size error\n");
                  this->tlmWrite_LLR_NumDecodeErrors(++this->m_numDecodeErrors);
                  return;
              }
              // get buffer for port

              NATIVE_UINT_TYPE decodeSize = entrySize;
              stat = this->m_inputAccumulator.deserialize(this->m_portBuff.getBuffAddr(),decodeSize,true);
              if (stat != Fw::FW_SERIALIZE_OK) {
                  DEBUG_PRINT("Decode data error\n");
                  this->tlmWrite_LLR_NumDecodeErrors(++this->m_numDecodeErrors);
                  return;
              }
              // set buffer to size of data
              stat = this->m_portBuff.setBuffLen(entrySize);
              if (stat != Fw::FW_SERIALIZE_OK) {
                  DEBUG_PRINT("Set setBuffLen error\n");
                  this->tlmWrite_LLR_NumDecodeErrors(++this->m_numDecodeErrors);
                  return;
              }
              // call output port
              DEBUG_PRINT("Calling port %d with %d bytes.\n",portNum,entrySize);
              if (this->isConnected_LLPortsOut_OutputPort(portNum)) {

                  if (3 == portNum) { // time GNC call
                      this->m_cycleTimer.start();
                  }

                  Fw::SerializeStatus stat = this->LLPortsOut_out(portNum,this->m_portBuff);

                  // If had issues deserializing the data, then report it:
                  if (stat != Fw::FW_SERIALIZE_OK) {

                      DEBUG_PRINT("LLPortsOut_out() serialize status error\n");
                      this->tlmWrite_LLR_NumBadSerialPortCalls(++this->m_numBadSerialPortCalls);
                      this->log_WARNING_HI_LLR_BadSerialPortCall(stat,portNum);
                      return;
                  }

              }
              else {

                  // TODO remove
#if 0
                  printf("size: %u, iter: %u, buffLeft: %u, buffLength: %u, state: %d, tLeft: %u, offset: %u\n",
                         size,iter,this->m_inputAccumulator.getBuffLeft(),
                         this->m_inputAccumulator.getBuffLength(),
                         this->m_transState, this->m_tranLeft, this->m_buffOffset);
                  printf("Send CRC is: 0x%04X, computed CRC: 0x%04X\n",sentCrc,compCrc);
                  for (NATIVE_UINT_TYPE byte = 0; byte < size; byte++) {
                      printf("[0x%02X] ",ptr[byte]);
                  }
                  printf("\n");
#endif

                  this->tlmWrite_LLR_NumInvalidPorts(++this->m_numInvalidPorts);
                  this->log_WARNING_HI_LLR_InvalidPortNum(portNum);
                  return;
              }

              // decrement size
              size -= sizeof(portNum) + sizeof(entrySize) + entrySize;

              // check for end of buffer. Should only be CRC left.
              // Do <= sizeof(U16) so we don't go into the weeds if there is an overlooked error
              DEBUG_PRINT("Packet Bytes left: %d\n",size);
              if (size <= sizeof(U16)) {
                  break;
              }
          } else {
              DEBUG_PRINT("Empty packet\n");
              break;
          } // if data
      } // for each entry
  }

  void LLRouterComponentImpl ::
     extractPacket(Fw::Buffer& buff) {
      NATIVE_UINT_TYPE buffSize = buff.getsize();
      DEBUG_PRINT("Buff size is %d\n",buffSize);
      BYTE* ptr = (BYTE*)buff.getdata();
      DEBUG_PRINT("Buff offset is %d\n",this->m_buffOffset);
      // see if we are at the end of the buffer
      if (this->m_buffOffset == buffSize-1) {
          // look for size next
          DEBUG_PRINT("Look for size\n");
          this->m_buffOffset++;
          this->m_transState = TRAN_SIZE;
      }
      // see if only one byte of the size is there
      else if ((this->m_buffOffset == buffSize-2) &&
               (TRAN_SIZE2 == this->m_transState)) {
          // look for 2nd byte of size next time
          DEBUG_PRINT("Look for size2\n");
          this->m_tempSize = ptr[this->m_buffOffset];
          this->m_buffOffset+=2;
          this->m_transState = TRAN_SIZE2;
      } else { // next byte(s) complete the size
          if ((this->m_transState != TRAN_SIZE) &&
              (this->m_transState != TRAN_SIZE2)) {
              // if sync was already found, don't move ahead
              this->m_buffOffset++;
          }

          U16 trans_size = 0u;
          if (TRAN_SIZE2 == this->m_transState) {
              // use the stored first size byte from last time
              U8 tempB[2];
              tempB[0] = ptr[this->m_buffOffset];
              tempB[1] = this->m_tempSize;
              Fw::SerialBuffer tempBuff(tempB, 2);
              tempBuff.fill();
              // TODO(mereweth) - check ser status
              tempBuff.deserialize(trans_size);
              DEBUG_PRINT("Transaction size is %d\n",trans_size);
          }
          else {
              Fw::SerialBuffer tempBuff(&ptr[this->m_buffOffset], 2);
              tempBuff.fill();
              // TODO(mereweth) - check ser status
              tempBuff.deserialize(trans_size);
              DEBUG_PRINT("Transaction size is %d\n",trans_size);
          }
          
          // If the transaction size is greater than the safety bound, reset
          if (trans_size >= MAX_TRANS_SIZE) {

              // TODO(mereweth) - add tlm back in
              ++this->m_numTooBigPktSize;
              //this->TlmOut_out(0,LL_TLM_NUM_ZERO_PKT_SIZE_HL_PKTS,++this->m_numZeroPktSize);
              this->m_transState = TRAN_WAITING;
              return;
          }

          // If the transaction size is zero, then skip this entire packet, as this can only
          // occur if there was corruption of data (data loss, corrupted bytes):
          if (trans_size == 0) {

              this->tlmWrite_LLR_NumZeroPktSize(++this->m_numZeroPktSize);
              this->m_transState = TRAN_WAITING;
              return;
          }

          this->m_tranLeft = trans_size + CRC_SIZE; // data + CRC

          // see how many bytes are left in buffer
          if (TRAN_SIZE2 == this->m_transState) {
              // used the stored first size byte from last time
              this->m_buffOffset++; // point to first byte
          }
          else {
              this->m_buffOffset+=2; // point to first byte
          }

          NATIVE_UINT_TYPE bytesLeft = buffSize - this->m_buffOffset;
          DEBUG_PRINT("bytesLeft %d\n",bytesLeft);

          // reset buffer location
          this->m_inputAccumulator.resetSer();

          NATIVE_UINT_TYPE bytesToCopy;
          // see if whole or partial transaction
          if (bytesLeft >= this->m_tranLeft) { // data + CRC
              DEBUG_PRINT("Whole!\n");
              bytesToCopy = this->m_tranLeft;
          } else { // copy the piece we have
              DEBUG_PRINT("Partial!\n");
              bytesToCopy = bytesLeft;
          }

          DEBUG_PRINT("Bytes to copy: %d\n",bytesToCopy);
          // copy bytes
          if (bytesLeft) {
              Fw::SerializeStatus stat = this->m_inputAccumulator.serialize(&ptr[this->m_buffOffset],bytesToCopy,true);
              FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
          }

          // subtract bytes from remaining transaction size
          this->m_tranLeft -= bytesToCopy;
          // add bytes to offset of current receive buffer
          this->m_buffOffset += bytesToCopy;
          // check for whole transaction
          DEBUG_PRINT("Trans left: %d\n",this->m_tranLeft);
          if (0 == this->m_tranLeft) {
              this->decodePacket();
              this->m_transState = TRAN_WAITING;
          } else {
              // switch to getting rest of transaction
              this->m_transState = TRAN_PROCESSING;
          }
      } // end if at end of buffer

  }

  void LLRouterComponentImpl ::
     extractPacket2(Fw::Buffer& buff) {
      NATIVE_UINT_TYPE buffSize = buff.getsize();
      DEBUG_PRINT("Buff size is %d\n",buffSize);
      BYTE* ptr = (BYTE*)buff.getdata() + this->m_buffOffset;
      NATIVE_UINT_TYPE bytesToCopy;
      // see if we are at the end of the buffer
      if (buffSize >= this->m_tranLeft) { // rest of buffer is present
          DEBUG_PRINT("REST!\n");
          bytesToCopy = this->m_tranLeft;
      } else { // copy partial buffer
          DEBUG_PRINT("PARTIAL2!\n");
          bytesToCopy = buffSize;
      }

      DEBUG_PRINT("Bytes to copy: %d\n",bytesToCopy);

      // copy bytes
      Fw::SerializeStatus stat = this->m_inputAccumulator.serialize(ptr,bytesToCopy,true);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

      // subtract bytes from remaining transaction size
      this->m_tranLeft -= bytesToCopy;
      // add byptes to offset of current receive buffer
      this->m_buffOffset += bytesToCopy;

      if (0 == this->m_tranLeft) {
          this->decodePacket();
          this->m_transState = TRAN_WAITING;
      }
  }

  void LLRouterComponentImpl ::
    SerReadPort_handler(
      NATIVE_INT_TYPE portNum, /*!< The port number*/
      Fw::Buffer &serBuffer, /*!< Buffer containing data*/
      Drv::SerialReadStatus &status /*!< Status of read*/
  ) {

      // flowing flag is set by Sched handler when it detects we are in preflight
      // mode. This avoids buffer overflows at startup if we reboot while LL was still sending
      // data.
      if (this->m_flowing) {

          if (this->m_LLRMode == LLR_NORMAL) {

              ++this->m_numPackets;

              // Return if there is bad status:
              if (status != Drv::SER_OK) {

                  // Go back to waiting state to look for the next sync marker:
                  this->m_transState = TRAN_WAITING;

                  LLR_ReceiveError error = LLR_OTHER_ERR;
                  switch (status) {

                      case Drv::SER_PARITY_ERR:
                          error = LLR_PARITY;
                          break;

                      case Drv::SER_NO_BUFFERS:
                          error = LLR_NO_BUFFERS;
                          break;

                      case Drv::SER_BUFFER_TOO_SMALL:
                          error = LLR_BUFFER_TOO_SMALL;
                          break;

                      case Drv::SER_OTHER_ERR:
                          error = LLR_OTHER_ERR;
                          break;

                      default:
                          FW_ASSERT(0,status);
                          break;
                  }

                  this->log_WARNING_HI_LLR_DataReceiveError(error);
                  this->tlmWrite_LLR_NumSerialErrors(++this->m_numSerialErrors);

              }
              // No serial errors:
              else {

    #if UART_LOOPBACK == 0
                  this->processSerialBuffer(serBuffer);

    #else
                  printf("Received %d bytes.\n",serBuffer.getsize());
                  this->SerWritePort_out(0,serBuffer);
    #endif
              }
          } else {
              // In patch mode

              if (status == Drv::SER_OK) {
                  BYTE* data_ptr = (BYTE*)serBuffer.getdata();

                  // Don't send packets until after we have rebooted
                  if (this->m_LLRebootState == REBOOT_NONE) {
                      for (U32 i = 0; i < serBuffer.getsize(); i++) {

                          if (this->m_seenMagicString || this->m_legacy_start_byte) {
                              if (data_ptr[i] == PATCH_ACK_CHAR) {
                                  DEBUG_PRINT("Received Ack\n");
                                  runPatchFSM();
                                  break;
                              }
                          } else {
                              for (U32 idx = 0; idx < sizeof(BOOTLOADER_MAGIC_STRING) - 1; idx++) {
                                  this->m_magicStringBuffer[idx] = this->m_magicStringBuffer[idx + 1];
                              }

                              this->m_magicStringBuffer[sizeof(BOOTLOADER_MAGIC_STRING) - 1] = data_ptr[i];

                              if (memcmp(this->m_magicStringBuffer, BOOTLOADER_MAGIC_STRING, sizeof(BOOTLOADER_MAGIC_STRING)) == 0) {
                                  this->m_seenMagicString = true;
                                  DEBUG_PRINT("Received Magic String\n");
                                  runPatchFSM();
                                  break;
                              }
                          }
                      }
                  }
              }
          }
      }
      // send buffer back to driver after resetting size
      serBuffer.setsize(SERIAL_BUFFER_SIZE);
      this->SerialBufferSend_out(0,serBuffer);
      this->m_newPacket = true;
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void LLRouterComponentImpl ::
    LLR_CYCLE_SERIAL_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        bool on
    )
  {
      this->m_schedSerial = on;
      this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
  }

#ifdef BUILD_UT
  void LLRouterComponentImpl::SerialDriverBuffer::operator=(const Fw::SerializeBufferBase& other) {
      this->resetSer();
      this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
  }

  LLRouterComponentImpl::SerialDriverBuffer::SerialDriverBuffer(
          const Fw::SerializeBufferBase& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.getBuffAddr(),other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  LLRouterComponentImpl::SerialDriverBuffer::SerialDriverBuffer(
          const LLRouterComponentImpl::SerialDriverBuffer& other) : Fw::SerializeBufferBase() {
      FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
      memcpy(this->m_buff,other.m_buff,other.getBuffLength());
      this->setBuffLen(other.getBuffLength());
  }

  LLRouterComponentImpl::SerialDriverBuffer::SerialDriverBuffer(): Fw::SerializeBufferBase() {

  }

#endif

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined commands
  // ----------------------------------------------------------------------

  void LLRouterComponentImpl::LLR_SET_PATCH_MODE_cmdHandler(
          FwOpcodeType opCode, /*!< The opcode*/
          U32 cmdSeq, /*!< The command sequence number*/
          LLRPatchMode Mode
  ) {
      m_numPatchBytesSent = 0;

      if (Mode == LLR_NORMAL) {

          if (m_LLRMode == LLR_PATCH) {
              this->finishUpdate();
          }

          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      } else if (Mode == LLR_PATCH) {

          setRouterPatchMode(LLR_PATCH);
          m_LLRPatchState = PATCH_STATE_DONE;

          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      } else {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_VALIDATION_ERROR);
      }

  }


  void LLRouterComponentImpl::LLR_LOAD_PATCH_FILE_cmdHandler(
        FwOpcodeType opCode, /*!< The opcode*/
        U32 cmdSeq, /*!< The command sequence number*/
        const Fw::CmdStringArg& file, /*!< Patch file*/
        LL updatingLL, /*!< The LL to update*/
        bool legacyBootloader, /*!< Update LLs with an old bootloader*/
        bool forceboot, /*!< Tell the bootloader to ignore CRC32 checks on startup*/
        U64 disabled_selftests /*!< Bitmask of tests to disable on startup. A 1 indicates that the test should not be run*/
  ) {

      if (m_LLRMode == LLR_NORMAL) {
          setRouterPatchMode(LLR_PATCH);
      }

      if (file.length() >= MAX_FILENAME_SIZE + MAX_PATH_SIZE) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_VALIDATION_ERROR);
          return;
      }

      memset(m_patchFilename, 0, sizeof(m_patchFilename));
      memcpy(m_patchFilename, file.toChar(), file.length());

      if (m_patchFile.open((char*)m_patchFilename, Os::File::OPEN_READ) != Os::File::OP_OK) {
          Fw::LogStringArg openFileString((const char*)m_patchFile.getLastErrorString());
          this->log_WARNING_HI_LLR_PatchError(LLR_PERR_OPEN, m_patchFile.getLastError(), openFileString);
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_VALIDATION_ERROR);
          return;
      }

      U64 filesize;
      Os::FileSystem::getFileSize((const char*)m_patchFilename, filesize);

      Fw::LogStringArg logString((const char*)m_patchFilename);
      log_ACTIVITY_HI_LLR_PatchFileOpen(logString, filesize);



      beginUpdate(updatingLL, legacyBootloader, forceboot, disabled_selftests);

      m_numPatchBytesSent = 0;
      tlmWrite_LLR_PatchProgress(m_numPatchBytesSent);

      m_patching_opcode = opCode;
      m_patching_cmdSeq = cmdSeq;

  }

  void LLRouterComponentImpl::setRouterPatchMode(LLRPatchMode newMode) {

      if (newMode != m_LLRMode) {
          Fw::LogStringArg logPatchString("Patch");
          Fw::LogStringArg logNormalString("Normal");

          switch(newMode) {
              case LLR_PATCH:
                  log_ACTIVITY_HI_LLR_PatchModeChange(logPatchString);
                  m_LLRMode = newMode;
                  break;

              case LLR_NORMAL:
                  log_ACTIVITY_HI_LLR_PatchModeChange(logNormalString);
                  m_LLRMode = newMode;
                  break;

              default:
                  // Invalid mode, ignore
                  break;
          }
      }
  }

  void LLRouterComponentImpl::runPatchFSM() {

      int bytes_written = 0;

      // Reset the timeout counter
      m_patchTimeoutCounter = PATCH_TIMEOUT;

      DEBUG_PRINT("FSM in %u\n", m_LLRPatchState);

      switch (m_LLRPatchState) {

          case PATCH_STATE_ADDRESS:

              log_ACTIVITY_HI_LLR_FirstACK();

              sendAddressPacket();
              m_LLRPatchState = PATCH_STATE_DATA;
              break;

          case PATCH_STATE_DATA:

              // Write the next patch
              bytes_written = sendDataPacket();

              if (bytes_written < 0) {
                  DEBUG_PRINT("Error Writing Data Packet\n");
              } else {

                  // Check if we cross a 100 KB boundary
                  if ((this->m_numPatchBytesSent / (1024 * 100)) !=
                      ((this->m_numPatchBytesSent + bytes_written) / (1024 * 100))) {

                      int kilobytesWritten = (this->m_numPatchBytesSent + bytes_written) / (1024);

                      log_ACTIVITY_HI_LLR_PatchEvent(kilobytesWritten);

                  }

                  this->m_patchFileSize += bytes_written;
                  this->m_numPatchBytesSent += bytes_written;

                  tlmWrite_LLR_PatchProgress(this->m_numPatchBytesSent);
                  

                  if (bytes_written == 0) {

                      // If there are no more bytes to send, send the metadata
                      sendMetadataPacket();
                      m_LLRPatchState = PATCH_STATE_EOF;
                  }
              }

              break;
          case PATCH_STATE_EOF:

              // We have received an ACK after sending metadata
              sendEoFPacket();

              // Close the file handle and return to normal operation
              finishUpdate();

              log_ACTIVITY_HI_LLR_PatchComplete();

              this->cmdResponse_out(m_patching_opcode, m_patching_cmdSeq, Fw::COMMAND_OK);

              m_LLRPatchState = PATCH_STATE_DONE;

              break;
          case PATCH_STATE_DONE:

              // Nothing to do here.
              break;
      }

  }


  enum UPacketCmd {
      UPACKET_CMD_DATA = 0,
      UPACKET_CMD_EOF = 1,
      UPACKET_CMD_SET_ADDR = 2,
      UPACKET_CMD_METADATA = 3
  };

  struct UPacket_t {
      uint8_t start_byte;
      uint8_t cmd;
      uint16_t length;
      BYTE* data;
      uint8_t checksum;
  };

  struct Metadata_t {
      uint32_t crc32;
      uint32_t file_size;
      uint32_t start_addr;
      uint32_t flags;
      uint64_t selftests;
  }__attribute__((__packed__));

  static int getPacketLength(struct UPacket_t* packet) {
      return 1 + HLProc::LLRouterComponentImpl::HEADER_LENGTH + packet->length + 1;
  }

  static void writeAsBigEndian(BYTE* out, uint32_t in) {
      out[0] = (in >> 24) & 0xFF;
      out[1] = (in >> 16) & 0xFF;
      out[2] = (in >> 8) & 0xFF;
      out[3] = in & 0xFF;
  }

  // Writes a packet structure into a buffer
  static void writePacketToMem(struct UPacket_t* packet, BYTE* buffer) {

      uint8_t checksum = HLProc::LLRouterComponentImpl::START_BYTE;
      checksum += packet->cmd;
      checksum += (packet->length >> 8) & 0xFF;
      checksum += packet->length & 0xFF;
      int i;
      for (i = 0; i < packet->length; i++) {
          checksum += packet->data[i];
      }

      checksum = (checksum ^ 0xFF) + 1;
      packet->checksum = checksum;

      buffer[0] = HLProc::LLRouterComponentImpl::START_BYTE;
      buffer[1] = packet->cmd;
      buffer[2] = (packet->length >> 8) & 0xFF;
      buffer[3] = packet->length & 0xFF;
      if (packet->length > 0) {
          memcpy(&buffer[4], packet->data, packet->length);
      }
      buffer[1 + packet->length + HLProc::LLRouterComponentImpl::HEADER_LENGTH] = packet->checksum;
  }

  static void getDataPacket(BYTE* data, int length, BYTE* packetBuffer, U32* bufferLen) {

      struct UPacket_t packet;
      packet.cmd = UPACKET_CMD_DATA;
      packet.length = length;
      packet.data = data;

      //assert(*bufferLen >= getPacketLength(&packet));

      writePacketToMem(&packet, packetBuffer);

      // Return the buffer length in bufferLen
      *bufferLen = getPacketLength(&packet);
  }

  static void getAddressPacket(U32 address, BYTE* packetBuffer, U32* bufferLen) {

      BYTE temp_data[4];

      struct UPacket_t packet;
      packet.cmd = UPACKET_CMD_SET_ADDR;
      packet.length = 4;

      writeAsBigEndian(temp_data, address);

      packet.data = temp_data;

      //assert(*bufferLen >= getPacketLength(&packet));

      writePacketToMem(&packet, packetBuffer);

      *bufferLen = getPacketLength(&packet);
  }

  static void getEoFPacket(BYTE* packetBuffer, U32* bufferLen) {

      struct UPacket_t packet;
      packet.cmd = UPACKET_CMD_EOF;
      packet.length = 0;
      packet.data = 0;

      //assert(*bufferLen >= getPacketLength(&packet));

      writePacketToMem(&packet, packetBuffer);

      *bufferLen = getPacketLength(&packet);
  }


  static void getMetadataPacket(U32 checksum, int bytes_sent, U32 app_start_addr, U32 flags, U64 selftests, BYTE* packetBuffer, U32* bufferLen) {

      struct Metadata_t mdata;

      BYTE temp_data[sizeof(struct Metadata_t)];
      struct UPacket_t packet;
      packet.cmd = UPACKET_CMD_METADATA;
      packet.length = sizeof(struct Metadata_t);

      mdata.crc32 = checksum;
      mdata.file_size = bytes_sent;
      mdata.start_addr = app_start_addr;
      mdata.flags = flags;
      mdata.selftests = selftests;

      writeAsBigEndian(temp_data, mdata.crc32);
      writeAsBigEndian(&temp_data[4], mdata.file_size);
      writeAsBigEndian(&temp_data[8], mdata.start_addr);
      writeAsBigEndian(&temp_data[12], mdata.flags);
      writeAsBigEndian(&temp_data[16], (mdata.selftests >> 32) & 0xFFFFFFFF);
      writeAsBigEndian(&temp_data[20], mdata.selftests & 0xFFFFFFFF);

      packet.data = temp_data;

      //assert(*bufferLen >= getPacketLength(&packet));

      writePacketToMem(&packet, packetBuffer);

      *bufferLen = getPacketLength(&packet);
  }

  static U32 calculateCRC32FromFile(Os::File file, U8* filename) {

      const U32 maxChunkSize = 32;
      const U32 initialSeed = 0xFFFFFFFF;

      // Seek to beginning of file
      file.seek(0, true);

      U8 file_buffer[maxChunkSize];

      bool endOfFile = false;

      U32 seed = initialSeed;

      while (!endOfFile) {

          int chunkSize = maxChunkSize;

          if (file.read(file_buffer, chunkSize, false) == Os::File::OP_OK) {
              // chunkSize modified by file.read

              if (chunkSize == 0) {
                  endOfFile = true;
                  continue;
              }

              int chunkIdx = 0;

              while (chunkIdx < chunkSize) {
                seed = update_crc_32(seed, file_buffer[chunkIdx]);
                chunkIdx++;
              }

          } else {
              DEBUG_PRINT("Error Reading from file in calcCRC32\n");
              return 0;
          }
      }

      return seed;
  }

  void LLRouterComponentImpl::writePacket(BYTE* output_buffer, U32 buffer_len) {

      if (buffer_len < SERIAL_BUFFER_SIZE) {

          this->m_patchOutputBuffer.resetSer();
          this->m_patchOutputBuffer.serialize(output_buffer, buffer_len, true);

          this->m_patchOutputBuffObj.setdata((U64)this->m_patchOutputBuffer.getBuffAddr());
          this->m_patchOutputBuffObj.setsize(this->m_patchOutputBuffer.getBuffLength());

          DEBUG_PRINT("Sending %d bytes as update\n", this->m_patchOutputBuffer.getBuffLength());
          this->SerWritePort_out(0,this->m_patchOutputBuffObj);
      } else {
          DEBUG_PRINT("Unable to send: %u < %u", m_outputBuffer.getBuffLength(), buffer_len);
      }
  }

  void LLRouterComponentImpl::beginUpdate(LL updatingLL, bool legacyBootloader, bool forceboot, U64 disabled_selftests) {

      m_patchingLL = updatingLL;
      m_patchTimeoutCounter = PATCH_TIMEOUT;

      //TODO(mereweth) - set m_primeLL
      m_seenMagicString = false;
      m_legacy_start_byte = legacyBootloader;
      memset(m_magicStringBuffer, 0, sizeof(m_magicStringBuffer));

      m_forcebootFlag = forceboot;
      m_selftests = ~disabled_selftests;

      // If necessary, switch LLs
      if (m_primeLL != m_patchingLL) {
        //TODO(mereweth)
      }
      
      // Set the patch bit for patching LL
      if (updatingLL == LL_A) {
        //TODO(mereweth)
      } else {
        //TODO(mereweth)
      }

      // Reset the R5 into the bootloader
      this->m_LLRebootState = REBOOT_FOR_PATCHING_SET;

      // Initialize state
      m_LLRPatchState = PATCH_STATE_ADDRESS;
  }

  void LLRouterComponentImpl::sendAddressPacket() {

      BYTE packet_buffer[MAX_TOTAL_PACKET_LENGTH];

      U32 packet_buffer_size = sizeof(packet_buffer);

      getAddressPacket(DEFAULT_R5_START_ADDR, packet_buffer, &packet_buffer_size);

      DEBUG_PRINT("Sending address update packet\n");

      // Writer packet buffer out serial port
      this->writePacket(packet_buffer, packet_buffer_size);
  }

  int LLRouterComponentImpl::sendDataPacket() {

      U8 data_buffer[MAX_DATA_PACKET_LENGTH];
      BYTE packet_buffer[MAX_TOTAL_PACKET_LENGTH];

      int buffer_size = sizeof(data_buffer);

      if (m_patchFile.read(&data_buffer, buffer_size, false) != Os::File::OP_OK) {
          Fw::LogStringArg readFileString((const char*)m_patchFile.getLastErrorString());
          log_WARNING_HI_LLR_PatchError(LLR_PERR_READ, m_patchFile.getLastError(), readFileString);
          return -1;
      }

      if (buffer_size == 0) {
          // End of file
          return 0;
      }

      U32 packet_buffer_size = sizeof(packet_buffer);

      getDataPacket(data_buffer, buffer_size, packet_buffer, &packet_buffer_size);

      DEBUG_PRINT("Sending data update packet\n");

      // Write packet buffer out serial port
      this->writePacket(packet_buffer, packet_buffer_size);

      return buffer_size;
  }

  void LLRouterComponentImpl::sendMetadataPacket() {

      BYTE packet_buffer[MAX_TOTAL_PACKET_LENGTH];

      // Calculate crc32
      m_patchFile.seek(0);
      U32 crc32 = calculateCRC32FromFile(m_patchFile, m_patchFilename);

      U32 packet_buffer_size = sizeof(packet_buffer);

      U32 flags = 0;
      if (this->m_forcebootFlag) {
          flags |= METADATA_FLAGS_FORCEBOOT;
      }

      getMetadataPacket(crc32,
                        this->m_numPatchBytesSent,
                        DEFAULT_R5_START_ADDR,
                        flags,
                        this->m_selftests,
                        packet_buffer,
                        &packet_buffer_size);

      DEBUG_PRINT("Sending metadata update packet\n");

      // Write packet buffer out serial port
      this->writePacket(packet_buffer, packet_buffer_size);
  }

  void LLRouterComponentImpl::sendEoFPacket() {

      BYTE packet_buffer[MAX_TOTAL_PACKET_LENGTH];

      U32 packet_buffer_size = sizeof(packet_buffer);

      getEoFPacket(packet_buffer, &packet_buffer_size);

      DEBUG_PRINT("Sending EoF update packet\n");

      // Write packet buffer out serial port
      this->writePacket(packet_buffer, packet_buffer_size);
  }

  void LLRouterComponentImpl::finishUpdate() {

      DEBUG_PRINT("Done with update\n");

      setRouterPatchMode(LLR_NORMAL);
      m_LLRPatchState = PATCH_STATE_DONE;
      m_patchFile.close();

      // Reset the prime LL
      if (m_primeLL != m_patchingLL) {
          //TODO(mereweth)
      }

      // Clear the patch mode for both LLs just to be safe
      //TODO(mereweth)
      this->m_LLRebootState = REBOOT_ALL_SET;
  }

} // end namespace HLProc
