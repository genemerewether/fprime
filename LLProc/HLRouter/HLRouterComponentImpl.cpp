// ====================================================================== 
// \title  HLRouterImpl.cpp
// \author tcanham
// \brief  cpp file for HLRouter component implementation class
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


#include <LLProc/HLRouter/HLRouterComponentImpl.hpp>
#include <LLProc/HLRouter/HLRouterComponentImplCfg.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>
#include <string.h> // memcpy
#include <Common/Cfg/QuestConstants.hpp>
//#include <LLProc/LLProcDebug/LLProcDebugComponentImplCfg.hpp>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#include <Utils/Hash/libcrc/lib_crc.h> // borrow CRC

#ifdef __cplusplus
}
#endif // __cplusplus

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#define DEBUG_PRINT(x,...)

namespace LLProc {

  //!< Shouldn't be more loop iterations than the number of bytes in the buffer
  //!< and it can be higher than 5 b/c of off-nominal conditions
  static const NATIVE_UINT_TYPE MAX_ITERS = SERIAL_BUFFER_SIZE;

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  HLRouterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    HLRouterComponentImpl(
        const char *const compName
    ) :
      HLRouterComponentBase(compName)
#else
    HLRouterImpl(void)
#endif
    ,m_transState(TRAN_WAITING)
    ,m_tranLeft(0)
    ,m_buffOffset(0)
    ,m_numResyncs(0)
    ,m_numDecodeErrors(0)
    ,m_numPackets(0)
    ,m_numCrcErrors(0)
    ,m_numGoodPackets(0)
    ,m_numInvalidPorts(0)
    ,m_numBadSerialPortCalls(0)
    ,m_numOutputBufferOverflows(0)
    ,m_numZeroPktSize(0)
    ,m_maxSerialSendSize(0)
  {

  }

  void HLRouterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    HLRouterComponentBase::init(instance);

    // Initialize memory buffer objects
    this->m_inputBuffObj.setdata((U64)this->m_inputBuff);
    this->m_inputBuffObj.setsize(sizeof(this->m_inputBuff));

    this->m_outputBuffObj.setdata((U64)this->m_outputBuffer.getBuffAddr());
    this->m_outputBuffObj.setsize(this->m_outputBuffer.getBuffLength());
    // set up outgoing serial buffer for the first cycle
    this->resetSerOutBuff();
  }

  HLRouterComponentImpl ::
    ~HLRouterComponentImpl(void)
  {
  }

  void HLRouterComponentImpl ::
    resetSerOutBuff(void) {
      // reset buffer
      this->m_outputBuffer.resetSer();
      Fw::SerializeStatus stat;
      // serialize start of frame character for next round of data
      stat = this->m_outputBuffer.serialize(Cfg::UART_SYNC_BYTE);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
      // serialize placeholder size - will be filled in when packet is complete
      stat = this->m_outputBuffer.serialize((U8)0);
      FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
  }

  U16 HLRouterComponentImpl ::
      calculate_crc_chksum(const U8 *buff, U8 size) {

      FW_ASSERT(buff != NULL);

      U16 crc = 0xFFFF;

      for (U32 i = 0; i < size; ++i) {
          crc = update_crc_16(crc, buff[i]);
      }

      return crc;
  }

  void HLRouterComponentImpl ::
    processSerialBuffer(Fw::Buffer& buff) {

      // Process serial data
      // Get amount of data received
      NATIVE_UINT_TYPE buffSize = buff.getsize();

      this->m_buffOffset = 0; // reset receive buffer offset
      for (NATIVE_UINT_TYPE iter = 0; iter < MAX_ITERS; iter++) {
          DEBUG_PRINT("IT: %d\n",iter);
          if (this->m_buffOffset < buffSize) {
              BYTE* ptr = reinterpret_cast<BYTE*>(buff.getdata());
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

                                      // TODO remove
#if 0
                                      printf("buffOffset: %d, start: %u\n",this->m_buffOffset,start);
                                      //char str[LLProc::LL_DEBUG_TEXT_SIZE];
                                      for (U32 i = 0; i < buffSize; ++i) {
                                          printf("%u: 0x%x\n",i,ptr[i]);
                                          //(void)sscanf(str,"%u: 0x%x\n",&i,ptr+i);
                                      }
                                      //this->DebugOut_out(0,(U8 *)str,LLProc::LL_DEBUG_TEXT_SIZE);
#endif

                                      // TODO(mereweth) - add tlm back in
                                      ++this->m_numResyncs;
                                      //this->TlmOut_out(0,LL_TLM_UART_RECV_RESYNC_ERRS,++this->m_numResyncs);
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
                      // if it gets here, first byte is size
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


  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void HLRouterComponentImpl ::
    Sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      switch (context) {
          case HLRTR_SCHED_UART_SEND:
              // Send out serial data if there is data beyond SYNC and size bytes
              if (this->m_outputBuffer.getBuffLength() > 2*sizeof(BYTE)) {
                  // set size
                  BYTE* ptr = this->m_outputBuffer.getBuffAddr();
                  // poke in size
                  ptr[1] = static_cast<BYTE>(this->m_outputBuffer.getBuffLength() - 2*sizeof(BYTE)); // data minus SYNC and size byte
                  // set CRC - CRC starts after sync and size bytes
                  U16 crc = HLRouterComponentImpl::calculate_crc_chksum(&ptr[2*sizeof(BYTE)],this->m_outputBuffer.getBuffLength()-2*sizeof(BYTE));
                  Fw::SerializeStatus stat = this->m_outputBuffer.serialize(crc);
                  FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);
                  // set the serial buffer size
                  NATIVE_UINT_TYPE outBuffSzie = this->m_outputBuffer.getBuffLength();
                  this->m_outputBuffObj.setsize(outBuffSzie);
                  DEBUG_PRINT("Sending %d bytes\n",outBuffSzie);

                  if (outBuffSzie > this->m_maxSerialSendSize) {
                      this->m_maxSerialSendSize = outBuffSzie;
                      // TODO(mereweth) - add tlm back in
                      //this->TlmOut_out(0, LL_TLM_MAX_LL_PKT_SIZE, this->m_maxSerialSendSize);
                  }

                  // send data to driver
                  this->SerWritePort_out(0,this->m_outputBuffObj);
                  // reset buffer for next RTI
                  this->resetSerOutBuff();
              }
              break;
          case HLRTR_SCHED_UART_RECEIVE: {
                  // Get current serial data
                  Drv::SerialReadStatus status;
                  // reset size of buffer to tell driver how much space.
                  this->m_inputBuffObj.setsize(SERIAL_BUFFER_SIZE);
                  // Note that status is not updated by the R5UartDriver, so ignoring it
                  this->SerReadPort_out(0,this->m_inputBuffObj,status);
                  if (this->m_inputBuffObj.getsize()) {
                      // TODO(mereweth) - add tlm back in
                      ++this->m_numPackets;
                      //this->TlmOut_out(0,LL_TLM_NUM_HL_PKTS,++this->m_numPackets);
                  }
                  this->processSerialBuffer(this->m_inputBuffObj);
              }
              break;
          default:
              FW_ASSERT(0,context);
              break;
      }

  } // end Sched_handler

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined serial input ports
  // ----------------------------------------------------------------------

  void HLRouterComponentImpl ::
    LLPortsIn_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
    )
  {
      // Add to outgoing buffer
      DEBUG_PRINT("Port %d in. Size: %d\n",portNum,Buffer.getBuffLength());

      // Check if we can fit the data in the output buffer:
      const NATIVE_UINT_TYPE total_size = sizeof(U8) + sizeof(U8) + Buffer.getBuffLength();

      if (total_size > Buffer.getBuffLength() && // Checking for overflow in addition above
          total_size <= this->m_outputBuffer.getBuffSerLeft()) {

          // Add port number
          Fw::SerializeStatus stat = this->m_outputBuffer.serialize((U8)portNum);
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

          // Add size of buffer
          stat = this->m_outputBuffer.serialize((U8)Buffer.getBuffLength());
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

          // Add data
          stat = this->m_outputBuffer.serialize(Buffer.getBuffAddr(),Buffer.getBuffLength(),true);
          FW_ASSERT(Fw::FW_SERIALIZE_OK == stat,stat);

      }
      else {

          DEBUG_PRINT("Output buffer overflow! Port %u in. Size: %u Left: %u\n",
                      portNum,total_size,this->m_outputBuffer.getBuffSerLeft());
          // TODO(mereweth) - add tlm back in
          ++this->m_numOutputBufferOverflows;
          //this->TlmOut_out(0,LL_TLM_NUM_OUT_BUFFER_OVERFLOW_HL_PKTS,++this->m_numOutputBufferOverflows);
      }

  }

  void HLRouterComponentImpl ::
     decodePacket(void) {

      DEBUG_PRINT("** decodePacket()\n");

      BYTE* ptr = this->m_inputAccumulator.getBuffAddr();
      NATIVE_UINT_TYPE size = this->m_inputAccumulator.getBuffLength();
      // Check in extractPacket() will prevent us from ever trying to decode a packet that doesnt
      // have some data in besides a CRC:
      FW_ASSERT(size > CRC_SIZE, size);
      U16 sentCrc = (ptr[size-2] << 8 | ptr[size-1]);
      DEBUG_PRINT("UnCRCing %d bytes\n",size-CRC_SIZE);
      U16 compCrc = HLRouterComponentImpl::calculate_crc_chksum(ptr,size-CRC_SIZE);
      // check crc
      if (sentCrc != compCrc) {
          // TODO(mereweth) - add tlm back in
          ++this->m_numCrcErrors;
          //this->TlmOut_out(0,LL_TLM_UART_RECV_CRC_ERRS,++this->m_numCrcErrors);
          DEBUG_PRINT("CRC Error! 0x%04X 0x%4X\n",sentCrc,compCrc);
          return;
      }

      // TODO(mereweth) - add tlm back in
      ++this->m_numGoodPackets;
      //this->TlmOut_out(0,LL_TLM_NUM_GOOD_HL_PKTS,++this->m_numGoodPackets);

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
              if (stat != Fw::FW_SERIALIZE_OK or portNum >= this->getNum_HLPortsOut_OutputPorts()) {
                  DEBUG_PRINT("Decode portnum error %d %d\n",stat,this->getNum_HLPortsOut_OutputPorts());
                  // TODO(mereweth) - add tlm back in
                  ++this->m_numDecodeErrors;
                  //this->TlmOut_out(0,LL_TLM_NUM_DECODE_ERRS_HL_PKTS,++this->m_numDecodeErrors);
                  return;
              }
              // get size
              U8 entrySize;
              stat = this->m_inputAccumulator.deserialize(entrySize);
              if (stat != Fw::FW_SERIALIZE_OK) {
                  DEBUG_PRINT("Decode size error\n");
                  // TODO(mereweth) - add tlm back in
                  ++this->m_numDecodeErrors;
                  //this->TlmOut_out(0,LL_TLM_NUM_DECODE_ERRS_HL_PKTS,++this->m_numDecodeErrors);
                  return;
              }
              // get buffer for port

              NATIVE_UINT_TYPE decodeSize = entrySize;
              stat = this->m_inputAccumulator.deserialize(this->m_portBuff.getBuffAddr(),decodeSize,true);
              if (stat != Fw::FW_SERIALIZE_OK) {
                  DEBUG_PRINT("Decode data error\n");
                  // TODO(mereweth) - add tlm back in
                  ++this->m_numDecodeErrors;
                  //this->TlmOut_out(0,LL_TLM_NUM_DECODE_ERRS_HL_PKTS,++this->m_numDecodeErrors);
                  return;
              }
              // set buffer to size of data
              stat = this->m_portBuff.setBuffLen(entrySize);
              if (stat != Fw::FW_SERIALIZE_OK) {
                  DEBUG_PRINT("Set setBuffLen error\n");
                  // TODO(mereweth) - add tlm back in
                  ++this->m_numDecodeErrors;
                  //this->TlmOut_out(0,LL_TLM_NUM_DECODE_ERRS_HL_PKTS,++this->m_numDecodeErrors);
                  return;
              }
              // call output port
              DEBUG_PRINT("Calling port %d with %d bytes.\n",portNum,entrySize);

              if (this->isConnected_HLPortsOut_OutputPort(portNum)) {

                  Fw::SerializeStatus stat = this->HLPortsOut_out(portNum,this->m_portBuff);

                  // If had issues deserializing the data, then report it:
                  if (stat != Fw::FW_SERIALIZE_OK) {

                      DEBUG_PRINT("HLPortsOut_out() serialize status error\n");
                      // TODO(mereweth) - add tlm back in
                      ++this->m_numBadSerialPortCalls;
                      //this->TlmOut_out(0,LL_TLM_NUM_BAD_PORT_CALLS_HL_PKTS,++this->m_numBadSerialPortCalls);
                      return;
                  }
              }
              else {

                  // TODO(mereweth) - add tlm back in
                  ++this->m_numInvalidPorts;
                  //this->TlmOut_out(0,LL_TLM_NUM_INVALID_PORTS_HL_PKTS,++this->m_numInvalidPorts);
                  DEBUG_PRINT("Invalid port number: %u\n",portNum);
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
      } // for all entries
  }

  void HLRouterComponentImpl ::
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
      } else { // next byte is size
          if (this->m_transState != TRAN_SIZE) { // if sync was already found, don't move ahead
              this->m_buffOffset++;
          }

          BYTE trans_size = ptr[this->m_buffOffset];
          DEBUG_PRINT("Transaction size is %d\n",trans_size);

          // If the transaction size is zero, then skip this entire packet, as this can only
          // occur if there was corruption of data (data loss, corrupted bytes):
          if (trans_size == 0) {

              // TODO(mereweth) - add tlm back in
              ++this->m_numZeroPktSize;
              //this->TlmOut_out(0,LL_TLM_NUM_ZERO_PKT_SIZE_HL_PKTS,++this->m_numZeroPktSize);
              this->m_transState = TRAN_WAITING;
              return;
          }

          this->m_tranLeft = trans_size + CRC_SIZE; // data + CRC

          // see how many bytes are left in buffer
          this->m_buffOffset++; // point to first byte

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

  void HLRouterComponentImpl ::
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

#ifdef BUILD_UT
  void HLRouterComponentImpl::SerialDriverBuffer::operator=(const Fw::SerializeBufferBase& other) {
	  this->resetSer();
	  this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
  }

  HLRouterComponentImpl::SerialDriverBuffer::SerialDriverBuffer(
		  const Fw::SerializeBufferBase& other) : Fw::SerializeBufferBase() {
	  FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
	  memcpy(this->m_buff,other.getBuffAddr(),other.getBuffLength());
	  this->setBuffLen(other.getBuffLength());
  }

  HLRouterComponentImpl::SerialDriverBuffer::SerialDriverBuffer(
		  const HLRouterComponentImpl::SerialDriverBuffer& other) : Fw::SerializeBufferBase() {
	  FW_ASSERT(sizeof(this->m_buff)>= other.getBuffLength(),sizeof(this->m_buff),other.getBuffLength());
	  memcpy(this->m_buff,other.m_buff,other.getBuffLength());
	  this->setBuffLen(other.getBuffLength());
  }

  HLRouterComponentImpl::SerialDriverBuffer::SerialDriverBuffer(): Fw::SerializeBufferBase() {

  }

#endif


} // end namespace LLProc
