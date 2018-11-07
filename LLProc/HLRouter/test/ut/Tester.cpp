// ====================================================================== 
// \title  HLRouter.hpp
// \author tcanham
// \brief  cpp file for HLRouter test harness implementation class
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

#include "Tester.hpp"
#include <Fw/Types/SerialBuffer.hpp>
#include <Common/Cfg/QuestConstants.hpp>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

#define FAKE_CRC 0x5A

namespace LLProc {

    // ----------------------------------------------------------------------
    // Construction and destruction
    // ----------------------------------------------------------------------

    Tester::Tester(void) :
#if FW_OBJECT_NAMES == 1
                    HLRouterGTestBase("Tester", MAX_HISTORY_SIZE), component(
                            "HLRouter")
#else
    HLRouterGTestBase(MAX_HISTORY_SIZE),
    component()
#endif
	 ,m_serPortWritten(false)
     ,m_numPacketsSent(0)
     ,m_fromSerPortHistorySize(0)
    {
        this->initComponents();
        this->connectPorts();

        // add buffer to Fw::Buffer
        this->m_serBufferObject.setdata((U64) this->m_serBuffer);

        // create serialized port history
        this->m_serPortHistory =
              new History<serPortHistory>(MAX_HISTORY_SIZE);
    }

    Tester::~Tester(void) {
    	delete this->m_serPortHistory;
    }

    // ----------------------------------------------------------------------
    // Tests
    // ----------------------------------------------------------------------

    void Tester::nominalOneFrame(void) {
    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        U8 data[10];
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();

        ASSERT_EQ((U32)1,this->m_fromSerPortHistorySize);
        // should be no decode errors
        this->verifyNoErrors(0,1,1);
        this->verifyPortCall(0,10,0);
    }

    void Tester::nominalTwoFrames(void) {

        U8 data[18];

    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,10);
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();
        ASSERT_EQ((U32)1,this->m_fromSerPortHistorySize);
        this->verifyPortCall(0,10,0);

    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();
        ASSERT_EQ((U32)1,this->m_fromSerPortHistorySize);
        this->verifyPortCall(0,18,0);
        // should be no decode errors
        this->verifyNoErrors(2,3,2);

    }

    void Tester::nominalOneSplit(void) {
    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        U8 data[10];
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer(-1,8);

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();

        // shouldn't have been called yet
        ASSERT_EQ((U32)0,this->m_fromSerPortHistorySize);

        this->resetBuffer();
        this->addBuffer(8,-1);

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();
        this->verifyPortCall(0,10,0);

        // should be no decode errors
        this->verifyNoErrors(1,2,1);

    }

    void Tester::nominalOneSplitTwice(void) {
        // reset test buffer
        this->m_testBuffer.resetSer();
        // reset output buffer
        this->resetBuffer();
        // create a port call
        this->startBuffer();
        U8 data[10];
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer(-1,5);

        this->clearHLPort();
        // invoke sched port to cause serial read request
        printf("Call 1\n");
        this->sendBuffer();

        // shouldn't have been called yet
        ASSERT_EQ((U32)0,this->m_fromSerPortHistorySize);

        this->resetBuffer();
        this->addBuffer(5,3);

        this->clearHLPort();
        // invoke sched port to cause serial read request
        printf("Call 2\n");
        this->sendBuffer();

        // shouldn't have been called yet
        ASSERT_EQ((U32)0,this->m_fromSerPortHistorySize);


        this->resetBuffer();
        this->addBuffer(8,-1);

        this->clearHLPort();
        // invoke sched port to cause serial read request
        printf("Call 3\n");
        this->sendBuffer();
        this->verifyPortCall(0,10,0);

        // should be no decode errors
        this->verifyNoErrors(2,3,1);

    }

    void Tester::nominalTwoFramesOneBuffer(void) {
        U8 data[18];

        // first buffer

    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,10);
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        // second buffer

    	// reset test buffer
    	this->m_testBuffer.resetSer();
        // create a port call
        this->startBuffer();
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(1,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();

        ASSERT_EQ((U32)2,this->m_fromSerPortHistorySize);

        this->verifyPortCall(0,10,0);
        this->verifyPortCall(1,18,1);

        // should be no decode errors
        this->verifyNoErrors(0,2,2);

    }

    void Tester::nominalSyncOnly(void) {
    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        U8 data[10];
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer(-1,1);

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();
        ASSERT_EQ((U32)0,this->m_fromSerPortHistorySize);

        this->resetBuffer();
        this->addBuffer(1,-1);
        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();
        ASSERT_EQ((U32)1,this->m_fromSerPortHistorySize);
        this->verifyPortCall(0,10,0);

        // should be no decode errors
        this->verifyNoErrors(1,2,1);

    }

    void Tester::nominalSyncPlusSize(void) {
    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        U8 data[10];
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer(-1,2);

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();
        ASSERT_EQ((U32)0,this->m_fromSerPortHistorySize);

        this->resetBuffer();
        this->addBuffer(2,-1);
        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();
        ASSERT_EQ((U32)1,this->m_fromSerPortHistorySize);
        this->verifyPortCall(0,10,0);

        // should be no decode errors
        this->verifyNoErrors(1,2,1);

    }

    void Tester::nominalSendPackets(void) {

        class TestSerialize : public Fw::SerializeBufferBase {

            public:

                U8* getBuffAddr(void) {
                    return this->m_buff;
                }

            private:

                NATIVE_UINT_TYPE getBuffCapacity(void) const {
                    return sizeof(this->m_buff);
                }

                const U8* getBuffAddr(void) const {
                    return this->m_buff;
                }

                U8 m_buff[200];

        };

        TestSerialize testBuff; // buffer to fill with test data
        HLRouterComponentImpl::SerialDriverBuffer compBuff; // buffer to compare results
        // fill in sync byte and size
        ASSERT_EQ(Fw::FW_SERIALIZE_OK,compBuff.serialize(Cfg::UART_SYNC_BYTE));
        ASSERT_EQ(Fw::FW_SERIALIZE_OK,compBuff.serialize((U8)0));


        // send a packet for each port
        const U32 TEST_PKT_SIZE = 10;
        for (NATIVE_UINT_TYPE packet = 0; packet < HLRouterComponentImpl::NUM_LLPORTSIN_INPUT_PORTS; packet++) {
            // fill with data
            U8* data = testBuff.getBuffAddr();
            for (NATIVE_UINT_TYPE byte = 0; byte < TEST_PKT_SIZE; byte++) {
                data[byte] = packet+byte;
            }
            testBuff.setBuffLen(TEST_PKT_SIZE);
            this->invoke_to_LLPortsIn(packet,testBuff);

            // fill out expected buffer
            ASSERT_EQ(Fw::FW_SERIALIZE_OK,compBuff.serialize((U8)packet));
            ASSERT_EQ(Fw::FW_SERIALIZE_OK,compBuff.serialize((U8)(TEST_PKT_SIZE)));
            ASSERT_EQ(Fw::FW_SERIALIZE_OK,compBuff.serialize(data,TEST_PKT_SIZE,true));

        }

        // put size in compare buffer
        U8* data = compBuff.getBuffAddr();
        data[1] = compBuff.getBuffLength() - 2*sizeof(U8);
        // put in CRC
        U16 crc = HLRouterComponentImpl::calculate_crc_chksum(&data[2*sizeof(BYTE)],compBuff.getBuffLength()-2*sizeof(BYTE));
        ASSERT_EQ(Fw::FW_SERIALIZE_OK,compBuff.serialize(crc));

        // reset port call flag
        this->m_serPortWritten = false;
        // send sched port call to push data
        this->invoke_to_Sched(0,HLRTR_SCHED_UART_SEND);

        // verify UART port call
        ASSERT_TRUE(this->m_serPortWritten);

        // Verify data is as expected
        ASSERT_EQ(compBuff.getBuffLength(),this->m_serBufferObject.getsize());
        // verify CRC
        ASSERT_EQ( (data[compBuff.getBuffLength()-2] << 8 | data[compBuff.getBuffLength()-1]),
                   (this->m_serBuffer[m_serBufferObject.getsize()-2] << 8 | this->m_serBuffer[m_serBufferObject.getsize()-1]) );

        //dumpBuff(rData,recvBuff.getsize());
        for (NATIVE_UINT_TYPE byte = 0; byte < compBuff.getBuffLength(); byte++) {
            //printf("BYTE: %d 0x%02X 0x%02X\n",byte,data[byte],this->m_serBuffer[byte]);
            ASSERT_EQ(data[byte],this->m_serBuffer[byte]);
        }

    }

    void Tester::offNomResyncOnePacket(void) {
        // Fill with non-sync values
    	this->resetBuffer();
    	this->addOffBytes(2);

    	// reset test buffer
    	this->m_testBuffer.resetSer();
        // create a port call
        this->startBuffer();
        U8 data[10];
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();

        ASSERT_EQ(this->m_numPacketsSent, this->component.m_numPackets);
        ASSERT_EQ(this->m_numPacketsSent, this->component.m_numGoodPackets);
        ASSERT_EQ((U32)1,this->m_fromSerPortHistorySize);
        // check number of resyncs
        ASSERT_EQ((NATIVE_UINT_TYPE)1,this->component.m_numResyncs);
        /*ASSERT_from_TlmOut_SIZE(3);
          ASSERT_from_TlmOut(1,LL_TLM_UART_RECV_RESYNC_ERRS,1U);*/
        // should be no decode errors
        ASSERT_EQ((U32)0,this->component.m_numDecodeErrors);
        ASSERT_EQ((U32)0,this->component.m_numDecodeErrors);
        ASSERT_EQ((U32)0,this->component.m_numCrcErrors);
        ASSERT_EQ((U32)0,this->component.m_numInvalidPorts);
        ASSERT_EQ((U32)0,this->component.m_numBadSerialPortCalls);
        ASSERT_EQ((U32)0,this->component.m_numOutputBufferOverflows);
        ASSERT_EQ((U32)0,this->component.m_numZeroPktSize);
        this->verifyPortCall(0,10,0);

    }

    void Tester::offNomPktResyncPkt(void) {
        U8 data[18];

        // first buffer

    	// reset test buffer
    	this->m_testBuffer.resetSer();
    	// reset output buffer
    	this->resetBuffer();
        // create a port call
        this->startBuffer();
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(0,data,10);
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        // add some bytes to the middle to cause a resync
    	this->addOffBytes(2);

        // second buffer

    	// reset test buffer
    	this->m_testBuffer.resetSer();
        // create a port call
        this->startBuffer();
        this->fillBuffer(data,sizeof(data));
        this->addPortCall(1,data,sizeof(data));
        this->finishBuffer();
        // add buffer to output buffer
        this->addBuffer();

        this->clearHLPort();
        // invoke sched port to cause serial read request
        this->sendBuffer();

        ASSERT_EQ(2U, this->component.m_numGoodPackets);
        ASSERT_EQ(this->m_numPacketsSent, this->component.m_numPackets);
        ASSERT_EQ((U32)2,this->m_fromSerPortHistorySize);

        this->verifyPortCall(0,10,0);
        this->verifyPortCall(1,18,1);

        // check number of resyncs
        ASSERT_EQ((NATIVE_UINT_TYPE)1,this->component.m_numResyncs);
        /*ASSERT_from_TlmOut_SIZE(4);
          ASSERT_from_TlmOut(2,LL_TLM_UART_RECV_RESYNC_ERRS,1U);*/
        // should be no decode errors
        ASSERT_EQ((U32)0,this->component.m_numDecodeErrors);
        ASSERT_EQ((U32)0,this->component.m_numCrcErrors);
        ASSERT_EQ((U32)0,this->component.m_numInvalidPorts);
        ASSERT_EQ((U32)0,this->component.m_numBadSerialPortCalls);
        ASSERT_EQ((U32)0,this->component.m_numOutputBufferOverflows);
        ASSERT_EQ((U32)0,this->component.m_numZeroPktSize);
    }

    void Tester::offNomDecodeError(void) {

          this->clearHistory();

          // reset test buffer
          this->m_testBuffer.resetSer();
          // reset output buffer
          this->resetBuffer();
          // create a port call
          this->startBuffer();
          U8 data[10];
          this->fillBuffer(data,sizeof(data));
          // Adding invalid port number of 25
          this->addPortCall(25,data,sizeof(data));
          this->finishBuffer();
          // add buffer to output buffer
          this->addBuffer();

          this->clearHLPort();
          // send serial receive message
          this->sendBuffer();

          ASSERT_EQ(this->m_numPacketsSent, this->component.m_numPackets);
          ASSERT_EQ(this->m_numPacketsSent, this->component.m_numGoodPackets);
          // check decode error
          ASSERT_EQ((NATIVE_UINT_TYPE)1,this->component.m_numDecodeErrors);
          /*ASSERT_from_TlmOut_SIZE(3);
            ASSERT_from_TlmOut(2,LL_TLM_NUM_DECODE_ERRS_HL_PKTS,1U);*/

          ASSERT_EQ((U32)0,this->component.m_numCrcErrors);
          ASSERT_EQ((U32)0,this->component.m_numInvalidPorts);
          ASSERT_EQ((U32)0,this->component.m_numBadSerialPortCalls);
          ASSERT_EQ((U32)0,this->component.m_numOutputBufferOverflows);
          ASSERT_EQ((U32)0,this->component.m_numZeroPktSize);

          // Verify HLPortsOut not called:
          ASSERT_EQ(0U,this->m_fromSerPortHistorySize);

      }

      void Tester::offNomSerialError(void) {

          // Currently serial errors are not detected b/c driver does not detect them
      }

      void Tester::offNomCrcError(void) {

           this->clearHistory();

           // reset test buffer
           this->m_testBuffer.resetSer();
           // reset output buffer
           this->resetBuffer();
           // create a port call
           this->startBuffer();
           U8 data[10];
           this->fillBuffer(data,sizeof(data));
           this->addPortCall(0,data,sizeof(data));

           // Add bad CRC:
           // update size
           BYTE* ptr = this->m_testBuffer.getBuffAddr();
           ptr[1] = this->m_testBuffer.getBuffLength() - 2*sizeof(BYTE); // buffer size - sync and size
           // add CRC
           //U16 exp_crc = HLRouterComponentImpl::calculate_crc_chksum(&ptr[2*sizeof(BYTE)],this->m_testBuffer.getBuffLength()-2*sizeof(BYTE));
           U16 actual_crc = 0xF00D;
           ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize(actual_crc));

           // add buffer to output buffer
           this->addBuffer();

           this->clearHLPort();
           // send serial receive message
           this->sendBuffer();

           ASSERT_EQ(this->m_numPacketsSent, this->component.m_numPackets);
           ASSERT_EQ(0U, this->component.m_numGoodPackets);
           // check crc error
           ASSERT_EQ((NATIVE_UINT_TYPE)1,this->component.m_numCrcErrors);
           /*ASSERT_from_TlmOut_SIZE(2);
             ASSERT_from_TlmOut(1,LL_TLM_UART_RECV_CRC_ERRS,1U);*/

           ASSERT_EQ((U32)0,this->component.m_numDecodeErrors);
           ASSERT_EQ((U32)0,this->component.m_numInvalidPorts);
           ASSERT_EQ((U32)0,this->component.m_numBadSerialPortCalls);
           ASSERT_EQ((U32)0,this->component.m_numOutputBufferOverflows);
           ASSERT_EQ((U32)0,this->component.m_numZeroPktSize);

           // Verify HLPortsOut not called:
           ASSERT_EQ(0U,this->m_fromSerPortHistorySize);

       }

      void Tester::offNomInvalidPort(void) {

           // No way to test this from UT b/c all of the ports are connected

       }

      void Tester::offNomBadSerialPortCall(void) {

           // No way to test this from UT b/c the output Serial port calls a InputSerial port,
           // which always returns a good status.

       }

      void Tester::offNomOutpuBufferOverflow(void) {

          this->clearHistory();

          // Send data w/ a buffer that is too large:
          U8 data[SERIAL_BUFFER_SIZE+1];
          Fw::SerialBuffer testBuffer(data,SERIAL_BUFFER_SIZE+1);
          testBuffer.fill();

          this->invoke_to_LLPortsIn(0,testBuffer);

          // Verify buffer overflow:
          ASSERT_EQ(1U, this->component.m_numOutputBufferOverflows);
          /*ASSERT_from_TlmOut_SIZE(1);
            ASSERT_from_TlmOut(0,LL_TLM_NUM_OUT_BUFFER_OVERFLOW_HL_PKTS,1U);*/

          // Verify no other errors:
          ASSERT_EQ((U32)0,this->component.m_numDecodeErrors);
          ASSERT_EQ((U32)0,this->component.m_numInvalidPorts);
          ASSERT_EQ((U32)0,this->component.m_numBadSerialPortCalls);
          ASSERT_EQ((U32)0,this->component.m_numCrcErrors);
          ASSERT_EQ((U32)0,this->component.m_numZeroPktSize);

       }

      void Tester::offNomZeroPacketSize(void) {

          this->clearHistory();

          // reset test buffer
          this->m_testBuffer.resetSer();
          // reset output buffer
          this->resetBuffer();
          // create a port call
          this->startBuffer();
          U8 data[10];
          this->fillBuffer(data,sizeof(data));
          this->addPortCall(0,data,sizeof(data));

          // Add zero size:
          BYTE* ptr = this->m_testBuffer.getBuffAddr();
          ptr[1] = 0; // buffer size
          // add CRC
          U16 crc = HLRouterComponentImpl::calculate_crc_chksum(&ptr[2*sizeof(BYTE)],this->m_testBuffer.getBuffLength()-2*sizeof(BYTE));
          ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize(crc));

          // add buffer to output buffer
          this->addBuffer();

          this->clearHLPort();
          // send serial receive message
          this->sendBuffer();

          ASSERT_EQ(this->m_numPacketsSent, this->component.m_numPackets);
          ASSERT_EQ(0U, this->component.m_numGoodPackets);
          // check zero packet size error
          ASSERT_EQ((NATIVE_UINT_TYPE)1,this->component.m_numZeroPktSize);
          /*ASSERT_from_TlmOut_SIZE(2);
            ASSERT_from_TlmOut(1,LL_TLM_NUM_ZERO_PKT_SIZE_HL_PKTS,1U);*/

          ASSERT_EQ((U32)0,this->component.m_numDecodeErrors);
          ASSERT_EQ((U32)0,this->component.m_numInvalidPorts);
          ASSERT_EQ((U32)0,this->component.m_numBadSerialPortCalls);
          ASSERT_EQ((U32)0,this->component.m_numOutputBufferOverflows);
          ASSERT_EQ((U32)0,this->component.m_numCrcErrors);

          // Verify HLPortsOut not called:
          ASSERT_EQ(0U,this->m_fromSerPortHistorySize);

       }

    void Tester::dumpBuff(NATIVE_UINT_TYPE bytes)  {
        printf ("TB:\n");
        for (NATIVE_UINT_TYPE byte = 0; byte < bytes; byte++) {
            printf("%d: [0x%02X]\n",byte,this->m_serBuffer[byte]);
        }
    }

    void Tester::startBuffer(void) {
    	ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize(Cfg::UART_SYNC_BYTE));
    	// placeholder size
    	ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize((U8)0));
    	printf("Start buffer size: %d\n",this->m_testBuffer.getBuffLength());
    }

    void Tester::addPortCall(U8 portNum, BYTE* buffer, U8 bytes) {
    	// serialize port number
    	ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize(portNum));
    	ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize(bytes));
    	ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize(buffer,bytes,true));
    	printf("Add port call buffer size: %d\n",this->m_testBuffer.getBuffLength());

    }

    void Tester::finishBuffer(void) {
    	// update size
        BYTE* ptr = this->m_testBuffer.getBuffAddr();
    	ptr[1] = this->m_testBuffer.getBuffLength() - 2*sizeof(BYTE); // buffer size - sync and size
    	printf("Setting buffer size to %d\n",ptr[1]);
    	// add CRC
    	printf("CRCing %lu bytes\n",this->m_testBuffer.getBuffLength()-2*sizeof(BYTE));
    	// CRC starting at first byte after sync and size
        U16 crc = HLRouterComponentImpl::calculate_crc_chksum(&ptr[2*sizeof(BYTE)],this->m_testBuffer.getBuffLength()-2*sizeof(BYTE));
    	ASSERT_EQ(Fw::FW_SERIALIZE_OK,this->m_testBuffer.serialize(crc));
    	printf("finishBuffer buffer size: %d\n",this->m_testBuffer.getBuffLength());
    }

    void Tester::fillBuffer(U8* buff, NATIVE_UINT_TYPE bytes) {
    	for (NATIVE_UINT_TYPE byte = 0; byte < bytes; byte++) {
    		buff[byte] = byte;
    	}
    }

    void Tester::addBuffer(NATIVE_INT_TYPE start, NATIVE_INT_TYPE size) {
    	// get current offset
    	NATIVE_UINT_TYPE offset = this->m_serBufferObject.getsize();
    	printf("addBuffer offset: %d\n",offset);
    	BYTE* startPtr;
    	NATIVE_INT_TYPE copySize;
    	NATIVE_UINT_TYPE buffSize = this->m_testBuffer.getBuffLength();
    	if (-1 == start) {
    		startPtr = this->m_testBuffer.getBuffAddr();
    		start = 0;
    	} else {
    		ASSERT_TRUE((NATIVE_UINT_TYPE)start < buffSize);
    		startPtr = &this->m_testBuffer.getBuffAddr()[start];
    	}

    	if (-1 == size) {
    		copySize = buffSize - start;
    	} else {
    		copySize = size;
    		ASSERT_TRUE(start + copySize <= (NATIVE_INT_TYPE)buffSize);
    	}

    	// copy into current output buffer location
    	memcpy(&this->m_serBuffer[offset],startPtr,copySize);
    	// update size
        this->m_serBufferObject.setsize(copySize + offset);
    }

    void Tester::addOffBytes(NATIVE_UINT_TYPE bytes) {
    	NATIVE_UINT_TYPE offset = this->m_serBufferObject.getsize();
    	printf("addOffBytes offset: %d\n",offset);
    	BYTE* ptr = (BYTE*)this->m_serBufferObject.getdata();
    	for (NATIVE_UINT_TYPE byte = 0; byte < bytes; byte++) {
    		printf("byte+offset = %d\n",byte+offset);
    		ptr[byte+offset] = 0x20 + (BYTE)byte;
    	}
        this->m_serBufferObject.setsize(bytes + offset);

    }


    void Tester::resetBuffer(void) {
    	this->m_serBufferObject.setsize(0);
    }

    void Tester::verifyPortCall(NATIVE_INT_TYPE port, NATIVE_UINT_TYPE bytes, NATIVE_UINT_TYPE order) {
    	serPortHistory e = this->m_serPortHistory->at(order);

    	ASSERT_EQ(port,e.port);
    	// should be
    	ASSERT_EQ(bytes,e.buff.getBuffLength());
    	BYTE* ptr = e.buff.getBuffAddr();
    	for (NATIVE_UINT_TYPE byte = 0; byte < bytes; byte++) {
    		ASSERT_EQ(byte,ptr[byte]);
    	}
    }

    void Tester::verifyNoErrors(U32 idx_num_pkts, U32 idx_num_good_pkts, U32 num_good_pkts)
    {
        ASSERT_EQ(this->m_numPacketsSent, this->component.m_numPackets);
        ASSERT_EQ(num_good_pkts, this->component.m_numGoodPackets);
        /*ASSERT_from_TlmOut_SIZE(idx_num_good_pkts+1);
          ASSERT_from_TlmOut(idx_num_pkts,LL_TLM_NUM_HL_PKTS,this->m_numPacketsSent);
          ASSERT_from_TlmOut(idx_num_good_pkts,LL_TLM_NUM_GOOD_HL_PKTS,num_good_pkts);*/

        ASSERT_EQ((U32)0,this->component.m_numResyncs);
        ASSERT_EQ((U32)0,this->component.m_numDecodeErrors);
        ASSERT_EQ((U32)0,this->component.m_numCrcErrors);
        ASSERT_EQ((U32)0,this->component.m_numInvalidPorts);
        ASSERT_EQ((U32)0,this->component.m_numBadSerialPortCalls);
        ASSERT_EQ((U32)0,this->component.m_numOutputBufferOverflows);
        ASSERT_EQ((U32)0,this->component.m_numZeroPktSize);
    }

    // ----------------------------------------------------------------------
    // Handlers for typed from ports
    // ----------------------------------------------------------------------

    void Tester::from_SerReadPort_handler(const NATIVE_INT_TYPE portNum,
            Fw::Buffer &serBuffer, Drv::SerialReadStatus &status) {
        // Return staged data
        ASSERT_TRUE(serBuffer.getsize() > this->m_serBufferObject.getsize());
        memcpy((void*)serBuffer.getdata(),(void*)this->m_serBufferObject.getdata(),this->m_serBufferObject.getsize());
        serBuffer.setsize(this->m_serBufferObject.getsize());
    }

    void Tester::from_SerWritePort_handler(const NATIVE_INT_TYPE portNum,
            Fw::Buffer &serBuffer) {
        // Copy data to test buffers
        this->m_serBufferObject = serBuffer;
        memcpy(this->m_serBuffer,(void*)serBuffer.getdata(),serBuffer.getsize());
        this->m_serPortWritten = true;
    }

  /*    void Tester ::
      from_TlmOut_handler(
          const NATIVE_INT_TYPE portNum,
          U32 id,
          U32 val
      )
    {
      this->pushFromPortEntry_TlmOut(id, val);
      }*/


    void Tester ::
      from_HLPortsOut_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      )
    {
    	printf("HL Handler! %d\n",Buffer.getBuffLength());

    	serPortHistory _e = {
          portNum, Buffer
        };

    	this->m_serPortHistory->push_back(_e);
        ++this->m_fromSerPortHistorySize;
    }

    void Tester ::
      from_DebugOut_handler(
          const NATIVE_INT_TYPE portNum,
          U8 *string,
          U32 size
      )
    {
      this->pushFromPortEntry_DebugOut(string, size);
    }

    void Tester::clearHLPort(void) {
    	this->m_serPortHistory->clear();
    	this->m_fromSerPortHistorySize = 0;
    }

    void Tester::sendBuffer(void) {

        // invoke sched port to cause serial read request
        this->invoke_to_Sched(0,HLRTR_SCHED_UART_RECEIVE);
        ++this->m_numPacketsSent;
    }

    // ----------------------------------------------------------------------
    // Helper methods
    // ----------------------------------------------------------------------

    void Tester ::
      connectPorts(void)
    {

      // Sched
      this->connect_to_Sched(
          0,
          this->component.get_Sched_InputPort(0)
      );

      // SerReadPort
      this->component.set_SerReadPort_OutputPort(
          0,
          this->get_from_SerReadPort(0)
      );

      // SerWritePort
      this->component.set_SerWritePort_OutputPort(
          0,
          this->get_from_SerWritePort(0)
      );

      // TlmOut
      /*      this->component.set_TlmOut_OutputPort(
          0,
          this->get_from_TlmOut(0)
          );*/

      // DebugOut
      this->component.set_DebugOut_OutputPort(
          0,
          this->get_from_DebugOut(0)
      );


    // ----------------------------------------------------------------------
    // Connect serial output ports
    // ----------------------------------------------------------------------
      for (NATIVE_INT_TYPE i = 0; i < 25; ++i) {
        this->component.set_HLPortsOut_OutputPort(
            i,
            this->get_from_HLPortsOut(i)
        );
      }


    // ----------------------------------------------------------------------
    // Connect serial input ports
    // ----------------------------------------------------------------------
      // LLPortsIn
      for (NATIVE_INT_TYPE i = 0; i < 25; ++i) {
        this->connect_to_LLPortsIn(
            i,
            this->component.get_LLPortsIn_InputPort(i)
        );
      }


    }

    void Tester::initComponents(void) {
        this->init();
        this->component.init(
        INSTANCE);
    }

} // end namespace LLProc
