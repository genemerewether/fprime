// ====================================================================== 
// \title  STIM300.hpp
// \author kubiak
// \brief  cpp file for STIM300 test harness implementation class
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

#include <thread>
#include <functional>
#include <chrono>

#include "Tester.hpp"
#include "STIM300Model.hpp"

#define INSTANCE 0
#define MAX_HISTORY_SIZE 100

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      STIM300GTestBase("Tester", MAX_HISTORY_SIZE),
      component("STIM300"),
#else
      STIM300GTestBase(MAX_HISTORY_SIZE),
      component(),
#endif
      m_modelMutex(),
      m_eventRB(),
      m_expPkts(),
      m_uartBufferData(),
      m_uartExtBuffer(&this->m_uartBufferData[0], sizeof(this->m_uartBufferData)),
      m_uartFwBuffer(0, 0, reinterpret_cast<U64>(&this->m_uartBufferData[0]), sizeof(this->m_uartBufferData)),
      m_receivedPkts(0),
      m_partialPkts(0)
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester(void) 
  {
    
  }

  // ----------------------------------------------------------------------
  // Tests 
  // ----------------------------------------------------------------------

  void Tester::eventFunc(Fw::Time time) {
    std::lock_guard<std::mutex> lock(this->m_modelMutex);
    this->m_eventRB.queue(&time);
  }

  void Tester::bytesFunc(U8* data, U32 len) {
    std::lock_guard<std::mutex> lock(this->m_modelMutex);

    this->m_uartExtBuffer.serialize(data, len, true);
  }

  void Tester::expectFunc(U8* data, U32 len, ROS::sensor_msgs::ImuNoCov imuPkt) {
    std::lock_guard<std::mutex> lock(this->m_modelMutex);

    this->m_uartExtBuffer.serialize(data, len, true);

    //std::cout << "Exp: " << imuPkt.getheader().getseq() << std::endl; 
    this->m_expPkts.queue(&imuPkt);
    this->m_receivedPkts++;
  }

  void Tester ::
    nominalTest(void) 
  {
    using namespace std::placeholders;

    STIM300Model model(200,
                       std::bind(&Tester::eventFunc, this, _1),
                       std::bind(&Tester::bytesFunc, this, _1, _2),
                       std::bind(&Tester::expectFunc, this, _1, _2, _3));
                    
    this->component.m_timeSyncState = STIM300ComponentImpl::S300_TS_SYNCED;

    Drv::SerialReadStatus readStat = Drv::SER_OK;
    ROS::sensor_msgs::ImuNoCov tempImuPkt;
    U8 stimPktBuffer[256];
    U32 stimPktLen;
    U32 totalLen;
    Fw::Time eventTime;

    // Send one packet
    this->clearHistory();

    model.generatePkts(tempImuPkt, eventTime, stimPktBuffer, stimPktLen);

    this->m_expPkts.queue(&tempImuPkt);
    this->m_eventRB.queue(&eventTime);
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[0]));
    this->m_uartFwBuffer.setsize(stimPktLen);

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    // Seed the packet counter correctly
    this->component.m_pktCounter = (tempImuPkt.getheader().getseq() - 1) % 256;

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(1);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 1);

    // Send one packet again
    this->clearHistory();

    model.generatePkts(tempImuPkt, eventTime, stimPktBuffer, stimPktLen);

    this->m_expPkts.queue(&tempImuPkt);
    this->m_eventRB.queue(&eventTime);
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[0]));
    this->m_uartFwBuffer.setsize(stimPktLen);

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(1);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 2);

    // Send two packets
    this->clearHistory();

    model.generatePkts(tempImuPkt, eventTime, stimPktBuffer, stimPktLen);

    this->m_expPkts.queue(&tempImuPkt);
    this->m_eventRB.queue(&eventTime);
    totalLen = stimPktLen;

    model.generatePkts(tempImuPkt, eventTime, &stimPktBuffer[stimPktLen], stimPktLen);

    this->m_expPkts.queue(&tempImuPkt);
    this->m_eventRB.queue(&eventTime);
    totalLen += stimPktLen;
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[0]));
    this->m_uartFwBuffer.setsize(totalLen);

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(2);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 4);

    // Send 1.5 packets
    this->clearHistory();

    model.generatePkts(tempImuPkt, eventTime, stimPktBuffer, stimPktLen);

    this->m_expPkts.queue(&tempImuPkt);
    this->m_eventRB.queue(&eventTime);
    totalLen = stimPktLen;

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    model.generatePkts(tempImuPkt, eventTime, &stimPktBuffer[stimPktLen], stimPktLen);

    this->m_expPkts.queue(&tempImuPkt);
    this->m_eventRB.queue(&eventTime);
    totalLen += stimPktLen;
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[0]));
    this->m_uartFwBuffer.setsize(totalLen - 4);

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(1);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 5);

    // Send 0.5 packets
    this->clearHistory();
    memmove(&stimPktBuffer[0], &stimPktBuffer[totalLen-4], 4);
    this->m_uartFwBuffer.setsize(4);

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(1);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 6);

    // Send 0.5 packets
    this->clearHistory();
    model.generatePkts(tempImuPkt, eventTime, stimPktBuffer, stimPktLen);

    this->m_expPkts.queue(&tempImuPkt);
    this->m_eventRB.queue(&eventTime);
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[0]));
    this->m_uartFwBuffer.setsize(stimPktLen-4);

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(0);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 6);

    // Send 0.5 packets
    this->clearHistory();
    memmove(&stimPktBuffer[0], &stimPktBuffer[stimPktLen-4], 4);
    this->m_uartFwBuffer.setsize(4);

    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(1);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 7);

    // Send 0 packets
    this->clearHistory();

    this->m_uartFwBuffer.setsize(0);

    this->invoke_to_sched(0, 0);

    ASSERT_from_IMU_SIZE(0);

    ASSERT_TLM_NumPackets_SIZE(1);
    ASSERT_TLM_NumPackets(0, 7);

  }

  void Tester ::
    manyPackets() 
  {
    using namespace std::placeholders;

    const int iterations = 10000;
    //const int uart_per_sched = 4;
    const int uart_period_us = 2000;

    bool firstPass = true;

    STIM300Model model(500,
                       std::bind(&Tester::eventFunc, this, _1),
                       std::bind(&Tester::bytesFunc, this, _1, _2),
                       std::bind(&Tester::expectFunc, this, _1, _2, _3));

    this->component.m_timeSyncState = STIM300ComponentImpl::S300_TS_SYNCED;

    std::cout << "Processing 10000 packets" << std::endl;

    std::thread modelThread(&Drv::STIM300Model::run, &model);

    for (int i = 1; i <= iterations; i++) {

        if (i % (iterations / 10) == 0) {
            std::cout << i / (iterations/100) << "%" << std::endl;
        }

        this->clearHistory();

        {
            std::lock_guard<std::mutex> lock(this->m_modelMutex);

            // Seed the packet counter
            if (firstPass) {
                if (this->m_expPkts.size() > 0) {
                    this->component.m_pktCounter = (this->m_expPkts.get(0)->getheader().getseq() - 1) % 256;
                    firstPass = false;
                }
            }

            this->m_uartFwBuffer.setsize(this->m_uartExtBuffer.getBuffLength());

            Drv::SerialReadStatus readStat = Drv::SER_OK;
            //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

            int numExpPkts = this->m_expPkts.size();
            this->invoke_to_sched(0, 0);

            ASSERT_from_IMU_SIZE(numExpPkts);

            ASSERT_EQ(0, this->m_expPkts.size());

            if (this->component.m_pktBufferIdx != 0) {
                this->m_partialPkts++;
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(uart_period_us));
    }

    model.stop();

    modelThread.join();

    std::cout << "Processed " << this->m_receivedPkts << " Packets" << std::endl;
    std::cout << "Precessed " << this->m_partialPkts << " Partial Packets" << std::endl;
  }

  void Tester ::
    timeSyncTest()
  {
    using namespace std::placeholders;

    const int iterations = 50000;
    const int uart_per_sched = 4;
    const int uart_period_us = 2000;
    const int gather_iters = 10;
    const int synced_iters = 20;

    bool gatherData = true;
    int gatherCount = 0;

    int checkCount = 0;
    int syncedCount = 0;

    int tsFails = 0;

    STIM300ComponentImpl::STIM300TSState expTsState;


    STIM300Model model(500,
                       std::bind(&Tester::eventFunc, this, _1),
                       std::bind(&Tester::bytesFunc, this, _1, _2),
                       std::bind(&Tester::expectFunc, this, _1, _2, _3));

    std::thread modelThread(&Drv::STIM300Model::run, &model);

    for (int i = 1; i <= iterations; i++) {

        this->clearHistory();

        // Latch in all received events to mimic deployment behavior
        {
            std::lock_guard<std::mutex> lock(this->m_modelMutex);

            while (this->m_eventRB.size() > 0) {
                Fw::Time temp;
                this->m_eventRB.dequeue(&temp);
                this->m_eventLatchedRB.queue(&temp);
            }

        }

        if (gatherData) {

            {
                std::lock_guard<std::mutex> lock(this->m_modelMutex);

                this->m_uartFwBuffer.setsize(this->m_uartExtBuffer.getBuffLength());

                Drv::SerialReadStatus readStat = Drv::SER_OK;
                //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

                this->m_uartExtBuffer.resetSer();
            }

            gatherCount++;
            if (gatherCount == gather_iters) {
                gatherData = false;
                syncedCount = 0;
                expTsState = STIM300ComponentImpl::S300_TS_CLEAR;
            }
        } else {
            int numExpPkts;
            int numExpEvents;
            {
                std::lock_guard<std::mutex> lock(this->m_modelMutex);

                // Push uart data to component
                this->m_uartFwBuffer.setsize(this->m_uartExtBuffer.getBuffLength());
                Drv::SerialReadStatus readStat = Drv::SER_OK;
                //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);
                this->m_uartExtBuffer.resetSer();

                if (i % uart_per_sched == 0) {

                    int expEventCalls;

                    ASSERT_EQ(expTsState, this->component.m_timeSyncState);

                    switch (expTsState) {
                        case STIM300ComponentImpl::S300_TS_CLEAR:

                            expEventCalls = this->m_eventLatchedRB.size() + 1;
                            this->invoke_to_sched(0, 0);
                            ASSERT_from_IMU_SIZE(0);

                            ASSERT_from_packetTime_SIZE(expEventCalls);
                            this->m_expPkts.reset();

                            ASSERT_EQ(STIM300ComponentImpl::S300_TS_WAIT, this->component.m_timeSyncState);

                            ASSERT_TLM_TimeSyncStatus_SIZE(1);
                            ASSERT_TLM_TimeSyncStatus(0, STIM300ComponentImpl::STIM300_TS_WAIT);

                            expTsState = STIM300ComponentImpl::S300_TS_WAIT;

                            break;
                        case STIM300ComponentImpl::S300_TS_WAIT:

                            // Expect that if at least one packet exists it will be used for timesync

                            if (this->m_expPkts.size() > 0) {

                                // We don't pass the first packet after a time sync
                                this->m_expPkts.remove();

                                numExpPkts = 0;
                                numExpEvents = this->m_eventLatchedRB.size();

                                this->invoke_to_sched(0, 0);

                                ASSERT_EQ(STIM300ComponentImpl::S300_TS_CHECK, this->component.m_timeSyncState);

                                ASSERT_from_IMU_SIZE(numExpPkts);

                                ASSERT_TLM_TimeSyncStatus_SIZE(1);
                                ASSERT_TLM_TimeSyncStatus(0, STIM300ComponentImpl::STIM300_TS_CHECK);

                                ASSERT_from_packetTime_SIZE(numExpEvents + 2); // Two calls to gatherEvents()

                                expTsState = STIM300ComponentImpl::S300_TS_CHECK;

                                checkCount = 0;
                            } else {

                                this->invoke_to_sched(0, 0);

                                ASSERT_EQ(STIM300ComponentImpl::S300_TS_WAIT, this->component.m_timeSyncState);

                                ASSERT_from_IMU_SIZE(0);

                                ASSERT_from_packetTime_SIZE(0);

                                ASSERT_TLM_TimeSyncStatus_SIZE(1);
                                ASSERT_TLM_TimeSyncStatus(0, STIM300ComponentImpl::STIM300_TS_WAIT);
                            }

                            break;
                        case STIM300ComponentImpl::S300_TS_CHECK:

                            this->m_expPkts.reset();

                            this->invoke_to_sched(0, 0);

                            ASSERT_from_IMU_SIZE(0);

                            if (this->component.m_timeSyncState == STIM300ComponentImpl::S300_TS_CHECK) {

                                ASSERT_LT(checkCount, STIM_TS_CHECK_CYCLES);

                                ASSERT_TLM_TimeSyncStatus_SIZE(1);
                                ASSERT_TLM_TimeSyncStatus(0, STIM300ComponentImpl::STIM300_TS_CHECK);

                                ASSERT_EQ(STIM300ComponentImpl::S300_TS_CHECK,
                                        this->component.m_timeSyncState);
                            } else if (this->component.m_timeSyncState == STIM300ComponentImpl::S300_TS_CLEAR) {
                                // There was an issue with time sync

                                ASSERT_TLM_TimeSyncStatus_SIZE(1);
                                ASSERT_TLM_TimeSyncStatus(0, STIM300ComponentImpl::STIM300_TS_CLEAR);

                                ASSERT_EQ(STIM300ComponentImpl::S300_TS_CLEAR,
                                        this->component.m_timeSyncState);

                                expTsState = STIM300ComponentImpl::S300_TS_CLEAR;
                                tsFails++;
                            } else {

                                //ASSERT_EQ(STIM_TS_CHECK_CYCLES, checkCount);

                                ASSERT_TLM_TimeSyncStatus_SIZE(1);
                                ASSERT_TLM_TimeSyncStatus(0, STIM300ComponentImpl::STIM300_TS_SYNCED);

                                ASSERT_EQ(STIM300ComponentImpl::S300_TS_SYNCED,
                                        this->component.m_timeSyncState);

                                expTsState = STIM300ComponentImpl::S300_TS_SYNCED;
                            }

                            checkCount++;
                            break;

                        case STIM300ComponentImpl::S300_TS_SYNCED:

                            if (syncedCount == synced_iters) {

                                // Manually poke the component to reset sync
                                this->component.m_timeSyncState = STIM300ComponentImpl::S300_TS_CLEAR;

                                expTsState = STIM300ComponentImpl::S300_TS_CLEAR;

                                gatherData = true;
                                gatherCount = 0;
                            } else {

                                numExpPkts = this->m_expPkts.size();
                                this->invoke_to_sched(0, 0);

                                ASSERT_from_IMU_SIZE(numExpPkts);

                                ASSERT_TLM_TimeSyncStatus_SIZE(1);
                                ASSERT_TLM_TimeSyncStatus(0, STIM300ComponentImpl::STIM300_TS_SYNCED);

                                ASSERT_EQ(0, this->m_expPkts.size());

                                syncedCount++;
                            }
                            break;
                        default:
                            FAIL();
                            break;
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(uart_period_us));
    }

    model.stop();

    modelThread.join();
  }

  void Tester ::
    manualTimeSyncTest()
  {
    using namespace std::placeholders;

    STIM300Model model(200,
                       std::bind(&Tester::eventFunc, this, _1),
                       std::bind(&Tester::bytesFunc, this, _1, _2),
                       std::bind(&Tester::expectFunc, this, _1, _2, _3));

    ROS::sensor_msgs::ImuNoCov imuPkt;
    Fw::Time eventTime;
    U8 stimPktBuffer[256];
    U32 stimPktLen;
    Drv::SerialReadStatus readStat = Drv::SER_OK;

    /********************
    * Sync case 1: Full *
    ********************/

    this->invoke_to_sched(0, 0);
    ASSERT_EQ(STIM300ComponentImpl::S300_TS_WAIT,
            this->component.m_timeSyncState);

    this->sendPktsFromModel(model, 4);

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    ASSERT_EQ(STIM300ComponentImpl::S300_TS_CHECK,
            this->component.m_timeSyncState);

    for (int i = 0; i < Drv::STIM_TS_CHECK_CYCLES - 1; i++) {
        this->clearHistory();
        this->sendPktsFromModel(model, 4);
        this->invoke_to_sched(0, 0);

        ASSERT_from_IMU_SIZE(0);
    }

    this->m_expPkts.reset();

    ASSERT_EQ(STIM300ComponentImpl::S300_TS_SYNCED,
            this->component.m_timeSyncState);

    for (int i = 0; i < 10; i++) {
        this->clearHistory();
        this->sendPktsFromModel(model, 4);
        this->invoke_to_sched(0, 0);

        ASSERT_from_IMU_SIZE(4);
    }
    
    /***********************
    * Sync case 2: Partial *
    ***********************/
    this->component.m_timeSyncState = STIM300ComponentImpl::S300_TS_CLEAR;

    this->sendPktsFromModel(model, 4);

    // Send 5 bytes
    model.generatePkts(imuPkt, eventTime, &stimPktBuffer[0], stimPktLen);
    this->m_eventLatchedRB.queue(&eventTime);
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[0]));
    this->m_uartFwBuffer.setsize(5);
    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    ASSERT_EQ(STIM300ComponentImpl::S300_TS_WAIT,
            this->component.m_timeSyncState);

    // Send the rest of the packet
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[5]));
    this->m_uartFwBuffer.setsize(stimPktLen - 5);
    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->sendPktsFromModel(model, 4);

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    ASSERT_EQ(STIM300ComponentImpl::S300_TS_CHECK,
            this->component.m_timeSyncState);


    for (int i = 0; i < Drv::STIM_TS_CHECK_CYCLES - 1; i++) {
        this->clearHistory();
        this->sendPktsFromModel(model, 4);
        this->invoke_to_sched(0, 0);

        ASSERT_from_IMU_SIZE(0);
    }

    this->m_expPkts.reset();

    ASSERT_EQ(STIM300ComponentImpl::S300_TS_SYNCED,
            this->component.m_timeSyncState);

    for (int i = 0; i < 10; i++) {
        this->clearHistory();
        this->sendPktsFromModel(model, 4);
        this->invoke_to_sched(0, 0);

        ASSERT_from_IMU_SIZE(4);
    }

    /*********************
    * Sync case 3: Split *
    *********************/
    // Expected to fail

    this->component.m_timeSyncState = STIM300ComponentImpl::S300_TS_CLEAR;

    this->clearHistory();

    this->sendPktsFromModel(model, 4);

    // Generate pkt and only send event
    model.generatePkts(imuPkt, eventTime, &stimPktBuffer[0], stimPktLen);
    this->m_eventLatchedRB.queue(&eventTime);

    this->invoke_to_sched(0, 0);
    ASSERT_EQ(STIM300ComponentImpl::S300_TS_WAIT,
            this->component.m_timeSyncState);

    // Send the uart data
    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&stimPktBuffer[0]));
    this->m_uartFwBuffer.setsize(stimPktLen);
    //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);

    this->sendPktsFromModel(model, 4);

    // Time sync event should be leading uart data by 1 packet
    // Should fail check
    this->invoke_to_sched(0, 0);
    ASSERT_EQ(STIM300ComponentImpl::S300_TS_CLEAR,
            this->component.m_timeSyncState);
  }

  void Tester ::
    compareImuPkts(
        ROS::sensor_msgs::ImuNoCov& act,
        ROS::sensor_msgs::ImuNoCov& exp
    )
  {
    ASSERT_EQ(exp.getheader().getseq(), act.getheader().getseq());

    if (this->component.m_timeSyncState == STIM300ComponentImpl::S300_TS_SYNCED) {
        EXPECT_EQ(exp.getheader().getstamp(), act.getheader().getstamp());
    }

    EXPECT_NEAR(exp.getangular_velocity().getx(), act.getangular_velocity().getx(), 0.0001);
    EXPECT_NEAR(exp.getangular_velocity().gety(), act.getangular_velocity().gety(), 0.0001);
    EXPECT_NEAR(exp.getangular_velocity().getz(), act.getangular_velocity().getz(), 0.0001);

    EXPECT_NEAR(exp.getlinear_acceleration().getx(), act.getlinear_acceleration().getx(), 0.0001);
    EXPECT_NEAR(exp.getlinear_acceleration().gety(), act.getlinear_acceleration().gety(), 0.0001);
    EXPECT_NEAR(exp.getlinear_acceleration().getz(), act.getlinear_acceleration().getz(), 0.0001);
  }

  void Tester ::
    sendPktsFromModel(STIM300Model& model, const int numPkts)
  {
    ROS::sensor_msgs::ImuNoCov imuPkt;
    Fw::Time eventTime;
    U32 stimPktLen;
    Drv::SerialReadStatus readStat = Drv::SER_OK;

    for (int i = 0; i < numPkts; i++) {
        model.generatePkts(imuPkt, eventTime, &this->m_uartBufferData[this->m_uartFwBuffer.getsize()], stimPktLen);
        this->m_expPkts.queue(&imuPkt);
        this->m_eventRB.queue(&eventTime);
        this->m_uartFwBuffer.setsize(this->m_uartFwBuffer.getsize() + stimPktLen);
        //this->invoke_to_serialRead(0, this->m_uartFwBuffer, readStat);
    }

    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&this->m_uartBufferData[0]));
  }


  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_IMU_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::sensor_msgs::ImuNoCov &ImuNoCov
    )
  {
    this->pushFromPortEntry_IMU(ImuNoCov);

    ROS::sensor_msgs::ImuNoCov expImuPkt;

    ASSERT_GT(this->m_expPkts.size(), 0);

    this->m_expPkts.dequeue(&expImuPkt);

    compareImuPkts(ImuNoCov, expImuPkt);
  }

  void Tester ::
    from_packetTime_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Time &time
    )
  {
    this->pushFromPortEntry_packetTime(time);

    if (this->m_eventRB.size() > 0) {
        this->m_eventRB.dequeue(&time);
    } else {
        time = Fw::Time();
    }
  }

  void Tester ::
    from_serialRead_handler(
        const NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::Buffer &serBuffer, /*!< Buffer containing data*/
        SerialReadStatus &status /*!< Status of read*/
    )
  {
      ASSERT_GT(serBuffer.getsize(), this->m_uartFwBuffer.getsize());

      memcpy(reinterpret_cast<U8*>(serBuffer.getdata()),
             reinterpret_cast<U8*>(this->m_uartFwBuffer.getdata()),
             this->m_uartFwBuffer.getsize());

      serBuffer.setsize(this->m_uartFwBuffer.getsize());

      this->m_uartFwBuffer.setsize(0);
      this->m_uartExtBuffer.resetSer();

      status = Drv::SER_OK;
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // sched
    this->connect_to_sched(
        0,
        this->component.get_sched_InputPort(0)
    );

    // serialRead
    this->component.set_serialRead_OutputPort(
        0,
        this->get_from_serialRead(0)
    );

    // IMU
    this->component.set_IMU_OutputPort(
        0, 
        this->get_from_IMU(0)
    );

    // packetTime
    this->component.set_packetTime_OutputPort(
        0, 
        this->get_from_packetTime(0)
    );

    // Tlm
    this->component.set_Tlm_OutputPort(
        0, 
        this->get_from_Tlm(0)
    );

    // Time
    this->component.set_Time_OutputPort(
        0, 
        this->get_from_Time(0)
    );

    // Log
    this->component.set_Log_OutputPort(
        0, 
        this->get_from_Log(0)
    );

    // LogText
    this->component.set_LogText_OutputPort(
        0, 
        this->get_from_LogText(0)
    );




  }

  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        INSTANCE
    );
  }

} // end namespace Drv
