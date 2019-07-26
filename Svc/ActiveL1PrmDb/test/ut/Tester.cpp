// ====================================================================== 
// \title  ActiveL1PrmDb.hpp
// \author kubiak
// \brief  cpp file for ActiveL1PrmDb test harness implementation class
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

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10
#define QUEUE_DEPTH 10

#define MAX_L1_RECV_SIZE 128
#define MAX_L2_RECV_SIZE 64

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      ActiveL1PrmDbGTestBase("Tester", MAX_HISTORY_SIZE),
      component("ActiveL1PrmDb", "", MAX_L1_RECV_SIZE),
      activeL2Comp("ActiveL2PrmDb", MAX_L2_RECV_SIZE)
#else
      ActiveL1PrmDbGTestBase(MAX_HISTORY_SIZE),
      component(),
      activeL2Comp()
#endif
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester(void) 
  {
    
  }

  struct prmPair {

      prmPair(FwPrmIdType id, NATIVE_INT_TYPE prmInt) {
          this->id = id;
          this->prm.serialize(prmInt);
      }

      prmPair(FwPrmIdType id, const char* prmStr) {
          this->id = id;
          this->prm.serialize(reinterpret_cast<const U8*>(prmStr), strlen(prmStr));
      }

      FwPrmIdType id;
      Fw::ParamBuffer prm;
  }; 

  // ----------------------------------------------------------------------
  // Tests 
  // ----------------------------------------------------------------------

  void Tester ::
    integratedTest(void) 
  {
    // Read file on L1 and send to L2

    Fw::ParamBuffer prmBuff;
    Fw::SerializeStatus serStat;
    Fw::ParamValid valid;
    Fw::ExternalSerializeBuffer expBuffer;

    struct prmPair prms[] = {
        {1, 10},
        {2, 50},
        {3, "MyString"},
        {5, "Skipped 4"},
        {6, -1000},
        {11, "After"}
    };

    U8 expectedSendList[] = {
//      PrmId    Length BufferData
        0,0,0,5, 0,11,  0,9,'S','k','i','p','p','e','d',' ','4',
        0,0,0,6, 0,4,   0xFF,0xFF,0xFC,0x18
    };

    U8 expectedSendList2[] = {
//      PrmId    Length BufferData
        0,0,0,5, 0,11,  0,9,'S','k','i','p','p','e','d',' ','4',
        0,0,0,6, 0,4,   0,0,0x13,0x88,
        0,0,0,7, 0,4,   0,0,1,0xFF,
        0,0,0,8, 0,7,   0,5,'L','2','P','r','m'
    };

    struct Svc::ActiveL1PrmDbComponentImpl::PrmDbRange activeL2Ranges[] = {
        {0, 5, 10}
    };

    this->component.setPrmDbRanges(activeL2Ranges, FW_NUM_ARRAY_ELEMENTS(activeL2Ranges));

    this->component.m_fileName = "ReadFileIntegrationTest.prm";

    this->clearHistory();
    this->component.readPrmFile();

    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_PrmFileLoadComplete_SIZE(1);
    ASSERT_EVENTS_PrmFileLoadComplete(0,6);

    expBuffer.setExtBuffer(expectedSendList, sizeof(expectedSendList));
    expBuffer.setBuffLen(sizeof(expectedSendList));

    this->clearHistory();
    this->activeL2Comp.sched_handlerBase(0, 0);
    this->activeL2Comp.doDispatch();
    this->component.doDispatch();

    ASSERT_from_sendPrm_SIZE(1);
    ASSERT_from_sendPrm(0, false, expBuffer);

    ASSERT_from_recvPrmReady_SIZE(1);
    ASSERT_from_recvPrmReady(0, MAX_L1_RECV_SIZE, false);

    // Two calls for recvPrm and sendPrmReady
    this->activeL2Comp.doDispatch();
    this->activeL2Comp.doDispatch();

    // Call for sendPrmReady
    this->component.doDispatch();

    struct Svc::PrmDbImpl::t_dbStruct expL2PrmDbEntries[4];
    expL2PrmDbEntries[0].used = true;
    expL2PrmDbEntries[0].id = 5;
    expL2PrmDbEntries[0].val.serialize(reinterpret_cast<const U8*>("Skipped 4"), strlen("Skipped 4"));

    expL2PrmDbEntries[1].used = true;
    expL2PrmDbEntries[1].id = 6;
    expL2PrmDbEntries[1].val.serialize(static_cast<I32>(-1000));

    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[0].used, expL2PrmDbEntries[0].used);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[0].id, expL2PrmDbEntries[0].id);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[0].val, expL2PrmDbEntries[0].val);

    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[1].used, expL2PrmDbEntries[1].used);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[1].id, expL2PrmDbEntries[1].id);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[1].val, expL2PrmDbEntries[1].val);

    ASSERT_FALSE(this->activeL2Comp.m_prmDb.m_db[2].used);

    // Set parameters on L2 to send back to L1

    prmBuff.resetSer();
    prmBuff.serialize(static_cast<I32>(5000));

    // Change -1000 to 5000
    this->activeL2Comp.L2_PRM_PUSH_UPDATES_cmdHandler(0, 0, Svc::ActiveL2PrmDbComponentImpl::PUSH_UPDATES);

    this->clearHistory();
    this->activeL2Comp.setPrm_handlerBase(0, 6, prmBuff);
    this->activeL2Comp.doDispatch();

    this->activeL2Comp.sched_handlerBase(0, 0);
    this->activeL2Comp.doDispatch();
    
    this->component.doDispatch();

    ASSERT_from_recvPrmReady_SIZE(1);
    ASSERT_from_recvPrmReady(0, MAX_L1_RECV_SIZE, false);

    this->activeL2Comp.doDispatch();

    prmBuff.resetSer();

    valid = this->invoke_to_getPrm(0, 6, prmBuff);
    ASSERT_TRUE(valid);

    I32 prmIntVal;

    prmBuff.deserialize(prmIntVal);

    ASSERT_EQ(prmIntVal, 5000);

    // Add two new parameters
    prmBuff.resetSer();
    prmBuff.serialize(static_cast<I32>(511));

    this->clearHistory();
    this->activeL2Comp.setPrm_handlerBase(0, 7, prmBuff);
    this->activeL2Comp.doDispatch();

    prmBuff.resetSer();
    prmBuff.serialize(reinterpret_cast<const U8*>("L2Prm"), strlen("L2Prm"));

    this->activeL2Comp.setPrm_handlerBase(0, 8, prmBuff);
    this->activeL2Comp.doDispatch();

    this->activeL2Comp.sched_handlerBase(0, 0);
    this->activeL2Comp.doDispatch();

    this->component.doDispatch();

    ASSERT_from_recvPrmReady_SIZE(1);
    ASSERT_from_recvPrmReady(0, MAX_L1_RECV_SIZE, false);

    this->activeL2Comp.doDispatch();

    prmBuff.resetSer();

    valid = this->invoke_to_getPrm(0, 7, prmBuff);
    ASSERT_TRUE(valid);

    prmBuff.deserialize(prmIntVal);
    ASSERT_EQ(prmIntVal, 511);

    prmBuff.resetSer();

    valid = this->invoke_to_getPrm(0, 8, prmBuff);
    ASSERT_TRUE(valid);

    U8 prmStrVal[64];
    NATIVE_UINT_TYPE prmStrLen = 64;

    prmBuff.deserialize(prmStrVal, prmStrLen);
    ASSERT_EQ(prmStrLen, strlen("L2Prm"));
    ASSERT_EQ(0, memcmp("L2Prm", prmStrVal, prmStrLen));

    // Reset the L2 PrmDb
    this->activeL2Comp.m_firstSched = true;
    this->activeL2Comp.m_updateMethod = Svc::ActiveL2PrmDbComponentImpl::ACTIVE_L2_NO_UPDATES;
    this->activeL2Comp.m_prmDb.clearDb();

    // Trigger a reload
    this->clearHistory();
    this->activeL2Comp.sched_handlerBase(0, 0);
    this->activeL2Comp.doDispatch();
    
    expBuffer.setExtBuffer(expectedSendList2, sizeof(expectedSendList2));
    expBuffer.setBuffLen(sizeof(expectedSendList2));

    this->component.doDispatch();

    ASSERT_from_sendPrm_SIZE(1);
    ASSERT_from_sendPrm(0, false, expBuffer);

    ASSERT_from_recvPrmReady_SIZE(1);
    ASSERT_from_recvPrmReady(0, MAX_L1_RECV_SIZE, false);

    this->activeL2Comp.doDispatch();
    this->activeL2Comp.doDispatch();

    // Call for sendPrmReady
    this->component.doDispatch();

    expL2PrmDbEntries[1].val.resetSer();
    expL2PrmDbEntries[1].val.serialize(static_cast<I32>(5000));

    expL2PrmDbEntries[2].used = true;
    expL2PrmDbEntries[2].id = 7;
    expL2PrmDbEntries[2].val.serialize(static_cast<I32>(511));

    expL2PrmDbEntries[3].used = true;
    expL2PrmDbEntries[3].id = 8;
    expL2PrmDbEntries[3].val.serialize(reinterpret_cast<const U8*>("L2Prm"), strlen("L2Prm"));

    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[0].used, expL2PrmDbEntries[0].used);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[0].id, expL2PrmDbEntries[0].id);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[0].val, expL2PrmDbEntries[0].val);

    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[1].used, expL2PrmDbEntries[1].used);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[1].id, expL2PrmDbEntries[1].id);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[1].val, expL2PrmDbEntries[1].val);

    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[2].used, expL2PrmDbEntries[2].used);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[2].id, expL2PrmDbEntries[2].id);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[2].val, expL2PrmDbEntries[2].val);

    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[3].used, expL2PrmDbEntries[3].used);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[3].id, expL2PrmDbEntries[3].id);
    ASSERT_EQ(this->activeL2Comp.m_prmDb.m_db[3].val, expL2PrmDbEntries[3].val);

    ASSERT_FALSE(this->activeL2Comp.m_prmDb.m_db[4].used);

  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_sendPrm_handler(
        const NATIVE_INT_TYPE portNum,
        bool morePrms,
        Fw::ParamList val
    )
  {
    this->pushFromPortEntry_sendPrm(morePrms, val);

    this->activeL2Comp.recvPrm_handlerBase(portNum, morePrms, val);
  }

  void Tester ::
    from_recvPrmReady_handler(
        const NATIVE_INT_TYPE portNum,
        U32 maxSize,
        bool reload
    )
  {
    this->pushFromPortEntry_recvPrmReady(maxSize, reload);

    this->activeL2Comp.sendPrmReady_handlerBase(portNum, maxSize, reload);
  }

  void Tester ::
    from_pingOut_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    this->pushFromPortEntry_pingOut(key);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {
    //this->component.set_sendPrm_OutputPort(0, this->activeL2Comp.get_recvPrm_InputPort(0));
    this->activeL2Comp.set_sendPrm_OutputPort(0, this->component.get_recvPrm_InputPort(0));

    //this->component.set_recvPrmReady_OutputPort(0, this->activeL2Comp.get_sendPrmReady_InputPort(0));
    this->activeL2Comp.set_recvPrmReady_OutputPort(0, this->component.get_sendPrmReady_InputPort(0));

    // sendPrmReady
    //for (NATIVE_INT_TYPE i = 0; i < 5; ++i) {
      //this->connect_to_sendPrmReady(
          //i,
          //this->component.get_sendPrmReady_InputPort(i)
      //);
    //}

    // setPrm
    this->connect_to_setPrm(
        0,
        this->component.get_setPrm_InputPort(0)
    );

    // recvPrm
    //for (NATIVE_INT_TYPE i = 0; i < 5; ++i) {
      //this->connect_to_recvPrm(
          //i,
          //this->component.get_recvPrm_InputPort(i)
      //);
    //}

    // pingIn
    this->connect_to_pingIn(
        0,
        this->component.get_pingIn_InputPort(0)
    );

    // getPrm
    this->connect_to_getPrm(
        0,
        this->component.get_getPrm_InputPort(0)
    );

    // CmdDisp
    this->connect_to_CmdDisp(
        0,
        this->component.get_CmdDisp_InputPort(0)
    );

    // sendPrm
    for (NATIVE_INT_TYPE i = 0; i < 5; ++i) {
      this->component.set_sendPrm_OutputPort(
          i, 
          this->get_from_sendPrm(i)
      );
    }


    // recvPrmReady
    for (NATIVE_INT_TYPE i = 0; i < 5; ++i) {
      this->component.set_recvPrmReady_OutputPort(
          i, 
          this->get_from_recvPrmReady(i)
      );
    }

    // pingOut
    this->component.set_pingOut_OutputPort(
        0, 
        this->get_from_pingOut(0)
    );

    // CmdStatus
    this->component.set_CmdStatus_OutputPort(
        0, 
        this->get_from_CmdStatus(0)
    );

    // CmdReg
    this->component.set_CmdReg_OutputPort(
        0, 
        this->get_from_CmdReg(0)
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
        QUEUE_DEPTH, INSTANCE
    );
    this->activeL2Comp.init(
        QUEUE_DEPTH, INSTANCE
    );
  }

} // end namespace Svc
