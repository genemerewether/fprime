// ====================================================================== 
// \title  ActiveL2PrmDb.hpp
// \author kubiak
// \brief  cpp file for ActiveL2PrmDb test harness implementation class
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

#define MAX_RECV_SIZE 128
namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      ActiveL2PrmDbGTestBase("Tester", MAX_HISTORY_SIZE),
      component("ActiveL2PrmDb", MAX_RECV_SIZE)
#else
      ActiveL2PrmDbGTestBase(MAX_HISTORY_SIZE),
      component(MAX_RECV_SIZE)
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
    requestPrmsTest(void) 
  {
    Fw::ParamList prmList;
    Fw::ParamValid valid;
    Fw::ParamBuffer prmBuff;

    struct prmPair prms[] = {
        {1, 10},
        {2, 50},
        {3, "MyString"},
        {6, -1000}
    };

    U8 sendList1[] = {
//      PrmId    Length BufferData
        0,0,0,1, 0,4,   0,0,0,10, // Parameter 1 has a value of 10
        0,0,0,3, 0,10,  0,8,'M','y','S','t','r','i','n','g' // Parameter 3 has a value of "MyString"
    };

    U8 sendList2[] = {
//      PrmId    Length BufferData
        0,0,0,2, 0,4,   0,0,0,50, // Parameter 2 has a value of 50
        0,0,0,6, 0,4,   0xFF,0xFF,0xFC,0x18 // Parameter 6 has a value of -1000
    };

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_EVENTS_SIZE(0);
    ASSERT_from_recvPrmReady_SIZE(1);
    ASSERT_from_recvPrmReady(0, MAX_RECV_SIZE, true);

    prmList.setBuff(sendList1, sizeof(sendList1));

    this->clearHistory();
    this->invoke_to_recvPrm(0, true, prmList);
    this->component.doDispatch();

    ASSERT_EVENTS_SIZE(0);
    ASSERT_from_recvPrmReady_SIZE(1);
    ASSERT_from_recvPrmReady(0, MAX_RECV_SIZE, false);

    prmList.setBuff(sendList2, sizeof(sendList2));

    this->clearHistory();
    this->invoke_to_recvPrm(0, false, prmList);
    this->component.doDispatch();

    ASSERT_EVENTS_SIZE(0);
    ASSERT_from_recvPrmReady_SIZE(1);
    ASSERT_from_recvPrmReady(0, MAX_RECV_SIZE, false);

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_EVENTS_SIZE(0);
    ASSERT_from_sendPrm_SIZE(0);

    for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

        prmBuff.resetSer();

        this->clearHistory();
        valid = this->invoke_to_getPrm(0, prms[idx].id, prmBuff);
        ASSERT_EQ(Fw::PARAM_VALID, valid);

        ASSERT_EVENTS_SIZE(0);

        ASSERT_EQ(prms[idx].prm.getBuffLength(),
                  prmBuff.getBuffLength());

        ASSERT_EQ(0, memcmp(prms[idx].prm.getBuffAddr(),
                            prmBuff.getBuffAddr(),
                            prmBuff.getBuffLength()));

    }

    prmBuff.resetSer();

    this->clearHistory();
    valid = this->invoke_to_getPrm(0, 100, prmBuff);
    ASSERT_EQ(Fw::PARAM_INVALID, valid);

  }

  void Tester ::
    setGetTest(void) 
  {
    Fw::ParamValid valid;
    Fw::ParamBuffer prmBuff;

    struct prmPair prms[] = {
        {1, 10},
        {2, 50},
        {3, "MyString"},
        {5, "Skipped 4"},
        {6, -1000}
    };

    for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {
        this->clearHistory();
        this->invoke_to_setPrm(0, prms[idx].id, prms[idx].prm);
        this->component.doDispatch();

        ASSERT_EVENTS_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded(0, prms[idx].id);
    }

    for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

        prmBuff.resetSer();

        this->clearHistory();
        valid = this->invoke_to_getPrm(0, prms[idx].id, prmBuff);
        ASSERT_EQ(Fw::PARAM_VALID, valid);

        ASSERT_EVENTS_SIZE(0);

        ASSERT_EQ(prms[idx].prm.getBuffLength(),
                  prmBuff.getBuffLength());

        ASSERT_EQ(0, memcmp(prms[idx].prm.getBuffAddr(),
                            prmBuff.getBuffAddr(),
                            prmBuff.getBuffLength()));

    }

    prmBuff.resetSer();

    this->clearHistory();
    valid = this->invoke_to_getPrm(0, 100, prmBuff);
    ASSERT_EQ(Fw::PARAM_INVALID, valid);
  }

  void Tester ::
    setSendTest(void) 
  {
    Fw::ParamValid valid;
    Fw::ParamBuffer prmBuff;
    Fw::ExternalSerializeBuffer expBuffer;

    struct prmPair prms[] = {
        {1, 10},
        {3, "MyString"},
        {2, 50},
        {6, -1000}
    };

    U8 sendList1[] = {
//      PrmId    Length BufferData
        0,0,0,1, 0,4,   0,0,0,10, // Parameter 1 has a value of 10
    };

    U8 sendList2[] = {
        0,0,0,3, 0,10,  0,8,'M','y','S','t','r','i','n','g' // Parameter 3 has a value of "MyString"
    };

    U8 sendList3[] = {
//      PrmId    Length BufferData
        0,0,0,2, 0,4,   0,0,0,50, // Parameter 2 has a value of 50
        0,0,0,6, 0,4,   0xFF,0xFF,0xFC,0x18 // Parameter 6 has a value of -1000
    };

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    this->sendCmd_L2_PRM_PUSH_UPDATES(0, 0, ActiveL2PrmDbComponentImpl::PUSH_UPDATES);
    this->component.doDispatch();

    this->invoke_to_sendPrmReady(0, 21, false);
    this->component.doDispatch();

    this->clearHistory();
    this->invoke_to_setPrm(0, prms[0].id, prms[0].prm);
    this->component.doDispatch();

    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_PrmIdAdded_SIZE(1);
    ASSERT_EVENTS_PrmIdAdded(0, prms[0].id);

    expBuffer.setExtBuffer(sendList1, sizeof(sendList1));
    expBuffer.setBuffLen(sizeof(sendList1));

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_from_sendPrm_SIZE(1);
    ASSERT_from_sendPrm(0, false, expBuffer);

    for (int idx = 1; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {
        this->clearHistory();
        this->invoke_to_setPrm(0, prms[idx].id, prms[idx].prm);
        this->component.doDispatch();

        ASSERT_EVENTS_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded(0, prms[idx].id);
    }

    this->invoke_to_sendPrmReady(0, 21, false);
    this->component.doDispatch();

    expBuffer.setExtBuffer(sendList2, sizeof(sendList2));
    expBuffer.setBuffLen(sizeof(sendList2));

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_from_sendPrm_SIZE(1);
    ASSERT_from_sendPrm(0, true, expBuffer);

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_from_sendPrm_SIZE(0);

    this->invoke_to_sendPrmReady(0, 21, false);
    this->component.doDispatch();

    expBuffer.setExtBuffer(sendList3, sizeof(sendList3));
    expBuffer.setBuffLen(sizeof(sendList3));

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_from_sendPrm_SIZE(1);
    ASSERT_from_sendPrm(0, false, expBuffer);
  }

  void Tester ::
    sendTooLargeTest(void) 
  {
    Fw::ParamValid valid;
    Fw::ParamBuffer prmBuff;
    Fw::ExternalSerializeBuffer expBuffer;

    struct prmPair prms[] = {
        {1, 10},
        {3, "MyString"},
        {2, 50},
        {6, -1000}
    };

    U8 sendList1[] = {
//      PrmId    Length BufferData
        0,0,0,1, 0,4,   0,0,0,10, // Parameter 1 has a value of 10
    };

    U8 sendList2[] = {
        0,0,0,3, 0,10,  0,8,'M','y','S','t','r','i','n','g' // Parameter 3 has a value of "MyString"
    };

    U8 sendList3[] = {
//      PrmId    Length BufferData
        0,0,0,2, 0,4,   0,0,0,50, // Parameter 2 has a value of 50
    };

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    this->sendCmd_L2_PRM_PUSH_UPDATES(0, 0, ActiveL2PrmDbComponentImpl::PUSH_UPDATES);
    this->component.doDispatch();

    for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {
        this->clearHistory();
        this->invoke_to_setPrm(0, prms[idx].id, prms[idx].prm);
        this->component.doDispatch();

        ASSERT_EVENTS_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded(0, prms[idx].id);
    }


    this->invoke_to_sendPrmReady(0, 11, false);
    this->component.doDispatch();

    expBuffer.setExtBuffer(sendList1, sizeof(sendList1));
    expBuffer.setBuffLen(sizeof(sendList1));

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_from_sendPrm_SIZE(1);
    ASSERT_from_sendPrm(0, true, expBuffer);

    this->invoke_to_sendPrmReady(0, 11, false);
    this->component.doDispatch();

    expBuffer.setExtBuffer(sendList3, sizeof(sendList1));
    expBuffer.setBuffLen(sizeof(sendList3));

    this->clearHistory();
    this->invoke_to_sched(0, 0);
    this->component.doDispatch();

    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_PrmSendTooLarge_SIZE(1);
    ASSERT_EVENTS_PrmSendTooLarge(0, 3, 16);

    ASSERT_from_sendPrm_SIZE(1);
    ASSERT_from_sendPrm(0, true, expBuffer);
  }

  void Tester ::
    sendBuffOvfTest(void) 
  {
    prmPair prm = {1, 10};

    this->sendCmd_L2_PRM_PUSH_UPDATES(0, 0, ActiveL2PrmDbComponentImpl::PUSH_UPDATES);
    this->component.doDispatch();

    for (int idx = 0; idx < ACTIVE_L2_PRMDB_SEND_BUFFER_ENTRIES; idx++) {
        this->clearHistory();
        this->invoke_to_setPrm(0, idx, prm.prm);
        this->component.doDispatch();

        ASSERT_EVENTS_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded_SIZE(1);
        ASSERT_EVENTS_PrmIdAdded(0, idx);
    }

    for (int idx = 0; idx < ACTIVE_L2_PRMDB_SEND_BUFFER_ENTRIES; idx++) {
        this->clearHistory();
        this->invoke_to_setPrm(0, idx, prm.prm);
        this->component.doDispatch();

        ASSERT_EVENTS_SIZE(1);
        ASSERT_EVENTS_PrmIdUpdated_SIZE(1);
        ASSERT_EVENTS_PrmIdUpdated(0, idx);
    }

    this->clearHistory();
    this->invoke_to_setPrm(0, ACTIVE_L2_PRMDB_SEND_BUFFER_ENTRIES, prm.prm);
    this->component.doDispatch();

    ASSERT_EVENTS_SIZE(2);
    ASSERT_EVENTS_PrmIdAdded_SIZE(1);
    ASSERT_EVENTS_PrmIdAdded(0, ACTIVE_L2_PRMDB_SEND_BUFFER_ENTRIES);

    ASSERT_EVENTS_PrmSendBufferFull_SIZE(1);
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
  }

  void Tester ::
    from_recvPrmReady_handler(
        const NATIVE_INT_TYPE portNum,
        U32 maxSize,
        bool reload
    )
  {
    this->pushFromPortEntry_recvPrmReady(maxSize, reload);
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


    // sched
    this->connect_to_sched(
        0,
        this->component.get_sched_InputPort(0)
    );

    // sendPrmReady
    this->connect_to_sendPrmReady(
        0,
        this->component.get_sendPrmReady_InputPort(0)
    );

    // setPrm
    this->connect_to_setPrm(
        0,
        this->component.get_setPrm_InputPort(0)
    );

    // recvPrm
    this->connect_to_recvPrm(
        0,
        this->component.get_recvPrm_InputPort(0)
    );

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
    this->component.set_sendPrm_OutputPort(
        0, 
        this->get_from_sendPrm(0)
    );

    // recvPrmReady
    this->component.set_recvPrmReady_OutputPort(
        0, 
        this->get_from_recvPrmReady(0)
    );

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
  }

} // end namespace Svc
