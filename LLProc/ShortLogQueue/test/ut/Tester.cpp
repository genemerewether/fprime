// ====================================================================== 
// \title  ShortLogQueue.hpp
// \author tcanham
// \brief  cpp file for ShortLogQueue test harness implementation class
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

namespace LLProc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      ShortLogQueueGTestBase("Tester", MAX_HISTORY_SIZE),
      component("ShortLogQueue")
#else
      ShortLogQueueGTestBase(MAX_HISTORY_SIZE),
      component()
#endif
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

  void Tester ::
    doNominalOneIn(void)
  {
      // cycle three times around, inserting and removing one record at a time
      for (NATIVE_UINT_TYPE cycle = 0; cycle < 3*SHORT_LOG_QUEUE_DEPTH; cycle++) {
          //printf("Cycle: %d\n",cycle);
          Fw::LogBuffer args;
          U8* argsPtr = args.getBuffAddr();
          for (NATIVE_UINT_TYPE byte = 0; byte < 10; byte++) {
              argsPtr[byte] = cycle+byte;
          }
          Fw::Time dummy;
          ASSERT_TRUE(args.setBuffLen(10) == Fw::FW_SERIALIZE_OK);
          this->clearFromPortHistory();
          this->invoke_to_LogRecv(0,cycle,dummy,Fw::LOG_WARNING_LO,args);
          // check head and tail. Head should have advanced.
          ASSERT_EQ((cycle+1)%SHORT_LOG_QUEUE_DEPTH,this->component.m_head);
          ASSERT_EQ((cycle)%SHORT_LOG_QUEUE_DEPTH,this->component.m_tail);
          this->invoke_to_Run(0,0);
          ASSERT_from_LogSend_SIZE(1);
          ASSERT_from_LogSend(0,cycle,Fw::LOG_WARNING_LO,args);
          // check head and tail. Tail should have caught up.
          ASSERT_EQ((cycle+1)%SHORT_LOG_QUEUE_DEPTH,this->component.m_head);
          ASSERT_EQ((cycle+1)%SHORT_LOG_QUEUE_DEPTH,this->component.m_tail);
      }
  }

  void Tester ::
    doNominalAllIn(void)
  {
      // Fill the buffer with entries
      for (NATIVE_UINT_TYPE cycle = 0; cycle < SHORT_LOG_QUEUE_DEPTH-1; cycle++) {
          //printf("Cycle: %d\n",cycle);
          Fw::LogBuffer args;
          U8* argsPtr = args.getBuffAddr();
          for (NATIVE_UINT_TYPE byte = 0; byte < (NATIVE_UINT_TYPE)10; byte++) {
              argsPtr[byte] = cycle+byte;
          }
          Fw::Time dummy;
          ASSERT_TRUE(args.setBuffLen(10) == Fw::FW_SERIALIZE_OK);
          this->invoke_to_LogRecv(0,cycle,dummy,Fw::LOG_WARNING_LO,args);
          // check head and tail. Head should have advanced. Tail should remain the same
          ASSERT_EQ((cycle+1)%SHORT_LOG_QUEUE_DEPTH,this->component.m_head);
          ASSERT_EQ((NATIVE_UINT_TYPE)0,this->component.m_tail);
      }

      // cycle the component, which should read out one at a time
      for (NATIVE_UINT_TYPE cycle = 0; cycle < SHORT_LOG_QUEUE_DEPTH-1; cycle++) {
          //printf("Cycle: %d\n",cycle);
          Fw::LogBuffer args;
          U8* argsPtr = args.getBuffAddr();
          for (NATIVE_UINT_TYPE byte = 0; byte < 10; byte++) {
              argsPtr[byte] = cycle+byte;
          }
          Fw::Time dummy;
          ASSERT_TRUE(args.setBuffLen(10) == Fw::FW_SERIALIZE_OK);
          this->clearFromPortHistory();
          this->invoke_to_Run(0,0);
          ASSERT_from_LogSend_SIZE(1);
          ASSERT_from_LogSend(0,cycle,Fw::LOG_WARNING_LO,args);
          // check head and tail. Head should be at end, Tail should have caught up.
          ASSERT_EQ(SHORT_LOG_QUEUE_DEPTH-1,this->component.m_head);
          ASSERT_EQ((cycle+1)%SHORT_LOG_QUEUE_DEPTH,this->component.m_tail);
      }
  }

  void Tester ::
    doNominalSomeIn(void)
  {
      NATIVE_UINT_TYPE BURST_SIZE = 15;
      // cycle three times around, inserting and removing one record at a time
      for (NATIVE_UINT_TYPE cycle = 0; cycle < (3*SHORT_LOG_QUEUE_DEPTH)/BURST_SIZE; cycle++) {
          for (NATIVE_UINT_TYPE burst = 0; burst < BURST_SIZE; burst++) {
              //printf("Cycle: %d Burst: %d\n",cycle,burst);
              Fw::LogBuffer args;
              U8* argsPtr = args.getBuffAddr();
              for (NATIVE_UINT_TYPE byte = 0; byte < 10; byte++) {
                  argsPtr[byte] = cycle*burst+byte;
              }
              Fw::Time dummy;
              ASSERT_TRUE(args.setBuffLen(10) == Fw::FW_SERIALIZE_OK);
              this->invoke_to_LogRecv(0,cycle*burst,dummy,Fw::LOG_WARNING_LO,args);
              // check head and tail. Head should have advanced.
              ASSERT_EQ((cycle*BURST_SIZE+burst+1)%SHORT_LOG_QUEUE_DEPTH,this->component.m_head);
              ASSERT_EQ((cycle*BURST_SIZE)%SHORT_LOG_QUEUE_DEPTH,this->component.m_tail);
          }
          for (NATIVE_UINT_TYPE burst = 0; burst < BURST_SIZE; burst++) {
              Fw::LogBuffer args;
              U8* argsPtr = args.getBuffAddr();
              for (NATIVE_UINT_TYPE byte = 0; byte < 10; byte++) {
                  argsPtr[byte] = cycle*burst+byte;
              }
              Fw::Time dummy;
              ASSERT_TRUE(args.setBuffLen(10) == Fw::FW_SERIALIZE_OK);
              this->clearFromPortHistory();
              this->invoke_to_Run(0,0);
              ASSERT_from_LogSend_SIZE(1);
              ASSERT_from_LogSend(0,cycle*burst,Fw::LOG_WARNING_LO,args);
              // check head and tail. Tail should have caught up.
              ASSERT_EQ(((cycle+1)*BURST_SIZE)%SHORT_LOG_QUEUE_DEPTH,this->component.m_head);
              ASSERT_EQ((cycle*BURST_SIZE+burst+1)%SHORT_LOG_QUEUE_DEPTH,this->component.m_tail);
          }
      }
  }



  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_LogSend_handler(
        const NATIVE_INT_TYPE portNum,
        FwEventIdType id,
        U8 severity,
        Fw::LogBuffer &args
    )
  {
    this->pushFromPortEntry_LogSend(id, severity, args);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // LogRecv
    this->connect_to_LogRecv(
        0,
        this->component.get_LogRecv_InputPort(0)
    );

    // Run
    this->connect_to_Run(
        0,
        this->component.get_Run_InputPort(0)
    );

    // LogSend
    this->component.set_LogSend_OutputPort(
        0, 
        this->get_from_LogSend(0)
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

} // end namespace LLProc
