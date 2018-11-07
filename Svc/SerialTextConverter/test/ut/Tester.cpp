// ====================================================================== 
// \title  SerialTextConverter.hpp
// \author gcgandhi
// \brief  cpp file for SerialTextConverter test harness implementation class
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
#include <fstream>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10
#define QUEUE_DEPTH 10

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      SerialTextConverterGTestBase("Tester", MAX_HISTORY_SIZE),
      component("SerialTextConverter")
#else
      SerialTextConverterGTestBase(MAX_HISTORY_SIZE),
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
  run_all_tests(void)
  {
      Fw::Buffer test_buff;
      Drv::SerialReadStatus stat = Drv::SER_OK;

      printf("Two strings less than FW_LOG_STRING_MAX_SIZE bytes w/ end null\n");
      this->clearHistory();

      // Creating "abcd\0fgh\0".  Cant use strcat() as it gets rid of middle \0
      char test_buff_1[9];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_1); ++i) {

          if (i == 4 || i == (FW_NUM_ARRAY_ELEMENTS(test_buff_1)-1)) {
              test_buff_1[i] = '\0';
          }
          else {
              test_buff_1[i] = 'a' + i;
          }
      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_1),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_1));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(2);
      ASSERT_EVENTS_STC_SerialText(0,"abcd");
      ASSERT_EVENTS_STC_SerialText(1,"fgh");
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("Two strings less than FW_LOG_STRING_MAX_SIZE bytes w/o end null\n");
      this->clearHistory();

      // Creating "abcd\0fghi".  Cant use strcat() as it gets rid of middle \0
      char test_buff_11[9];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_11); ++i) {

          if (i == 4) {
              test_buff_11[i] = '\0';
          }
          else {
              test_buff_11[i] = 'a' + i;
          }
      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_11),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_11));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(2);
      ASSERT_EVENTS_STC_SerialText(0,"abcd");
      ASSERT_EVENTS_STC_SerialText(1,"fghi");
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("Two strings greater than FW_LOG_STRING_MAX_SIZE bytes w/ end null\n");
      this->clearHistory();

      // Creating "yzyzyz...y\0abc\0".  Cant use strcat() as it gets rid of middle \0
      char test_buff_2[FW_LOG_STRING_MAX_SIZE+4];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_2); ++i) {

          if (i == FW_LOG_STRING_MAX_SIZE-1 || i == (FW_NUM_ARRAY_ELEMENTS(test_buff_2)-1)) {
              test_buff_2[i] = '\0';
          }
          else if (i < FW_LOG_STRING_MAX_SIZE-1) {
              test_buff_2[i] = (i%2==0) ? 'y' : 'z';
          }
          else {
              test_buff_2[i] = 'a' + (i-FW_LOG_STRING_MAX_SIZE);
          }
      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_2),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_2));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(2);
      ASSERT_EVENTS_STC_SerialText(0,"yzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzy"); // 80 chars (including null)
      ASSERT_EVENTS_STC_SerialText(1,"abc"); // 4 chars (including null)
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("Two strings greater than FW_LOG_STRING_MAX_SIZE bytes w/o end null\n");
      this->clearHistory();

      // Creating "yzyzyz...y\0abcd".  Cant use strcat() as it gets rid of middle \0
      char test_buff_22[FW_LOG_STRING_MAX_SIZE+4];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_22); ++i) {

          if (i == FW_LOG_STRING_MAX_SIZE-1) {
              test_buff_22[i] = '\0';
          }
          else if (i < FW_LOG_STRING_MAX_SIZE-1) {
              test_buff_22[i] = (i%2==0) ? 'y' : 'z';
          }
          else {
              test_buff_22[i] = 'a' + (i-FW_LOG_STRING_MAX_SIZE);
          }
      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_22),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_22));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(2);
      ASSERT_EVENTS_STC_SerialText(0,"yzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzy"); // 40 chars (including null)
      ASSERT_EVENTS_STC_SerialText(1,"abcd"); // 5 chars (including null)
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("One string greater than FW_LOG_STRING_MAX_SIZE bytes w/ end null\n");
      this->clearHistory();

      // Creating "yzyzyz...yzabc\0".
      char test_buff_3[FW_LOG_STRING_MAX_SIZE+4];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_3); ++i) {

          if (i == (FW_NUM_ARRAY_ELEMENTS(test_buff_3)-1)) {
              test_buff_3[i] = '\0';
          }
          else if (i <= FW_LOG_STRING_MAX_SIZE-1) {
              test_buff_3[i] = (i%2==0) ? 'y' : 'z';
          }
          else {
              test_buff_3[i] = 'a' + (i-FW_LOG_STRING_MAX_SIZE);
          }
      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_3),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_3));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(2);
      ASSERT_EVENTS_STC_SerialText(0,"yzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzy"); // 40 chars (including null)
      ASSERT_EVENTS_STC_SerialText(1,"zabc"); // 5 chars (including null)
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("One string greater than FW_LOG_STRING_MAX_SIZE bytes w/o end null\n");
      this->clearHistory();

      // Creating "yzyzyz...yzabcd".
      char test_buff_33[FW_LOG_STRING_MAX_SIZE+4];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_33); ++i) {

          if (i <= FW_LOG_STRING_MAX_SIZE-1) {
              test_buff_33[i] = (i%2==0) ? 'y' : 'z';
          }
          else {
              test_buff_33[i] = 'a' + (i-FW_LOG_STRING_MAX_SIZE);
          }
      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_33),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_33));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(2);
      ASSERT_EVENTS_STC_SerialText(0,"yzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzy"); // 40 chars (including null)
      ASSERT_EVENTS_STC_SerialText(1,"zabcd"); // 6 chars (including null)
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("One string of FW_LOG_STRING_MAX_SIZE bytes w/ end null\n");
      this->clearHistory();

      // Creating "yzyzyz...y\0".
      char test_buff_4[FW_LOG_STRING_MAX_SIZE];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_4); ++i) {

          if (i == (FW_NUM_ARRAY_ELEMENTS(test_buff_4)-1)) {
              test_buff_4[i] = '\0';
          }
          else {
              test_buff_4[i] = (i%2==0) ? 'y' : 'z';
          }

      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_4),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_4));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(1);
      ASSERT_EVENTS_STC_SerialText(0,"yzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzy"); // 40 chars (including null)
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("One string of FW_LOG_STRING_MAX_SIZE bytes w/o end null\n");
      this->clearHistory();

      // Creating "yzyzyz...yz".
      char test_buff_5[FW_LOG_STRING_MAX_SIZE];
      for (U32 i = 0; i < FW_NUM_ARRAY_ELEMENTS(test_buff_4); ++i) {

          test_buff_5[i] = (i%2==0) ? 'y' : 'z';
      }
      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_5),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_5));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(0);
      ASSERT_EVENTS_STC_SerialText_SIZE(2);
      ASSERT_EVENTS_STC_SerialText(0,"yzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzyzy"); // 40 chars (including null)
      ASSERT_EVENTS_STC_SerialText(1,"z"); // 2 chars (including null)
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      printf("STC_SET_MODE command test\n");
      this->clearHistory();

      sendCmd_STC_SET_MODE(0, 0, SerialTextConverterComponentImpl::EVR_MODE);
      this->component.doDispatch();
      ASSERT_EQ(SerialTextConverterComponentImpl::EVR_MODE, this->component.m_mode);
      ASSERT_EVENTS_STC_SetMode_Cmd_Sent_SIZE(1);
      ASSERT_EVENTS_STC_SetMode_Cmd_Sent(0,SerialTextConverterComponentImpl::EVR_MODE_EV);
      ASSERT_EVENTS_STC_File_SIZE(0);
      ASSERT_CMD_RESPONSE_SIZE(1);
      ASSERT_CMD_RESPONSE(0,SerialTextConverterComponentImpl::OPCODE_STC_SET_MODE,0,Fw::COMMAND_OK);

      // Command will fail b/c no open file:
      sendCmd_STC_SET_MODE(0, 0, SerialTextConverterComponentImpl::FILE_MODE);
      this->component.doDispatch();
      ASSERT_EQ(SerialTextConverterComponentImpl::EVR_MODE, this->component.m_mode); // mode doesnt change
      ASSERT_EVENTS_STC_SetMode_Cmd_Sent_SIZE(1);
      ASSERT_EVENTS_STC_SetMode_Cmd_Invalid_SIZE(1);
      ASSERT_EVENTS_STC_File_SIZE(0);
      ASSERT_CMD_RESPONSE_SIZE(2);
      ASSERT_CMD_RESPONSE(1,SerialTextConverterComponentImpl::OPCODE_STC_SET_MODE,0,Fw::COMMAND_EXECUTION_ERROR);

      // Setup file for writing to:
      bool stat_file = this->component.set_log_file("test_file",512);

      ASSERT_TRUE(stat_file);
      ASSERT_TRUE(this->component.m_log_file.m_openFile);
      ASSERT_EQ(0,strcmp("test_file",this->component.m_log_file.m_fileName.toChar()));
      ASSERT_EQ(0U, this->component.m_log_file.m_currentFileSize);
      ASSERT_EQ(512U, this->component.m_log_file.m_maxFileSize);

      // Command will now succeed:
      sendCmd_STC_SET_MODE(0, 0, SerialTextConverterComponentImpl::FILE_MODE);
      this->component.doDispatch();
      ASSERT_EQ(SerialTextConverterComponentImpl::FILE_MODE, this->component.m_mode);
      ASSERT_EVENTS_STC_SetMode_Cmd_Sent_SIZE(2);
      ASSERT_EVENTS_STC_SetMode_Cmd_Sent(1,SerialTextConverterComponentImpl::FILE_MODE_EV);
      ASSERT_EVENTS_STC_File_SIZE(1);
      ASSERT_EVENTS_STC_File(0,"test_file");
      ASSERT_CMD_RESPONSE_SIZE(3);
      ASSERT_CMD_RESPONSE(2,SerialTextConverterComponentImpl::OPCODE_STC_SET_MODE,0,Fw::COMMAND_OK);

      test_buff.set(0,0,reinterpret_cast<U64>(test_buff_1),
                    FW_NUM_ARRAY_ELEMENTS(test_buff_1));

      // Send serial test data:
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      // Read file to verify contents:
      std::ifstream stream1("test_file");
      while(stream1) {
          char buf[256];
          stream1.getline(buf,256);
          if (stream1) {
              std::cout << "readLine: " << buf << std::endl;
              ASSERT_EQ(0,strcmp(test_buff_1,buf));
          }
      }
      stream1.close();

      printf("Bad serial read status\n");
      this->clearHistory();

      stat = Drv::SER_PARITY_ERR;
      this->invoke_to_SerReadPort(0,test_buff,stat);
      this->component.doDispatch();

      ASSERT_EVENTS_STC_SerialReadBadStatus_SIZE(1);
      ASSERT_EVENTS_STC_SerialReadBadStatus(0,stat);
      ASSERT_EVENTS_STC_SerialText_SIZE(0);
      ASSERT_from_SerialBufferSend_SIZE(1);
      test_buff.setsize(STC_READ_BUFF_SIZE);
      ASSERT_from_SerialBufferSend(0,test_buff);

      // Clean up:
      remove("test_file");
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_SerialBufferSend_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    this->pushFromPortEntry_SerialBufferSend(fwBuffer);
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

      // SerReadPort
      this->connect_to_SerReadPort(
          0,
          this->component.get_SerReadPort_InputPort(0)
      );

      // CmdDisp
      this->connect_to_CmdDisp(
          0,
          this->component.get_CmdDisp_InputPort(0)
      );

      // SerialBufferSend
      this->component.set_SerialBufferSend_OutputPort(
          0,
          this->get_from_SerialBufferSend(0)
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

  void Tester::textLogIn(
                   const FwEventIdType id, //!< The event ID
                   Fw::Time& timeTag, //!< The time
                   const Fw::TextLogSeverity severity, //!< The severity
                   const Fw::TextLogString& text //!< The event string
               ) {
       TextLogEntry e = { id, timeTag, severity, text };

       printTextLogHistoryEntry(e,stdout);
   }

} // end namespace Svc
