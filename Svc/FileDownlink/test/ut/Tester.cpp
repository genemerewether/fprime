// ====================================================================== 
// \title  Tester.cpp
// \author bocchino
// \brief  cpp file for FileDownlink test harness implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
// 
// ====================================================================== 

#include <errno.h>
#include <unistd.h>

#include "Tester.hpp"

#define INSTANCE 0
#define CMD_SEQ 0
#define QUEUE_DEPTH 10
#define DOWNLINK_PACKET_SIZE 100
#define MANAGER_ID 100
#define BUFFER_ID 200

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
      FileDownlinkGTestBase("Tester", MAX_HISTORY_SIZE),
      component("FileDownlink", DOWNLINK_PACKET_SIZE, 1000, 100)
  {
    this->connectPorts();
    this->initComponents();
    this->buff_returned = false;
  }

  Tester ::
    ~Tester(void) 
  {
    
  }

  // ----------------------------------------------------------------------
  // Tests 
  // ----------------------------------------------------------------------

  void Tester ::
    downlink(void) 
  {

    // Assert idle mode
    ASSERT_EQ(FileDownlink::Mode::IDLE, this->component.mode.get());

    // Create a file
    const char *const sourceFileName = "source.bin";
    const char *const destFileName = "dest.bin";
    U8 data[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    FileBuffer fileBufferOut(data, sizeof(data));
    fileBufferOut.write(sourceFileName);

    // Send the file and assert COMMAND_OK
    this->sendFile(sourceFileName, destFileName, Fw::COMMAND_OK);

    // Assert telemetry
    ASSERT_TLM_SIZE(4);
    ASSERT_TLM_FileDownlink_PacketsSent_SIZE(3);
    for (size_t i = 0; i < 3; ++i)
      ASSERT_TLM_FileDownlink_PacketsSent(i, i+1);
    ASSERT_TLM_FileDownlink_FilesSent_SIZE(1);
    ASSERT_TLM_FileDownlink_FilesSent(0, 1);

    // Assert events
    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_FileDownlink_FileSent_SIZE(1);
//    printTextLogHistory(stdout);
    //ASSERT_EVENTS_FileDownlink_FileSent(0, sourceFileName, destFileName);

    // Validate the packet history
    History<Fw::FilePacket::DataPacket> dataPackets(MAX_HISTORY_SIZE);
    CFDP::Checksum checksum;
    fileBufferOut.getChecksum(checksum);
    validatePacketHistory(
        *this->fromPortHistory_bufferSendOut,
        dataPackets,
        Fw::FilePacket::T_END,
        3,
        checksum,
        0
    );

    // Compare the outgoing and incoming files
    FileBuffer fileBufferIn(dataPackets);
    ASSERT_EQ(true, FileBuffer::compare(fileBufferIn, fileBufferOut));

    // Assert idle mode
    ASSERT_EQ(FileDownlink::Mode::IDLE, this->component.mode.get());

    // Remove the outgoing file
    this->removeFile(sourceFileName);

  }

  void Tester ::
    fileOpenError(void) 
  {
    
    const char *const sourceFileName = "missing_directory/source.bin";
    const char *const destFileName = "dest.bin";

    // Send the file and assert COMMAND_EXECUTION_ERROR
    this->sendFile(
        sourceFileName, 
        destFileName,
        Fw::COMMAND_EXECUTION_ERROR
    );

    // Assert telemetry
    ASSERT_TLM_SIZE(1);
    ASSERT_TLM_FileDownlink_Warnings(0, 1);

    // Assert events
    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_FileDownlink_FileOpenError_SIZE(1);
//    ASSERT_EVENTS_FileDownlink_FileOpenError(0, sourceFileName);

  }

  void Tester ::
    cancelDownlink(void) 
  {
    // Create a file
    const char *const sourceFileName = "source.bin";
    const char *const destFileName = "dest.bin";
    U8 data[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    FileBuffer fileBufferOut(data, sizeof(data));
    fileBufferOut.write(sourceFileName);

    // Send a file
    Fw::CmdStringArg sourceCmdStringArg(sourceFileName);
    Fw::CmdStringArg destCmdStringArg(destFileName);
    this->sendCmd_FILE_DWN_SEND_FILE(
        INSTANCE, 
        CMD_SEQ, 
        sourceCmdStringArg,
        destCmdStringArg
    );
    this->component.doDispatch();

    // Send a cancel command and assert COMMAND_OK
    this->cancel(Fw::COMMAND_OK);

    // Assert CANCEL mode
    ASSERT_EQ(FileDownlink::Mode::CANCEL, this->component.mode.get());

    // Clear the command response history
    this->cmdResponseHistory->clear();

    this->component.Run_handler(0,0);

    // Assert telemetry
    ASSERT_TLM_SIZE(2);

    // Assert cancel event
    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_FileDownlink_DownlinkCanceled_SIZE(1);
//    ASSERT_EVENTS_FileDownlink_DownlinkCanceled(
//        0,
//        "source.bin",
//        "dest.bin"
//    );

    // Assert IDLE mode
    ASSERT_EQ(FileDownlink::Mode::IDLE, this->component.mode.get());
    
    // Remove the outgoing file
    this->removeFile(sourceFileName);
  }

  void Tester ::
    cancelInIdleMode(void) 
  {
    // Assert idle mode
    ASSERT_EQ(FileDownlink::Mode::IDLE, this->component.mode.get());

    // Send a cancel command
    this->cancel(Fw::COMMAND_OK);
    
    this->component.Run_handler(0,0);
    // Assert idle mode
    ASSERT_EQ(FileDownlink::Mode::IDLE, this->component.mode.get());

  }

  void Tester ::
    downlinkPartial(void)
  {
    // Assert idle mode
    ASSERT_EQ(FileDownlink::Mode::IDLE, this->component.mode.get());

    // Create a file
    const char *const sourceFileName = "source.bin";
    const char *const destFileName = "dest.bin";
    U8 data[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
    U8 dataSubset[] = { 1, 2, 3, 4 };
    U32 offset = 1;
    U32 length = 4;

    FileBuffer fileBufferOut(data, sizeof(data));
    fileBufferOut.write(sourceFileName);
    FileBuffer fileBufferOutSubset(dataSubset, sizeof(dataSubset));

    // Send the file and assert COMMAND_OK
    this->sendFilePartial(sourceFileName, destFileName, Fw::COMMAND_OK, offset, length);

    // Assert telemetry
    ASSERT_TLM_SIZE(4);
    ASSERT_TLM_FileDownlink_PacketsSent_SIZE(3);
    for (size_t i = 0; i < 3; ++i)
      ASSERT_TLM_FileDownlink_PacketsSent(i, i+1);
    ASSERT_TLM_FileDownlink_FilesSent_SIZE(1);
    ASSERT_TLM_FileDownlink_FilesSent(0, 1);

    // Assert events
    ASSERT_EVENTS_SIZE(1);
    ASSERT_EVENTS_FileDownlink_FileSent_SIZE(1);
//    printTextLogHistory(stdout);
    //ASSERT_EVENTS_FileDownlink_FileSent(0, sourceFileName, destFileName);

    // Validate the packet history
    History<Fw::FilePacket::DataPacket> dataPackets(MAX_HISTORY_SIZE);

    CFDP::Checksum checksum;
    checksum.update(dataSubset, offset, length);

    validatePacketHistory(
        *this->fromPortHistory_bufferSendOut,
        dataPackets,
        Fw::FilePacket::T_END,
        3,
        checksum,
        1
    );

    // Compare the outgoing and incoming files
    FileBuffer fileBufferIn(dataPackets);
    ASSERT_EQ(true, FileBuffer::compare(fileBufferIn, fileBufferOutSubset));

    // Assert idle mode
    ASSERT_EQ(FileDownlink::Mode::IDLE, this->component.mode.get());

    // Remove the outgoing file
    this->removeFile(sourceFileName);

  }

  // ----------------------------------------------------------------------
  // Handlers for from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_buffSendOutReturn_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer fwBuffer
    )
  {
    this->pushFromPortEntry_buffSendOutReturn(fwBuffer);
  }

  Fw::Buffer Tester ::
    from_bufferGetCaller_handler(
        const NATIVE_INT_TYPE portNum,
        U32 size
    )
  {
    U8 *const data = new U8[size];
    Fw::Buffer buffer(
        MANAGER_ID,
        BUFFER_ID,
        reinterpret_cast<U64>(data),
        size
    );
    return buffer;
  }

  void Tester ::
    from_bufferSendOut_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer& buffer
    )
  {
    pushFromPortEntry_bufferSendOut(buffer);
    this->buff_returned = true;
  }

  void Tester ::
    from_faultOut_handler(
        const NATIVE_INT_TYPE portNum,
        U32 FaultID,
        U32 eventContext
    )
  {
    this->pushFromPortEntry_faultOut(FaultID, eventContext);
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
  // Private instance methods
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {
    // cmdIn
    this->connect_to_cmdIn(
        0,
        this->component.get_cmdIn_InputPort(0)
    );

    // Run
    this->connect_to_Run(
        0,
        this->component.get_Run_InputPort(0)
    );

    // bufferReturn
    this->connect_to_bufferReturn(
        0,
        this->component.get_bufferReturn_InputPort(0)
    );

    // buffSendOutReturn
    this->component.set_buffSendOutReturn_OutputPort(
        0, 
        this->get_from_buffSendOutReturn(0)
    );

    // timeCaller
    this->component.set_timeCaller_OutputPort(
        0, 
        this->get_from_timeCaller(0)
    );

    // cmdRegOut
    this->component.set_cmdRegOut_OutputPort(
        0, 
        this->get_from_cmdRegOut(0)
    );

    // eventOut
    this->component.set_eventOut_OutputPort(
        0, 
        this->get_from_eventOut(0)
    );

    // bufferGetCaller
    this->component.set_bufferGetCaller_OutputPort(
        0, 
        this->get_from_bufferGetCaller(0)
    );

    // bufferSendOut
    this->component.set_bufferSendOut_OutputPort(
        0, 
        this->get_from_bufferSendOut(0)
    );

    // tlmOut
    this->component.set_tlmOut_OutputPort(
        0, 
        this->get_from_tlmOut(0)
    );

    // cmdResponseOut
    this->component.set_cmdResponseOut_OutputPort(
        0, 
        this->get_from_cmdResponseOut(0)
    );

    // faultOut
    this->component.set_faultOut_OutputPort(
        0, 
        this->get_from_faultOut(0)
    );

    // LogText
    this->component.set_LogText_OutputPort(
        0, 
        this->get_from_LogText(0)
    );

    // pingIn
    this->connect_to_pingIn(
        0,
        this->component.get_pingIn_InputPort(0)
    );

    // pingOut
    this->component.set_pingOut_OutputPort(
        0,
        this->get_from_pingOut(0)
    );

  }

  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        QUEUE_DEPTH,
        INSTANCE
    );
  }

  void Tester ::
    sendFile(
        const char *const sourceFileName,
        const char *const destFileName,
        const Fw::CommandResponse response
    )
  {
    Fw::Buffer mybuf;
    U32 filesSent = this->component.filesSent.n;
    // Command the File Downlink component to send the file
    Fw::CmdStringArg sourceCmdStringArg(sourceFileName);
    Fw::CmdStringArg destCmdStringArg(destFileName);
    this->sendCmd_FILE_DWN_SEND_FILE(
        INSTANCE, 
        CMD_SEQ, 
        sourceCmdStringArg,
        destCmdStringArg
    );
    this->component.doDispatch();
    
    // Assert command response if expecting error
    if (response != Fw::COMMAND_OK) {
       ASSERT_CMD_RESPONSE_SIZE(1);
       ASSERT_CMD_RESPONSE(
           0,
           FileDownlink::OPCODE_FILE_DWN_SEND_FILE,
           CMD_SEQ,
           response
       );
       return;
    }

    while (this->component.filesSent.n < filesSent + 1) { 
       if (this->buff_returned) {
          mybuf = from_bufferGetCaller_handler(0, DOWNLINK_PACKET_SIZE);
          this->component.bufferReturn_handler(0, mybuf); 
          this->buff_returned = false;
       }
       this->component.Run_handler(0,0);
    }

  }

  void Tester ::
    sendFilePartial(
      const char *const sourceFileName, //!< The source file name
      const char *const destFileName, //!< The destination file name
      const Fw::CommandResponse response, //!< The expected command response
      U32 startIndex, //!< The starting index
      U32 length //!< The amount of bytes to downlink
    )
  {
    Fw::Buffer mybuf;
    U32 filesSent = this->component.filesSent.n;
    // Command the File Downlink component to send the file
    Fw::CmdStringArg sourceCmdStringArg(sourceFileName);
    Fw::CmdStringArg destCmdStringArg(destFileName);
    this->sendCmd_FILE_DWN_SEND_PARTIAL(
            INSTANCE,
            CMD_SEQ,
            sourceCmdStringArg,
            destCmdStringArg,
            startIndex,
            length
    );
    this->component.doDispatch();

    // Assert command response if expecting error
    if (response != Fw::COMMAND_OK) {
        ASSERT_CMD_RESPONSE_SIZE(1);
        ASSERT_CMD_RESPONSE(
                0,
                FileDownlink::OPCODE_FILE_DWN_SEND_FILE,
                CMD_SEQ,
                response
        );
        return;
    }

    while (this->component.filesSent.n < filesSent + 1) {
        if (this->buff_returned) {
            mybuf = from_bufferGetCaller_handler(0, DOWNLINK_PACKET_SIZE);
            this->component.bufferReturn_handler(0, mybuf);
            this->buff_returned = false;
        }
        this->component.Run_handler(0,0);
    }
  }

  void Tester ::
    cancel(const Fw::CommandResponse response)
  {
    // Command the File Downlink component to cancel a file downlink
    this->sendCmd_FILE_DWN_CANCEL(INSTANCE, CMD_SEQ);
    // Assert command response
    ASSERT_CMD_RESPONSE_SIZE(1);
    ASSERT_CMD_RESPONSE(
        0,
        FileDownlink::OPCODE_FILE_DWN_CANCEL,
        CMD_SEQ,
        response
    );
  }

  void Tester ::
    removeFile(const char *const name)
  {
    const NATIVE_INT_TYPE status = ::unlink(name);
    if (status != 0) {
      // OK if file is not there
      ASSERT_EQ(ENOENT, errno);
    }
  }

  // ----------------------------------------------------------------------
  // Private static methods
  // ---------------------------------------------------------------------- 

  void Tester ::
    validatePacketHistory(
        const History<FromPortEntry_bufferSendOut>& historyIn,
        History<Fw::FilePacket::DataPacket>& historyOut,
        const Fw::FilePacket::Type finalPacketType,
        const size_t numPackets,
        const CFDP::Checksum& checksum,
        U32 startOffset
    )
  {

    FW_ASSERT(numPackets > 0);

    const size_t size = historyIn.size();
    ASSERT_EQ(numPackets, size);

    {
      const Fw::Buffer& buffer = historyIn.at(0).fwBuffer;
      validateStartPacket(buffer);
    }

    for (size_t i = 1; i < size - 1; ++i) {
      const Fw::Buffer& buffer = historyIn.at(i).fwBuffer;
      Fw::FilePacket::DataPacket dataPacket;
      validateDataPacket(
          buffer,
          dataPacket,
          i,
          startOffset
      );
      historyOut.push_back(dataPacket);
    }

    const size_t index = size - 1;
    const Fw::Buffer& buffer = historyIn.at(index).fwBuffer;
    switch (finalPacketType) {
      case Fw::FilePacket::T_END: {
        validateEndPacket(buffer, index, checksum);
        break;
      }
      case Fw::FilePacket::T_CANCEL: {
        validateCancelPacket(buffer, index);
        break;
      }
      default:
        FW_ASSERT(0);
    }

  }

  void Tester ::
    validateFilePacket(
        const Fw::Buffer& buffer,
        Fw::FilePacket& filePacket
    )
  {
    const Fw::SerializeStatus status = filePacket.fromBuffer(buffer);
    ASSERT_EQ(Fw::FW_SERIALIZE_OK, status);
  }

  void Tester ::
    validateStartPacket(const Fw::Buffer& buffer)
  {
    Fw::FilePacket filePacket;
    validateFilePacket(buffer, filePacket);
    const Fw::FilePacket::Header& header = filePacket.asHeader();
    ASSERT_EQ(0U, header.sequenceIndex);
    ASSERT_EQ(Fw::FilePacket::T_START, header.type);
  }

  void Tester ::
    validateDataPacket(
        const Fw::Buffer& buffer,
        Fw::FilePacket::DataPacket& dataPacket,
        const U32 sequenceIndex,
        U32& byteOffset
    )
  {
    Fw::FilePacket filePacket;
    validateFilePacket(buffer, filePacket);
    const Fw::FilePacket::Header& header = filePacket.asHeader();
    ASSERT_EQ(sequenceIndex, header.sequenceIndex);
    ASSERT_EQ(Fw::FilePacket::T_DATA, header.type);
    dataPacket = filePacket.asDataPacket();
    ASSERT_EQ(byteOffset, dataPacket.byteOffset);
    byteOffset += dataPacket.dataSize;
  }

  void Tester ::
    validateEndPacket(
        const Fw::Buffer& buffer,
        const U32 sequenceIndex,
        const CFDP::Checksum& checksum
    )
  {
    Fw::FilePacket filePacket;
    validateFilePacket(buffer, filePacket);
    const Fw::FilePacket::Header& header = filePacket.asHeader();
    ASSERT_EQ(sequenceIndex, header.sequenceIndex);
    ASSERT_EQ(Fw::FilePacket::T_END, header.type);
    const Fw::FilePacket::EndPacket endPacket = filePacket.asEndPacket();
    CFDP::Checksum computedChecksum;
    endPacket.getChecksum(computedChecksum);
    ASSERT_EQ(true, checksum == computedChecksum);
  }

  void Tester ::
    validateCancelPacket(
        const Fw::Buffer& buffer,
        const U32 sequenceIndex
    )
  {
    Fw::FilePacket filePacket;
    validateFilePacket(buffer, filePacket);
    const Fw::FilePacket::Header& header = filePacket.asHeader();
    ASSERT_EQ(sequenceIndex, header.sequenceIndex);
    ASSERT_EQ(Fw::FilePacket::T_CANCEL, header.type);
  }

}
