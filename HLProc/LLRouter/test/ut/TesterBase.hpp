// ======================================================================
// \title  LLRouter/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for LLRouter component test harness base class
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

#ifndef LLRouter_TESTER_BASE_HPP
#define LLRouter_TESTER_BASE_HPP

#include <HLProc/LLRouter/LLRouterComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace HLProc {

  //! \class LLRouterTesterBase
  //! \brief Auto-generated base class for LLRouter component test harness
  //!
  class LLRouterTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object LLRouterTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect HLPortsIn to to_HLPortsIn[portNum]
      //!
      void connect_to_HLPortsIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputSerializePort *const HLPortsIn /*!< The port*/
      );

      //! Connect SerReadPort to to_SerReadPort[portNum]
      //!
      void connect_to_SerReadPort(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputSerialReadPort *const SerReadPort /*!< The port*/
      );

      //! Connect Sched to to_Sched[portNum]
      //!
      void connect_to_Sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const Sched /*!< The port*/
      );

      //! Connect CycleDone to to_CycleDone[portNum]
      //!
      void connect_to_CycleDone(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const CycleDone /*!< The port*/
      );

      //! Connect pingIn to to_pingIn[portNum]
      //!
      void connect_to_pingIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputPingPort *const pingIn /*!< The port*/
      );

      //! Connect CmdDisp to to_CmdDisp[portNum]
      //!
      void connect_to_CmdDisp(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputCmdPort *const CmdDisp /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from LLPortsOut
      //!
      //! \return from_LLPortsOut[portNum]
      //!
      Fw::InputSerializePort* get_from_LLPortsOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from SerWritePort
      //!
      //! \return from_SerWritePort[portNum]
      //!
      Drv::InputSerialWritePort* get_from_SerWritePort(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from SerialBufferSend
      //!
      //! \return from_SerialBufferSend[portNum]
      //!
      Fw::InputBufferSendPort* get_from_SerialBufferSend(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from pingOut
      //!
      //! \return from_pingOut[portNum]
      //!
      Svc::InputPingPort* get_from_pingOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from PolyGet
      //!
      //! \return from_PolyGet[portNum]
      //!
      Svc::InputPolyPort* get_from_PolyGet(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from CmdStatus
      //!
      //! \return from_CmdStatus[portNum]
      //!
      Fw::InputCmdResponsePort* get_from_CmdStatus(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from CmdReg
      //!
      //! \return from_CmdReg[portNum]
      //!
      Fw::InputCmdRegPort* get_from_CmdReg(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from Tlm
      //!
      //! \return from_Tlm[portNum]
      //!
      Fw::InputTlmPort* get_from_Tlm(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from Time
      //!
      //! \return from_Time[portNum]
      //!
      Fw::InputTimePort* get_from_Time(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from Log
      //!
      //! \return from_Log[portNum]
      //!
      Fw::InputLogPort* get_from_Log(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

#if FW_ENABLE_TEXT_LOGGING == 1
      //! Get the port that receives input from LogText
      //!
      //! \return from_LogText[portNum]
      //!
      Fw::InputLogTextPort* get_from_LogText(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );
#endif

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object LLRouterTesterBase
      //!
      LLRouterTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object LLRouterTesterBase
      //!
      virtual ~LLRouterTesterBase(void);

      // ----------------------------------------------------------------------
      // Test history
      // ----------------------------------------------------------------------

    protected:

      //! \class History
      //! \brief A history of port inputs
      //!
      template <typename T> class History {

        public:

          //! Create a History
          //!
          History(
              const U32 maxSize /*!< The maximum history size*/
          ) : 
              numEntries(0), 
              maxSize(maxSize) 
          { 
            this->entries = new T[maxSize];
          }

          //! Destroy a History
          //!
          ~History() {
            delete[] this->entries;
          }

          //! Clear the history
          //!
          void clear() { this->numEntries = 0; }

          //! Push an item onto the history
          //!
          void push_back(
              T entry /*!< The item*/
          ) {
            FW_ASSERT(this->numEntries < this->maxSize);
            entries[this->numEntries++] = entry;
          }

          //! Get an item at an index
          //!
          //! \return The item at index i
          //!
          T at(
              const U32 i /*!< The index*/
          ) const {
            FW_ASSERT(i < this->numEntries);
            return entries[i];
          }

          //! Get the number of entries in the history
          //!
          //! \return The number of entries in the history
          //!
          U32 size(void) const { return this->numEntries; }

        private:

          //! The number of entries in the history
          //!
          U32 numEntries;

          //! The maximum history size
          //!
          const U32 maxSize;

          //! The entries
          //!
          T *entries;

      };

      //! Clear all history
      //!
      void clearHistory(void);

    protected:

      // ----------------------------------------------------------------------
      // Handler prototypes for typed from ports
      // ----------------------------------------------------------------------

      //! Handler prototype for from_SerWritePort
      //!
      virtual void from_SerWritePort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      ) = 0;

      //! Handler base function for from_SerWritePort
      //!
      void from_SerWritePort_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      );

      //! Handler prototype for from_SerialBufferSend
      //!
      virtual void from_SerialBufferSend_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer fwBuffer 
      ) = 0;

      //! Handler base function for from_SerialBufferSend
      //!
      void from_SerialBufferSend_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer fwBuffer 
      );

      //! Handler prototype for from_pingOut
      //!
      virtual void from_pingOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      ) = 0;

      //! Handler base function for from_pingOut
      //!
      void from_pingOut_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Handler prototype for from_PolyGet
      //!
      virtual void from_PolyGet_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
      ) = 0;

      //! Handler base function for from_PolyGet
      //!
      void from_PolyGet_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Histories for typed from ports
      // ----------------------------------------------------------------------

      //! Clear from port history
      //!
      void clearFromPortHistory(void);

      //! The total number of from port entries
      //!
      U32 fromPortHistorySize;

      //! Push an entry on the history for from_SerWritePort
      void pushFromPortEntry_SerWritePort(
          Fw::Buffer &serBuffer 
      );

      //! A history entry for from_SerWritePort
      //!
      typedef struct {
        Fw::Buffer serBuffer;
      } FromPortEntry_SerWritePort;

      //! The history for from_SerWritePort
      //!
      History<FromPortEntry_SerWritePort> 
        *fromPortHistory_SerWritePort;

      //! Push an entry on the history for from_SerialBufferSend
      void pushFromPortEntry_SerialBufferSend(
          Fw::Buffer fwBuffer 
      );

      //! A history entry for from_SerialBufferSend
      //!
      typedef struct {
        Fw::Buffer fwBuffer;
      } FromPortEntry_SerialBufferSend;

      //! The history for from_SerialBufferSend
      //!
      History<FromPortEntry_SerialBufferSend> 
        *fromPortHistory_SerialBufferSend;

      //! Push an entry on the history for from_pingOut
      void pushFromPortEntry_pingOut(
          U32 key /*!< Value to return to pinger*/
      );

      //! A history entry for from_pingOut
      //!
      typedef struct {
        U32 key;
      } FromPortEntry_pingOut;

      //! The history for from_pingOut
      //!
      History<FromPortEntry_pingOut> 
        *fromPortHistory_pingOut;

      //! Push an entry on the history for from_PolyGet
      void pushFromPortEntry_PolyGet(
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
      );

      //! A history entry for from_PolyGet
      //!
      typedef struct {
        U32 entry;
        Svc::MeasurementStatus status;
        Fw::Time time;
        Fw::PolyType val;
      } FromPortEntry_PolyGet;

      //! The history for from_PolyGet
      //!
      History<FromPortEntry_PolyGet> 
        *fromPortHistory_PolyGet;

    protected:

      // ----------------------------------------------------------------------
      // Handler prototypes for serial from ports
      // ----------------------------------------------------------------------

      //! Handler prototype for from_LLPortsOut
      //!
      virtual void from_LLPortsOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      ) = 0;

      //! Handler base class function prototype for from_LLPortsOut
      //!
      void from_LLPortsOut_handlerBase (
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );


    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to HLPortsIn
      //!
      void invoke_to_HLPortsIn(
          NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer
      );

      //! Invoke the to port connected to SerReadPort
      //!
      void invoke_to_SerReadPort(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

      //! Invoke the to port connected to Sched
      //!
      void invoke_to_Sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Invoke the to port connected to CycleDone
      //!
      void invoke_to_CycleDone(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Invoke the to port connected to pingIn
      //!
      void invoke_to_pingIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_HLPortsIn ports
      //!
      //! \return The number of to_HLPortsIn ports
      //!
      NATIVE_INT_TYPE getNum_to_HLPortsIn(void) const;

      //! Get the number of from_LLPortsOut ports
      //!
      //! \return The number of from_LLPortsOut ports
      //!
      NATIVE_INT_TYPE getNum_from_LLPortsOut(void) const;

      //! Get the number of to_SerReadPort ports
      //!
      //! \return The number of to_SerReadPort ports
      //!
      NATIVE_INT_TYPE getNum_to_SerReadPort(void) const;

      //! Get the number of from_SerWritePort ports
      //!
      //! \return The number of from_SerWritePort ports
      //!
      NATIVE_INT_TYPE getNum_from_SerWritePort(void) const;

      //! Get the number of to_Sched ports
      //!
      //! \return The number of to_Sched ports
      //!
      NATIVE_INT_TYPE getNum_to_Sched(void) const;

      //! Get the number of to_CycleDone ports
      //!
      //! \return The number of to_CycleDone ports
      //!
      NATIVE_INT_TYPE getNum_to_CycleDone(void) const;

      //! Get the number of from_SerialBufferSend ports
      //!
      //! \return The number of from_SerialBufferSend ports
      //!
      NATIVE_INT_TYPE getNum_from_SerialBufferSend(void) const;

      //! Get the number of to_pingIn ports
      //!
      //! \return The number of to_pingIn ports
      //!
      NATIVE_INT_TYPE getNum_to_pingIn(void) const;

      //! Get the number of from_pingOut ports
      //!
      //! \return The number of from_pingOut ports
      //!
      NATIVE_INT_TYPE getNum_from_pingOut(void) const;

      //! Get the number of from_PolyGet ports
      //!
      //! \return The number of from_PolyGet ports
      //!
      NATIVE_INT_TYPE getNum_from_PolyGet(void) const;

      //! Get the number of to_CmdDisp ports
      //!
      //! \return The number of to_CmdDisp ports
      //!
      NATIVE_INT_TYPE getNum_to_CmdDisp(void) const;

      //! Get the number of from_CmdStatus ports
      //!
      //! \return The number of from_CmdStatus ports
      //!
      NATIVE_INT_TYPE getNum_from_CmdStatus(void) const;

      //! Get the number of from_CmdReg ports
      //!
      //! \return The number of from_CmdReg ports
      //!
      NATIVE_INT_TYPE getNum_from_CmdReg(void) const;

      //! Get the number of from_Tlm ports
      //!
      //! \return The number of from_Tlm ports
      //!
      NATIVE_INT_TYPE getNum_from_Tlm(void) const;

      //! Get the number of from_Time ports
      //!
      //! \return The number of from_Time ports
      //!
      NATIVE_INT_TYPE getNum_from_Time(void) const;

      //! Get the number of from_Log ports
      //!
      //! \return The number of from_Log ports
      //!
      NATIVE_INT_TYPE getNum_from_Log(void) const;

#if FW_ENABLE_TEXT_LOGGING == 1
      //! Get the number of from_LogText ports
      //!
      //! \return The number of from_LogText ports
      //!
      NATIVE_INT_TYPE getNum_from_LogText(void) const;
#endif

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_HLPortsIn[portNum] is connected
      //!
      bool isConnected_to_HLPortsIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_SerReadPort[portNum] is connected
      //!
      bool isConnected_to_SerReadPort(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_Sched[portNum] is connected
      //!
      bool isConnected_to_Sched(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_CycleDone[portNum] is connected
      //!
      bool isConnected_to_CycleDone(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_pingIn[portNum] is connected
      //!
      bool isConnected_to_pingIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_CmdDisp[portNum] is connected
      //!
      bool isConnected_to_CmdDisp(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      // ----------------------------------------------------------------------
      // Functions for sending commands
      // ----------------------------------------------------------------------

    protected:
    
      // send command buffers directly - used for intentional command encoding errors
      void sendRawCmd(FwOpcodeType opcode, U32 cmdSeq, Fw::CmdArgBuffer& args); 

      //! Send a LLR_SET_PATCH_MODE command
      //!
      void sendCmd_LLR_SET_PATCH_MODE(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq, /*!< The command sequence number*/
          LLRouterComponentBase::LLRPatchMode Mode 
      );

      //! Send a LLR_LOAD_PATCH_FILE command
      //!
      void sendCmd_LLR_LOAD_PATCH_FILE(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq, /*!< The command sequence number*/
          const Fw::CmdStringArg& file, /*!< Patch file*/
          LLRouterComponentBase::LL updatingLL, /*!< The LL to update*/
          bool legacyBootloader, /*!< Updating LLs with an old bootloader*/
          bool forceboot, /*!< Tell the bootloader to ignore CRC32 checks on startup*/
          U64 disabled_selftests /*!< Bitmask of tests to disable on startup. A 1 indicates that the test should not be run*/
      );

      //! Send a LLR_CYCLE_SERIAL command
      //!
      void sendCmd_LLR_CYCLE_SERIAL(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq, /*!< The command sequence number*/
          bool on /*!< Turn the command sending on or off.*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Command response handling
      // ----------------------------------------------------------------------

      //! Handle a command response
      //!
      virtual void cmdResponseIn(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          const Fw::CommandResponse response /*!< The command response*/
      );

      //! A type representing a command response
      //!
      typedef struct {
        FwOpcodeType opCode;
        U32 cmdSeq;
        Fw::CommandResponse response;
      } CmdResponse;

      //! The command response history
      //!
      History<CmdResponse> *cmdResponseHistory;

    protected:

      // ----------------------------------------------------------------------
      // Event dispatch
      // ----------------------------------------------------------------------

      //! Dispatch an event
      //!
      void dispatchEvents(
          const FwEventIdType id, /*!< The event ID*/
          Fw::Time& timeTag, /*!< The time*/
          const Fw::LogSeverity severity, /*!< The severity*/
          Fw::LogBuffer& args /*!< The serialized arguments*/
      );

      //! Clear event history
      //!
      void clearEvents(void);

      //! The total number of events seen
      //!
      U32 eventsSize;

#if FW_ENABLE_TEXT_LOGGING

    protected:

      // ----------------------------------------------------------------------
      // Text events
      // ----------------------------------------------------------------------

      //! Handle a text event
      //!
      virtual void textLogIn(
          const FwEventIdType id, /*!< The event ID*/
          Fw::Time& timeTag, /*!< The time*/
          const Fw::TextLogSeverity severity, /*!< The severity*/
          const Fw::TextLogString& text /*!< The event string*/
      );

      //! A history entry for the text log
      //!
      typedef struct {
        U32 id;
        Fw::Time timeTag;
        Fw::TextLogSeverity severity;
        Fw::TextLogString text;
      } TextLogEntry;

      //! The history of text log events
      //!
      History<TextLogEntry> *textLogHistory;

      //! Print a text log history entry
      //!
      static void printTextLogHistoryEntry(
          const TextLogEntry& e,
          FILE* file
      );

      //! Print the text log history
      //!
      void printTextLogHistory(FILE *const file);

#endif

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_DataReceiveError
      // ----------------------------------------------------------------------

      //! Handle event LLR_DataReceiveError
      //!
      virtual void logIn_WARNING_HI_LLR_DataReceiveError(
          LLRouterComponentBase::LLR_ReceiveError error /*!< The receive eror code*/
      );

      //! A history entry for event LLR_DataReceiveError
      //!
      typedef struct {
        LLRouterComponentBase::LLR_ReceiveError error;
      } EventEntry_LLR_DataReceiveError;

      //! The history of LLR_DataReceiveError events
      //!
      History<EventEntry_LLR_DataReceiveError> 
        *eventHistory_LLR_DataReceiveError;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_DataCrcError
      // ----------------------------------------------------------------------

      //! Handle event LLR_DataCrcError
      //!
      virtual void logIn_WARNING_HI_LLR_DataCrcError(
          U16 expected, 
          U16 actual 
      );

      //! A history entry for event LLR_DataCrcError
      //!
      typedef struct {
        U16 expected;
        U16 actual;
      } EventEntry_LLR_DataCrcError;

      //! The history of LLR_DataCrcError events
      //!
      History<EventEntry_LLR_DataCrcError> 
        *eventHistory_LLR_DataCrcError;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_InvalidPortNum
      // ----------------------------------------------------------------------

      //! Handle event LLR_InvalidPortNum
      //!
      virtual void logIn_WARNING_HI_LLR_InvalidPortNum(
          U8 portNum 
      );

      //! A history entry for event LLR_InvalidPortNum
      //!
      typedef struct {
        U8 portNum;
      } EventEntry_LLR_InvalidPortNum;

      //! The history of LLR_InvalidPortNum events
      //!
      History<EventEntry_LLR_InvalidPortNum> 
        *eventHistory_LLR_InvalidPortNum;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_BadSerialPortCall
      // ----------------------------------------------------------------------

      //! Handle event LLR_BadSerialPortCall
      //!
      virtual void logIn_WARNING_HI_LLR_BadSerialPortCall(
          I32 status, 
          U8 portNum 
      );

      //! A history entry for event LLR_BadSerialPortCall
      //!
      typedef struct {
        I32 status;
        U8 portNum;
      } EventEntry_LLR_BadSerialPortCall;

      //! The history of LLR_BadSerialPortCall events
      //!
      History<EventEntry_LLR_BadSerialPortCall> 
        *eventHistory_LLR_BadSerialPortCall;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchFileOpen
      // ----------------------------------------------------------------------

      //! Handle event LLR_PatchFileOpen
      //!
      virtual void logIn_ACTIVITY_HI_LLR_PatchFileOpen(
          Fw::LogStringArg& filename, 
          U32 filesize 
      );

      //! A history entry for event LLR_PatchFileOpen
      //!
      typedef struct {
        Fw::LogStringArg filename;
        U32 filesize;
      } EventEntry_LLR_PatchFileOpen;

      //! The history of LLR_PatchFileOpen events
      //!
      History<EventEntry_LLR_PatchFileOpen> 
        *eventHistory_LLR_PatchFileOpen;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchModeChange
      // ----------------------------------------------------------------------

      //! Handle event LLR_PatchModeChange
      //!
      virtual void logIn_ACTIVITY_HI_LLR_PatchModeChange(
          Fw::LogStringArg& newmode 
      );

      //! A history entry for event LLR_PatchModeChange
      //!
      typedef struct {
        Fw::LogStringArg newmode;
      } EventEntry_LLR_PatchModeChange;

      //! The history of LLR_PatchModeChange events
      //!
      History<EventEntry_LLR_PatchModeChange> 
        *eventHistory_LLR_PatchModeChange;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_FirstACK
      // ----------------------------------------------------------------------

      //! Handle event LLR_FirstACK
      //!
      virtual void logIn_ACTIVITY_HI_LLR_FirstACK(
          void
      );

      //! Size of history for event LLR_FirstACK
      //!
      U32 eventsSize_LLR_FirstACK;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchEvent
      // ----------------------------------------------------------------------

      //! Handle event LLR_PatchEvent
      //!
      virtual void logIn_ACTIVITY_HI_LLR_PatchEvent(
          U32 progress 
      );

      //! A history entry for event LLR_PatchEvent
      //!
      typedef struct {
        U32 progress;
      } EventEntry_LLR_PatchEvent;

      //! The history of LLR_PatchEvent events
      //!
      History<EventEntry_LLR_PatchEvent> 
        *eventHistory_LLR_PatchEvent;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchComplete
      // ----------------------------------------------------------------------

      //! Handle event LLR_PatchComplete
      //!
      virtual void logIn_ACTIVITY_HI_LLR_PatchComplete(
          void
      );

      //! Size of history for event LLR_PatchComplete
      //!
      U32 eventsSize_LLR_PatchComplete;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PatchError
      // ----------------------------------------------------------------------

      //! Handle event LLR_PatchError
      //!
      virtual void logIn_WARNING_HI_LLR_PatchError(
          LLRouterComponentBase::LLR_PatchError error, 
          I32 err_num, 
          Fw::LogStringArg& err_string 
      );

      //! A history entry for event LLR_PatchError
      //!
      typedef struct {
        LLRouterComponentBase::LLR_PatchError error;
        I32 err_num;
        Fw::LogStringArg err_string;
      } EventEntry_LLR_PatchError;

      //! The history of LLR_PatchError events
      //!
      History<EventEntry_LLR_PatchError> 
        *eventHistory_LLR_PatchError;

    protected:

      // ----------------------------------------------------------------------
      // Event: LLR_PolyError
      // ----------------------------------------------------------------------

      //! Handle event LLR_PolyError
      //!
      virtual void logIn_WARNING_HI_LLR_PolyError(
          void
      );

      //! Size of history for event LLR_PolyError
      //!
      U32 eventsSize_LLR_PolyError;

    protected:

      // ----------------------------------------------------------------------
      // Telemetry dispatch
      // ----------------------------------------------------------------------

      //! Dispatch telemetry
      //!
      void dispatchTlm(
          const FwChanIdType id, /*!< The channel ID*/
          const Fw::Time& timeTag, /*!< The time*/
          Fw::TlmBuffer& val /*!< The channel value*/
      );

      //! Clear telemetry history
      //!
      void clearTlm(void);

      //! The total number of telemetry inputs seen
      //!
      U32 tlmSize;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_CrcErrors
      // ----------------------------------------------------------------------

      //! Handle channel LLR_CrcErrors
      //!
      virtual void tlmInput_LLR_CrcErrors(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_CrcErrors
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_CrcErrors;

      //! The history of LLR_CrcErrors values
      //!
      History<TlmEntry_LLR_CrcErrors> 
        *tlmHistory_LLR_CrcErrors;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_ResyncErrors
      // ----------------------------------------------------------------------

      //! Handle channel LLR_ResyncErrors
      //!
      virtual void tlmInput_LLR_ResyncErrors(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_ResyncErrors
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_ResyncErrors;

      //! The history of LLR_ResyncErrors values
      //!
      History<TlmEntry_LLR_ResyncErrors> 
        *tlmHistory_LLR_ResyncErrors;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumPackets
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumPackets
      //!
      virtual void tlmInput_LLR_NumPackets(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumPackets
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumPackets;

      //! The history of LLR_NumPackets values
      //!
      History<TlmEntry_LLR_NumPackets> 
        *tlmHistory_LLR_NumPackets;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumSerialErrors
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumSerialErrors
      //!
      virtual void tlmInput_LLR_NumSerialErrors(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumSerialErrors
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumSerialErrors;

      //! The history of LLR_NumSerialErrors values
      //!
      History<TlmEntry_LLR_NumSerialErrors> 
        *tlmHistory_LLR_NumSerialErrors;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumGoodPackets
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumGoodPackets
      //!
      virtual void tlmInput_LLR_NumGoodPackets(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumGoodPackets
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumGoodPackets;

      //! The history of LLR_NumGoodPackets values
      //!
      History<TlmEntry_LLR_NumGoodPackets> 
        *tlmHistory_LLR_NumGoodPackets;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumDecodeErrors
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumDecodeErrors
      //!
      virtual void tlmInput_LLR_NumDecodeErrors(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumDecodeErrors
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumDecodeErrors;

      //! The history of LLR_NumDecodeErrors values
      //!
      History<TlmEntry_LLR_NumDecodeErrors> 
        *tlmHistory_LLR_NumDecodeErrors;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumInvalidPorts
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumInvalidPorts
      //!
      virtual void tlmInput_LLR_NumInvalidPorts(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumInvalidPorts
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumInvalidPorts;

      //! The history of LLR_NumInvalidPorts values
      //!
      History<TlmEntry_LLR_NumInvalidPorts> 
        *tlmHistory_LLR_NumInvalidPorts;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumBadSerialPortCalls
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumBadSerialPortCalls
      //!
      virtual void tlmInput_LLR_NumBadSerialPortCalls(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumBadSerialPortCalls
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumBadSerialPortCalls;

      //! The history of LLR_NumBadSerialPortCalls values
      //!
      History<TlmEntry_LLR_NumBadSerialPortCalls> 
        *tlmHistory_LLR_NumBadSerialPortCalls;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumOuputBufferOverflows
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumOuputBufferOverflows
      //!
      virtual void tlmInput_LLR_NumOuputBufferOverflows(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumOuputBufferOverflows
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumOuputBufferOverflows;

      //! The history of LLR_NumOuputBufferOverflows values
      //!
      History<TlmEntry_LLR_NumOuputBufferOverflows> 
        *tlmHistory_LLR_NumOuputBufferOverflows;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_NumZeroPktSize
      // ----------------------------------------------------------------------

      //! Handle channel LLR_NumZeroPktSize
      //!
      virtual void tlmInput_LLR_NumZeroPktSize(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_NumZeroPktSize
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_NumZeroPktSize;

      //! The history of LLR_NumZeroPktSize values
      //!
      History<TlmEntry_LLR_NumZeroPktSize> 
        *tlmHistory_LLR_NumZeroPktSize;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_MaxCycleTime
      // ----------------------------------------------------------------------

      //! Handle channel LLR_MaxCycleTime
      //!
      virtual void tlmInput_LLR_MaxCycleTime(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_MaxCycleTime
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_MaxCycleTime;

      //! The history of LLR_MaxCycleTime values
      //!
      History<TlmEntry_LLR_MaxCycleTime> 
        *tlmHistory_LLR_MaxCycleTime;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_CycleTime
      // ----------------------------------------------------------------------

      //! Handle channel LLR_CycleTime
      //!
      virtual void tlmInput_LLR_CycleTime(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_CycleTime
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_CycleTime;

      //! The history of LLR_CycleTime values
      //!
      History<TlmEntry_LLR_CycleTime> 
        *tlmHistory_LLR_CycleTime;

    protected:

      // ----------------------------------------------------------------------
      // Channel: LLR_PatchProgress
      // ----------------------------------------------------------------------

      //! Handle channel LLR_PatchProgress
      //!
      virtual void tlmInput_LLR_PatchProgress(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLR_PatchProgress
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_LLR_PatchProgress;

      //! The history of LLR_PatchProgress values
      //!
      History<TlmEntry_LLR_PatchProgress> 
        *tlmHistory_LLR_PatchProgress;

    protected:

      // ----------------------------------------------------------------------
      // Test time
      // ----------------------------------------------------------------------

      //! Set the test time for events and telemetry
      //!
      void setTestTime(
          const Fw::Time& timeTag /*!< The time*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to HLPortsIn
      //!
      Fw::OutputSerializePort m_to_HLPortsIn[25];

      //! To port connected to SerReadPort
      //!
      Drv::OutputSerialReadPort m_to_SerReadPort[1];

      //! To port connected to Sched
      //!
      Svc::OutputSchedPort m_to_Sched[1];

      //! To port connected to CycleDone
      //!
      Svc::OutputSchedPort m_to_CycleDone[1];

      //! To port connected to pingIn
      //!
      Svc::OutputPingPort m_to_pingIn[1];

      //! To port connected to CmdDisp
      //!
      Fw::OutputCmdPort m_to_CmdDisp[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to LLPortsOut
      //!
      Fw::InputSerializePort m_from_LLPortsOut[25];

      //! From port connected to SerWritePort
      //!
      Drv::InputSerialWritePort m_from_SerWritePort[1];

      //! From port connected to SerialBufferSend
      //!
      Fw::InputBufferSendPort m_from_SerialBufferSend[1];

      //! From port connected to pingOut
      //!
      Svc::InputPingPort m_from_pingOut[1];

      //! From port connected to PolyGet
      //!
      Svc::InputPolyPort m_from_PolyGet[1];

      //! From port connected to CmdStatus
      //!
      Fw::InputCmdResponsePort m_from_CmdStatus[1];

      //! From port connected to CmdReg
      //!
      Fw::InputCmdRegPort m_from_CmdReg[1];

      //! From port connected to Tlm
      //!
      Fw::InputTlmPort m_from_Tlm[1];

      //! From port connected to Time
      //!
      Fw::InputTimePort m_from_Time[1];

      //! From port connected to Log
      //!
      Fw::InputLogPort m_from_Log[1];

#if FW_ENABLE_TEXT_LOGGING == 1
      //! From port connected to LogText
      //!
      Fw::InputLogTextPort m_from_LogText[1];
#endif

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_LLPortsOut
      //!
      static void from_LLPortsOut_static(
          Fw::PassiveComponentBase *const callComp, //!< The component instance
          const NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer //!< serialized data buffer
      );

      //! Static function for port from_SerWritePort
      //!
      static void from_SerWritePort_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      );

      //! Static function for port from_SerialBufferSend
      //!
      static void from_SerialBufferSend_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer fwBuffer 
      );

      //! Static function for port from_pingOut
      //!
      static void from_pingOut_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Static function for port from_PolyGet
      //!
      static void from_PolyGet_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
      );

      //! Static function for port from_CmdStatus
      //!
      static void from_CmdStatus_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwOpcodeType opCode, /*!< Command Op Code*/
          U32 cmdSeq, /*!< Command Sequence*/
          Fw::CommandResponse response /*!< The command response argument*/
      );

      //! Static function for port from_CmdReg
      //!
      static void from_CmdReg_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwOpcodeType opCode /*!< Command Op Code*/
      );

      //! Static function for port from_Tlm
      //!
      static void from_Tlm_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwChanIdType id, /*!< Telemetry Channel ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TlmBuffer &val /*!< Buffer containing serialized telemetry value*/
      );

      //! Static function for port from_Time
      //!
      static void from_Time_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      );

      //! Static function for port from_Log
      //!
      static void from_Log_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::LogSeverity severity, /*!< The severity argument*/
          Fw::LogBuffer &args /*!< Buffer containing serialized log entry*/
      );

#if FW_ENABLE_TEXT_LOGGING == 1
      //! Static function for port from_LogText
      //!
      static void from_LogText_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TextLogSeverity severity, /*!< The severity argument*/
          Fw::TextLogString &text /*!< Text of log message*/
      );
#endif

    private:

      // ----------------------------------------------------------------------
      // Test time
      // ----------------------------------------------------------------------

      //! Test time stamp
      //!
      Fw::Time m_testTime;

  };

} // end namespace HLProc

#endif
