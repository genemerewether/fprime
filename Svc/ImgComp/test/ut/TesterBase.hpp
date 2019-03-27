// ======================================================================
// \title  ImgComp/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for ImgComp component test harness base class
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

#ifndef ImgComp_TESTER_BASE_HPP
#define ImgComp_TESTER_BASE_HPP

#include <Svc/ImgComp/ImgCompComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Svc {

  //! \class ImgCompTesterBase
  //! \brief Auto-generated base class for ImgComp component test harness
  //!
  class ImgCompTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object ImgCompTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect uncompressedIn to to_uncompressedIn[portNum]
      //!
      void connect_to_uncompressedIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputBufferSendPort *const uncompressedIn /*!< The port*/
      );

      //! Connect pingIn to to_pingIn[portNum]
      //!
      void connect_to_pingIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputPingPort *const pingIn /*!< The port*/
      );

      //! Connect schedIn to to_schedIn[portNum]
      //!
      void connect_to_schedIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const schedIn /*!< The port*/
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

      //! Get the port that receives input from compressedOutStorage
      //!
      //! \return from_compressedOutStorage[portNum]
      //!
      Fw::InputBufferSendPort* get_from_compressedOutStorage(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from compressedOutXmit
      //!
      //! \return from_compressedOutXmit[portNum]
      //!
      Fw::InputBufferSendPort* get_from_compressedOutXmit(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from compressedGetStorage
      //!
      //! \return from_compressedGetStorage[portNum]
      //!
      Fw::InputBufferGetPort* get_from_compressedGetStorage(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from compressedGetXmit
      //!
      //! \return from_compressedGetXmit[portNum]
      //!
      Fw::InputBufferGetPort* get_from_compressedGetXmit(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from uncompressedReturn
      //!
      //! \return from_uncompressedReturn[portNum]
      //!
      Fw::InputBufferSendPort* get_from_uncompressedReturn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from timeCaller
      //!
      //! \return from_timeCaller[portNum]
      //!
      Fw::InputTimePort* get_from_timeCaller(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from pingOut
      //!
      //! \return from_pingOut[portNum]
      //!
      Svc::InputPingPort* get_from_pingOut(
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

      //! Construct object ImgCompTesterBase
      //!
      ImgCompTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object ImgCompTesterBase
      //!
      virtual ~ImgCompTesterBase(void);

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

      //! Handler prototype for from_compressedOutStorage
      //!
      virtual void from_compressedOutStorage_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      ) = 0;

      //! Handler base function for from_compressedOutStorage
      //!
      void from_compressedOutStorage_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Handler prototype for from_compressedOutXmit
      //!
      virtual void from_compressedOutXmit_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      ) = 0;

      //! Handler base function for from_compressedOutXmit
      //!
      void from_compressedOutXmit_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Handler prototype for from_compressedGetStorage
      //!
      virtual Fw::Buffer from_compressedGetStorage_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      ) = 0;

      //! Handler base function for from_compressedGetStorage
      //!
      Fw::Buffer from_compressedGetStorage_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      );

      //! Handler prototype for from_compressedGetXmit
      //!
      virtual Fw::Buffer from_compressedGetXmit_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      ) = 0;

      //! Handler base function for from_compressedGetXmit
      //!
      Fw::Buffer from_compressedGetXmit_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      );

      //! Handler prototype for from_uncompressedReturn
      //!
      virtual void from_uncompressedReturn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      ) = 0;

      //! Handler base function for from_uncompressedReturn
      //!
      void from_uncompressedReturn_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
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

      //! Push an entry on the history for from_compressedOutStorage
      void pushFromPortEntry_compressedOutStorage(
          Fw::Buffer &fwBuffer 
      );

      //! A history entry for from_compressedOutStorage
      //!
      typedef struct {
        Fw::Buffer fwBuffer;
      } FromPortEntry_compressedOutStorage;

      //! The history for from_compressedOutStorage
      //!
      History<FromPortEntry_compressedOutStorage> 
        *fromPortHistory_compressedOutStorage;

      //! Push an entry on the history for from_compressedOutXmit
      void pushFromPortEntry_compressedOutXmit(
          Fw::Buffer &fwBuffer 
      );

      //! A history entry for from_compressedOutXmit
      //!
      typedef struct {
        Fw::Buffer fwBuffer;
      } FromPortEntry_compressedOutXmit;

      //! The history for from_compressedOutXmit
      //!
      History<FromPortEntry_compressedOutXmit> 
        *fromPortHistory_compressedOutXmit;

      //! Push an entry on the history for from_compressedGetStorage
      void pushFromPortEntry_compressedGetStorage(
          U32 size 
      );

      //! A history entry for from_compressedGetStorage
      //!
      typedef struct {
        U32 size;
      } FromPortEntry_compressedGetStorage;

      //! The history for from_compressedGetStorage
      //!
      History<FromPortEntry_compressedGetStorage> 
        *fromPortHistory_compressedGetStorage;

      //! Push an entry on the history for from_compressedGetXmit
      void pushFromPortEntry_compressedGetXmit(
          U32 size 
      );

      //! A history entry for from_compressedGetXmit
      //!
      typedef struct {
        U32 size;
      } FromPortEntry_compressedGetXmit;

      //! The history for from_compressedGetXmit
      //!
      History<FromPortEntry_compressedGetXmit> 
        *fromPortHistory_compressedGetXmit;

      //! Push an entry on the history for from_uncompressedReturn
      void pushFromPortEntry_uncompressedReturn(
          Fw::Buffer &fwBuffer 
      );

      //! A history entry for from_uncompressedReturn
      //!
      typedef struct {
        Fw::Buffer fwBuffer;
      } FromPortEntry_uncompressedReturn;

      //! The history for from_uncompressedReturn
      //!
      History<FromPortEntry_uncompressedReturn> 
        *fromPortHistory_uncompressedReturn;

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

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to uncompressedIn
      //!
      void invoke_to_uncompressedIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Invoke the to port connected to pingIn
      //!
      void invoke_to_pingIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Invoke the to port connected to schedIn
      //!
      void invoke_to_schedIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of from_compressedOutStorage ports
      //!
      //! \return The number of from_compressedOutStorage ports
      //!
      NATIVE_INT_TYPE getNum_from_compressedOutStorage(void) const;

      //! Get the number of from_compressedOutXmit ports
      //!
      //! \return The number of from_compressedOutXmit ports
      //!
      NATIVE_INT_TYPE getNum_from_compressedOutXmit(void) const;

      //! Get the number of from_compressedGetStorage ports
      //!
      //! \return The number of from_compressedGetStorage ports
      //!
      NATIVE_INT_TYPE getNum_from_compressedGetStorage(void) const;

      //! Get the number of from_compressedGetXmit ports
      //!
      //! \return The number of from_compressedGetXmit ports
      //!
      NATIVE_INT_TYPE getNum_from_compressedGetXmit(void) const;

      //! Get the number of to_uncompressedIn ports
      //!
      //! \return The number of to_uncompressedIn ports
      //!
      NATIVE_INT_TYPE getNum_to_uncompressedIn(void) const;

      //! Get the number of from_uncompressedReturn ports
      //!
      //! \return The number of from_uncompressedReturn ports
      //!
      NATIVE_INT_TYPE getNum_from_uncompressedReturn(void) const;

      //! Get the number of to_pingIn ports
      //!
      //! \return The number of to_pingIn ports
      //!
      NATIVE_INT_TYPE getNum_to_pingIn(void) const;

      //! Get the number of from_timeCaller ports
      //!
      //! \return The number of from_timeCaller ports
      //!
      NATIVE_INT_TYPE getNum_from_timeCaller(void) const;

      //! Get the number of from_pingOut ports
      //!
      //! \return The number of from_pingOut ports
      //!
      NATIVE_INT_TYPE getNum_from_pingOut(void) const;

      //! Get the number of to_schedIn ports
      //!
      //! \return The number of to_schedIn ports
      //!
      NATIVE_INT_TYPE getNum_to_schedIn(void) const;

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
      //! Whether to_uncompressedIn[portNum] is connected
      //!
      bool isConnected_to_uncompressedIn(
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
      //! Whether to_schedIn[portNum] is connected
      //!
      bool isConnected_to_schedIn(
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

      //! Send a IMGCOMP_NoOp command
      //!
      void sendCmd_IMGCOMP_NoOp(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq /*!< The command sequence number*/
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
      // Event: IMGCOMP_SoftCompError
      // ----------------------------------------------------------------------

      //! Handle event IMGCOMP_SoftCompError
      //!
      virtual void logIn_WARNING_HI_IMGCOMP_SoftCompError(
          ImgCompComponentBase::SoftCompErrorType error, /*!< The error type*/
          Fw::LogStringArg& msg 
      );

      //! A history entry for event IMGCOMP_SoftCompError
      //!
      typedef struct {
        ImgCompComponentBase::SoftCompErrorType error;
        Fw::LogStringArg msg;
      } EventEntry_IMGCOMP_SoftCompError;

      //! The history of IMGCOMP_SoftCompError events
      //!
      History<EventEntry_IMGCOMP_SoftCompError> 
        *eventHistory_IMGCOMP_SoftCompError;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_BadBuffer
      // ----------------------------------------------------------------------

      //! Handle event IMGCOMP_BadBuffer
      //!
      virtual void logIn_WARNING_HI_IMGCOMP_BadBuffer(
          void
      );

      //! Size of history for event IMGCOMP_BadBuffer
      //!
      U32 eventsSize_IMGCOMP_BadBuffer;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_BadSetting
      // ----------------------------------------------------------------------

      //! Handle event IMGCOMP_BadSetting
      //!
      virtual void logIn_WARNING_HI_IMGCOMP_BadSetting(
          U32 portNum 
      );

      //! A history entry for event IMGCOMP_BadSetting
      //!
      typedef struct {
        U32 portNum;
      } EventEntry_IMGCOMP_BadSetting;

      //! The history of IMGCOMP_BadSetting events
      //!
      History<EventEntry_IMGCOMP_BadSetting> 
        *eventHistory_IMGCOMP_BadSetting;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_NoBuffer
      // ----------------------------------------------------------------------

      //! Handle event IMGCOMP_NoBuffer
      //!
      virtual void logIn_WARNING_HI_IMGCOMP_NoBuffer(
          U32 size 
      );

      //! A history entry for event IMGCOMP_NoBuffer
      //!
      typedef struct {
        U32 size;
      } EventEntry_IMGCOMP_NoBuffer;

      //! The history of IMGCOMP_NoBuffer events
      //!
      History<EventEntry_IMGCOMP_NoBuffer> 
        *eventHistory_IMGCOMP_NoBuffer;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_BufferOffset
      // ----------------------------------------------------------------------

      //! Handle event IMGCOMP_BufferOffset
      //!
      virtual void logIn_WARNING_HI_IMGCOMP_BufferOffset(
          ImgCompComponentBase::BufferOffsetSkipType type, /*!< The error type*/
          ImgCompComponentBase::BufferOffsetSkipOutput output, /*!< The error type*/
          U32 inBuffer, 
          U32 portNum 
      );

      //! A history entry for event IMGCOMP_BufferOffset
      //!
      typedef struct {
        ImgCompComponentBase::BufferOffsetSkipType type;
        ImgCompComponentBase::BufferOffsetSkipOutput output;
        U32 inBuffer;
        U32 portNum;
      } EventEntry_IMGCOMP_BufferOffset;

      //! The history of IMGCOMP_BufferOffset events
      //!
      History<EventEntry_IMGCOMP_BufferOffset> 
        *eventHistory_IMGCOMP_BufferOffset;

    protected:

      // ----------------------------------------------------------------------
      // Event: IMGCOMP_NoRestartMarkers
      // ----------------------------------------------------------------------

      //! Handle event IMGCOMP_NoRestartMarkers
      //!
      virtual void logIn_WARNING_HI_IMGCOMP_NoRestartMarkers(
          I32 error 
      );

      //! A history entry for event IMGCOMP_NoRestartMarkers
      //!
      typedef struct {
        I32 error;
      } EventEntry_IMGCOMP_NoRestartMarkers;

      //! The history of IMGCOMP_NoRestartMarkers events
      //!
      History<EventEntry_IMGCOMP_NoRestartMarkers> 
        *eventHistory_IMGCOMP_NoRestartMarkers;

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
      // Channel: IMGCOMP_BuffersHandled
      // ----------------------------------------------------------------------

      //! Handle channel IMGCOMP_BuffersHandled
      //!
      virtual void tlmInput_IMGCOMP_BuffersHandled(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel IMGCOMP_BuffersHandled
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_IMGCOMP_BuffersHandled;

      //! The history of IMGCOMP_BuffersHandled values
      //!
      History<TlmEntry_IMGCOMP_BuffersHandled> 
        *tlmHistory_IMGCOMP_BuffersHandled;

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

      //! To port connected to uncompressedIn
      //!
      Fw::OutputBufferSendPort m_to_uncompressedIn[2];

      //! To port connected to pingIn
      //!
      Svc::OutputPingPort m_to_pingIn[1];

      //! To port connected to schedIn
      //!
      Svc::OutputSchedPort m_to_schedIn[1];

      //! To port connected to CmdDisp
      //!
      Fw::OutputCmdPort m_to_CmdDisp[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to compressedOutStorage
      //!
      Fw::InputBufferSendPort m_from_compressedOutStorage[2];

      //! From port connected to compressedOutXmit
      //!
      Fw::InputBufferSendPort m_from_compressedOutXmit[2];

      //! From port connected to compressedGetStorage
      //!
      Fw::InputBufferGetPort m_from_compressedGetStorage[2];

      //! From port connected to compressedGetXmit
      //!
      Fw::InputBufferGetPort m_from_compressedGetXmit[2];

      //! From port connected to uncompressedReturn
      //!
      Fw::InputBufferSendPort m_from_uncompressedReturn[2];

      //! From port connected to timeCaller
      //!
      Fw::InputTimePort m_from_timeCaller[1];

      //! From port connected to pingOut
      //!
      Svc::InputPingPort m_from_pingOut[1];

      //! From port connected to CmdStatus
      //!
      Fw::InputCmdResponsePort m_from_CmdStatus[1];

      //! From port connected to CmdReg
      //!
      Fw::InputCmdRegPort m_from_CmdReg[1];

      //! From port connected to Tlm
      //!
      Fw::InputTlmPort m_from_Tlm[1];

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

      //! Static function for port from_compressedOutStorage
      //!
      static void from_compressedOutStorage_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Static function for port from_compressedOutXmit
      //!
      static void from_compressedOutXmit_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Static function for port from_compressedGetStorage
      //!
      static Fw::Buffer from_compressedGetStorage_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      );

      //! Static function for port from_compressedGetXmit
      //!
      static Fw::Buffer from_compressedGetXmit_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 size 
      );

      //! Static function for port from_uncompressedReturn
      //!
      static void from_uncompressedReturn_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &fwBuffer 
      );

      //! Static function for port from_timeCaller
      //!
      static void from_timeCaller_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      );

      //! Static function for port from_pingOut
      //!
      static void from_pingOut_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
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

} // end namespace Svc

#endif
