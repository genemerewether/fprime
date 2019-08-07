// ======================================================================
// \title  ActiveL1PrmDb/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for ActiveL1PrmDb component test harness base class
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

#ifndef ActiveL1PrmDb_TESTER_BASE_HPP
#define ActiveL1PrmDb_TESTER_BASE_HPP

#include <Svc/ActiveL1PrmDb/ActiveL1PrmDbComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Svc {

  //! \class ActiveL1PrmDbTesterBase
  //! \brief Auto-generated base class for ActiveL1PrmDb component test harness
  //!
  class ActiveL1PrmDbTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object ActiveL1PrmDbTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect sendPrmReady to to_sendPrmReady[portNum]
      //!
      void connect_to_sendPrmReady(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputPrmListReadyPort *const sendPrmReady /*!< The port*/
      );

      //! Connect setPrm to to_setPrm[portNum]
      //!
      void connect_to_setPrm(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputPrmSetPort *const setPrm /*!< The port*/
      );

      //! Connect recvPrm to to_recvPrm[portNum]
      //!
      void connect_to_recvPrm(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputPrmListPort *const recvPrm /*!< The port*/
      );

      //! Connect pingIn to to_pingIn[portNum]
      //!
      void connect_to_pingIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputPingPort *const pingIn /*!< The port*/
      );

      //! Connect getPrm to to_getPrm[portNum]
      //!
      void connect_to_getPrm(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputPrmGetPort *const getPrm /*!< The port*/
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

      //! Get the port that receives input from sendPrm
      //!
      //! \return from_sendPrm[portNum]
      //!
      Fw::InputPrmListPort* get_from_sendPrm(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from recvPrmReady
      //!
      //! \return from_recvPrmReady[portNum]
      //!
      Fw::InputPrmListReadyPort* get_from_recvPrmReady(
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

      //! Construct object ActiveL1PrmDbTesterBase
      //!
      ActiveL1PrmDbTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object ActiveL1PrmDbTesterBase
      //!
      virtual ~ActiveL1PrmDbTesterBase(void);

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

      //! Handler prototype for from_sendPrm
      //!
      virtual void from_sendPrm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool morePrms, 
          Fw::ParamList val 
      ) = 0;

      //! Handler base function for from_sendPrm
      //!
      void from_sendPrm_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool morePrms, 
          Fw::ParamList val 
      );

      //! Handler prototype for from_recvPrmReady
      //!
      virtual void from_recvPrmReady_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 maxSize, 
          bool reload 
      ) = 0;

      //! Handler base function for from_recvPrmReady
      //!
      void from_recvPrmReady_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 maxSize, 
          bool reload 
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

      //! Push an entry on the history for from_sendPrm
      void pushFromPortEntry_sendPrm(
          bool morePrms, 
          Fw::ParamList val 
      );

      //! A history entry for from_sendPrm
      //!
      typedef struct {
        bool morePrms;
        Fw::ParamList val;
      } FromPortEntry_sendPrm;

      //! The history for from_sendPrm
      //!
      History<FromPortEntry_sendPrm> 
        *fromPortHistory_sendPrm;

      //! Push an entry on the history for from_recvPrmReady
      void pushFromPortEntry_recvPrmReady(
          U32 maxSize, 
          bool reload 
      );

      //! A history entry for from_recvPrmReady
      //!
      typedef struct {
        U32 maxSize;
        bool reload;
      } FromPortEntry_recvPrmReady;

      //! The history for from_recvPrmReady
      //!
      History<FromPortEntry_recvPrmReady> 
        *fromPortHistory_recvPrmReady;

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

      //! Invoke the to port connected to sendPrmReady
      //!
      void invoke_to_sendPrmReady(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 maxSize, 
          bool reload 
      );

      //! Invoke the to port connected to setPrm
      //!
      void invoke_to_setPrm(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, /*!< Parameter ID*/
          Fw::ParamBuffer &val /*!< Buffer containing serialized parameter value*/
      );

      //! Invoke the to port connected to recvPrm
      //!
      void invoke_to_recvPrm(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool morePrms, 
          Fw::ParamList val 
      );

      //! Invoke the to port connected to pingIn
      //!
      void invoke_to_pingIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Invoke the to port connected to getPrm
      //!
      Fw::ParamValid invoke_to_getPrm(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, /*!< Parameter ID*/
          Fw::ParamBuffer &val /*!< Buffer containing serialized parameter value*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of from_sendPrm ports
      //!
      //! \return The number of from_sendPrm ports
      //!
      NATIVE_INT_TYPE getNum_from_sendPrm(void) const;

      //! Get the number of from_recvPrmReady ports
      //!
      //! \return The number of from_recvPrmReady ports
      //!
      NATIVE_INT_TYPE getNum_from_recvPrmReady(void) const;

      //! Get the number of to_sendPrmReady ports
      //!
      //! \return The number of to_sendPrmReady ports
      //!
      NATIVE_INT_TYPE getNum_to_sendPrmReady(void) const;

      //! Get the number of to_setPrm ports
      //!
      //! \return The number of to_setPrm ports
      //!
      NATIVE_INT_TYPE getNum_to_setPrm(void) const;

      //! Get the number of to_recvPrm ports
      //!
      //! \return The number of to_recvPrm ports
      //!
      NATIVE_INT_TYPE getNum_to_recvPrm(void) const;

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

      //! Get the number of to_getPrm ports
      //!
      //! \return The number of to_getPrm ports
      //!
      NATIVE_INT_TYPE getNum_to_getPrm(void) const;

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
      //! Whether to_sendPrmReady[portNum] is connected
      //!
      bool isConnected_to_sendPrmReady(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_setPrm[portNum] is connected
      //!
      bool isConnected_to_setPrm(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_recvPrm[portNum] is connected
      //!
      bool isConnected_to_recvPrm(
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
      //! Whether to_getPrm[portNum] is connected
      //!
      bool isConnected_to_getPrm(
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

      //! Send a PRM_SAVE_FILE command
      //!
      void sendCmd_PRM_SAVE_FILE(
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
      // Event: PrmIdNotFound
      // ----------------------------------------------------------------------

      //! Handle event PrmIdNotFound
      //!
      virtual void logIn_WARNING_LO_PrmIdNotFound(
          U32 Id /*!< The parameter ID*/
      );

      //! A history entry for event PrmIdNotFound
      //!
      typedef struct {
        U32 Id;
      } EventEntry_PrmIdNotFound;

      //! The history of PrmIdNotFound events
      //!
      History<EventEntry_PrmIdNotFound> 
        *eventHistory_PrmIdNotFound;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmIdUpdated
      // ----------------------------------------------------------------------

      //! Handle event PrmIdUpdated
      //!
      virtual void logIn_ACTIVITY_HI_PrmIdUpdated(
          U32 Id /*!< The parameter ID*/
      );

      //! A history entry for event PrmIdUpdated
      //!
      typedef struct {
        U32 Id;
      } EventEntry_PrmIdUpdated;

      //! The history of PrmIdUpdated events
      //!
      History<EventEntry_PrmIdUpdated> 
        *eventHistory_PrmIdUpdated;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmDbFull
      // ----------------------------------------------------------------------

      //! Handle event PrmDbFull
      //!
      virtual void logIn_FATAL_PrmDbFull(
          U32 Id /*!< The parameter ID*/
      );

      //! A history entry for event PrmDbFull
      //!
      typedef struct {
        U32 Id;
      } EventEntry_PrmDbFull;

      //! The history of PrmDbFull events
      //!
      History<EventEntry_PrmDbFull> 
        *eventHistory_PrmDbFull;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmIdAdded
      // ----------------------------------------------------------------------

      //! Handle event PrmIdAdded
      //!
      virtual void logIn_ACTIVITY_HI_PrmIdAdded(
          U32 Id /*!< The parameter ID*/
      );

      //! A history entry for event PrmIdAdded
      //!
      typedef struct {
        U32 Id;
      } EventEntry_PrmIdAdded;

      //! The history of PrmIdAdded events
      //!
      History<EventEntry_PrmIdAdded> 
        *eventHistory_PrmIdAdded;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmFileWriteError
      // ----------------------------------------------------------------------

      //! Handle event PrmFileWriteError
      //!
      virtual void logIn_WARNING_HI_PrmFileWriteError(
          ActiveL1PrmDbComponentBase::PrmWriteError stage, /*!< The write stage*/
          I32 record, /*!< The record that had the failure*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event PrmFileWriteError
      //!
      typedef struct {
        ActiveL1PrmDbComponentBase::PrmWriteError stage;
        I32 record;
        I32 error;
      } EventEntry_PrmFileWriteError;

      //! The history of PrmFileWriteError events
      //!
      History<EventEntry_PrmFileWriteError> 
        *eventHistory_PrmFileWriteError;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmFileSaveComplete
      // ----------------------------------------------------------------------

      //! Handle event PrmFileSaveComplete
      //!
      virtual void logIn_ACTIVITY_HI_PrmFileSaveComplete(
          U32 records /*!< The number of records saved*/
      );

      //! A history entry for event PrmFileSaveComplete
      //!
      typedef struct {
        U32 records;
      } EventEntry_PrmFileSaveComplete;

      //! The history of PrmFileSaveComplete events
      //!
      History<EventEntry_PrmFileSaveComplete> 
        *eventHistory_PrmFileSaveComplete;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmFileReadError
      // ----------------------------------------------------------------------

      //! Handle event PrmFileReadError
      //!
      virtual void logIn_WARNING_HI_PrmFileReadError(
          ActiveL1PrmDbComponentBase::PrmReadError stage, /*!< The write stage*/
          I32 record, /*!< The record that had the failure*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event PrmFileReadError
      //!
      typedef struct {
        ActiveL1PrmDbComponentBase::PrmReadError stage;
        I32 record;
        I32 error;
      } EventEntry_PrmFileReadError;

      //! The history of PrmFileReadError events
      //!
      History<EventEntry_PrmFileReadError> 
        *eventHistory_PrmFileReadError;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmFileLoadComplete
      // ----------------------------------------------------------------------

      //! Handle event PrmFileLoadComplete
      //!
      virtual void logIn_ACTIVITY_HI_PrmFileLoadComplete(
          U32 records /*!< The number of records loaded*/
      );

      //! A history entry for event PrmFileLoadComplete
      //!
      typedef struct {
        U32 records;
      } EventEntry_PrmFileLoadComplete;

      //! The history of PrmFileLoadComplete events
      //!
      History<EventEntry_PrmFileLoadComplete> 
        *eventHistory_PrmFileLoadComplete;

    protected:

      // ----------------------------------------------------------------------
      // Event: PrmSendTooLarge
      // ----------------------------------------------------------------------

      //! Handle event PrmSendTooLarge
      //!
      virtual void logIn_WARNING_HI_PrmSendTooLarge(
          U32 prmId, /*!< Parameter that could not be sent*/
          U32 prmSize, /*!< Size of serialized parameter*/
          I32 portNum /*!< Port Number of level 2 prmDb*/
      );

      //! A history entry for event PrmSendTooLarge
      //!
      typedef struct {
        U32 prmId;
        U32 prmSize;
        I32 portNum;
      } EventEntry_PrmSendTooLarge;

      //! The history of PrmSendTooLarge events
      //!
      History<EventEntry_PrmSendTooLarge> 
        *eventHistory_PrmSendTooLarge;

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

      //! To port connected to sendPrmReady
      //!
      Fw::OutputPrmListReadyPort m_to_sendPrmReady[5];

      //! To port connected to setPrm
      //!
      Fw::OutputPrmSetPort m_to_setPrm[1];

      //! To port connected to recvPrm
      //!
      Fw::OutputPrmListPort m_to_recvPrm[5];

      //! To port connected to pingIn
      //!
      Svc::OutputPingPort m_to_pingIn[1];

      //! To port connected to getPrm
      //!
      Fw::OutputPrmGetPort m_to_getPrm[1];

      //! To port connected to CmdDisp
      //!
      Fw::OutputCmdPort m_to_CmdDisp[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to sendPrm
      //!
      Fw::InputPrmListPort m_from_sendPrm[5];

      //! From port connected to recvPrmReady
      //!
      Fw::InputPrmListReadyPort m_from_recvPrmReady[5];

      //! From port connected to pingOut
      //!
      Svc::InputPingPort m_from_pingOut[1];

      //! From port connected to CmdStatus
      //!
      Fw::InputCmdResponsePort m_from_CmdStatus[1];

      //! From port connected to CmdReg
      //!
      Fw::InputCmdRegPort m_from_CmdReg[1];

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

      //! Static function for port from_sendPrm
      //!
      static void from_sendPrm_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool morePrms, 
          Fw::ParamList val 
      );

      //! Static function for port from_recvPrmReady
      //!
      static void from_recvPrmReady_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 maxSize, 
          bool reload 
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

} // end namespace Svc

#endif
