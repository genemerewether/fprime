// ======================================================================
// \title  KraitRouter/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for KraitRouter component test harness base class
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

#ifndef KraitRouter_TESTER_BASE_HPP
#define KraitRouter_TESTER_BASE_HPP

#include <SnapdragonFlight/KraitRouter/KraitRouterComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace SnapdragonFlight {

  //! \class KraitRouterTesterBase
  //! \brief Auto-generated base class for KraitRouter component test harness
  //!
  class KraitRouterTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object KraitRouterTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect HexPortsIn to to_HexPortsIn[portNum]
      //!
      void connect_to_HexPortsIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputSerializePort *const HexPortsIn /*!< The port*/
      );

      //! Connect Sched to to_Sched[portNum]
      //!
      void connect_to_Sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const Sched /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from KraitPortsOut
      //!
      //! \return from_KraitPortsOut[portNum]
      //!
      Fw::InputSerializePort* get_from_KraitPortsOut(
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

      //! Construct object KraitRouterTesterBase
      //!
      KraitRouterTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object KraitRouterTesterBase
      //!
      virtual ~KraitRouterTesterBase(void);

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
      // Handler prototypes for serial from ports
      // ----------------------------------------------------------------------

      //! Handler prototype for from_KraitPortsOut
      //!
      virtual void from_KraitPortsOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      ) = 0;

      //! Handler base class function prototype for from_KraitPortsOut
      //!
      void from_KraitPortsOut_handlerBase (
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );


    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to HexPortsIn
      //!
      void invoke_to_HexPortsIn(
          NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer
      );

      //! Invoke the to port connected to Sched
      //!
      void invoke_to_Sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_HexPortsIn ports
      //!
      //! \return The number of to_HexPortsIn ports
      //!
      NATIVE_INT_TYPE getNum_to_HexPortsIn(void) const;

      //! Get the number of from_KraitPortsOut ports
      //!
      //! \return The number of from_KraitPortsOut ports
      //!
      NATIVE_INT_TYPE getNum_from_KraitPortsOut(void) const;

      //! Get the number of to_Sched ports
      //!
      //! \return The number of to_Sched ports
      //!
      NATIVE_INT_TYPE getNum_to_Sched(void) const;

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
      //! Whether to_HexPortsIn[portNum] is connected
      //!
      bool isConnected_to_HexPortsIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_Sched[portNum] is connected
      //!
      bool isConnected_to_Sched(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

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
      // Event: KR_DataReceiveError
      // ----------------------------------------------------------------------

      //! Handle event KR_DataReceiveError
      //!
      virtual void logIn_WARNING_HI_KR_DataReceiveError(
          KraitRouterComponentBase::KR_ReceiveError error, /*!< The receive error code*/
          I32 stat /*!< Status code - depends on error type*/
      );

      //! A history entry for event KR_DataReceiveError
      //!
      typedef struct {
        KraitRouterComponentBase::KR_ReceiveError error;
        I32 stat;
      } EventEntry_KR_DataReceiveError;

      //! The history of KR_DataReceiveError events
      //!
      History<EventEntry_KR_DataReceiveError> 
        *eventHistory_KR_DataReceiveError;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_InvalidPortNum
      // ----------------------------------------------------------------------

      //! Handle event KR_InvalidPortNum
      //!
      virtual void logIn_WARNING_HI_KR_InvalidPortNum(
          KraitRouterComponentBase::KR_PortType PortType, /*!< Type of port that was not connected*/
          U8 portNum 
      );

      //! A history entry for event KR_InvalidPortNum
      //!
      typedef struct {
        KraitRouterComponentBase::KR_PortType PortType;
        U8 portNum;
      } EventEntry_KR_InvalidPortNum;

      //! The history of KR_InvalidPortNum events
      //!
      History<EventEntry_KR_InvalidPortNum> 
        *eventHistory_KR_InvalidPortNum;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_BadSerialPortCall
      // ----------------------------------------------------------------------

      //! Handle event KR_BadSerialPortCall
      //!
      virtual void logIn_WARNING_HI_KR_BadSerialPortCall(
          I32 status, 
          U8 portNum 
      );

      //! A history entry for event KR_BadSerialPortCall
      //!
      typedef struct {
        I32 status;
        U8 portNum;
      } EventEntry_KR_BadSerialPortCall;

      //! The history of KR_BadSerialPortCall events
      //!
      History<EventEntry_KR_BadSerialPortCall> 
        *eventHistory_KR_BadSerialPortCall;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_MemoryError
      // ----------------------------------------------------------------------

      //! Handle event KR_MemoryError
      //!
      virtual void logIn_WARNING_HI_KR_MemoryError(
          KraitRouterComponentBase::KR_MemoryError error /*!< Type of memory operation that failed*/
      );

      //! A history entry for event KR_MemoryError
      //!
      typedef struct {
        KraitRouterComponentBase::KR_MemoryError error;
      } EventEntry_KR_MemoryError;

      //! The history of KR_MemoryError events
      //!
      History<EventEntry_KR_MemoryError> 
        *eventHistory_KR_MemoryError;

    protected:

      // ----------------------------------------------------------------------
      // Event: KR_WriteError
      // ----------------------------------------------------------------------

      //! Handle event KR_WriteError
      //!
      virtual void logIn_WARNING_HI_KR_WriteError(
          I32 error /*!< The error code*/
      );

      //! A history entry for event KR_WriteError
      //!
      typedef struct {
        I32 error;
      } EventEntry_KR_WriteError;

      //! The history of KR_WriteError events
      //!
      History<EventEntry_KR_WriteError> 
        *eventHistory_KR_WriteError;

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
      // Channel: KR_ResyncErrors
      // ----------------------------------------------------------------------

      //! Handle channel KR_ResyncErrors
      //!
      virtual void tlmInput_KR_ResyncErrors(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel KR_ResyncErrors
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_KR_ResyncErrors;

      //! The history of KR_ResyncErrors values
      //!
      History<TlmEntry_KR_ResyncErrors> 
        *tlmHistory_KR_ResyncErrors;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumPackets
      // ----------------------------------------------------------------------

      //! Handle channel KR_NumPackets
      //!
      virtual void tlmInput_KR_NumPackets(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel KR_NumPackets
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_KR_NumPackets;

      //! The history of KR_NumPackets values
      //!
      History<TlmEntry_KR_NumPackets> 
        *tlmHistory_KR_NumPackets;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumDecodeErrors
      // ----------------------------------------------------------------------

      //! Handle channel KR_NumDecodeErrors
      //!
      virtual void tlmInput_KR_NumDecodeErrors(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel KR_NumDecodeErrors
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_KR_NumDecodeErrors;

      //! The history of KR_NumDecodeErrors values
      //!
      History<TlmEntry_KR_NumDecodeErrors> 
        *tlmHistory_KR_NumDecodeErrors;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumInvalidPorts
      // ----------------------------------------------------------------------

      //! Handle channel KR_NumInvalidPorts
      //!
      virtual void tlmInput_KR_NumInvalidPorts(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel KR_NumInvalidPorts
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_KR_NumInvalidPorts;

      //! The history of KR_NumInvalidPorts values
      //!
      History<TlmEntry_KR_NumInvalidPorts> 
        *tlmHistory_KR_NumInvalidPorts;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumBadSerialPortCalls
      // ----------------------------------------------------------------------

      //! Handle channel KR_NumBadSerialPortCalls
      //!
      virtual void tlmInput_KR_NumBadSerialPortCalls(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel KR_NumBadSerialPortCalls
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_KR_NumBadSerialPortCalls;

      //! The history of KR_NumBadSerialPortCalls values
      //!
      History<TlmEntry_KR_NumBadSerialPortCalls> 
        *tlmHistory_KR_NumBadSerialPortCalls;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumOutputBufferOverflows
      // ----------------------------------------------------------------------

      //! Handle channel KR_NumOutputBufferOverflows
      //!
      virtual void tlmInput_KR_NumOutputBufferOverflows(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel KR_NumOutputBufferOverflows
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_KR_NumOutputBufferOverflows;

      //! The history of KR_NumOutputBufferOverflows values
      //!
      History<TlmEntry_KR_NumOutputBufferOverflows> 
        *tlmHistory_KR_NumOutputBufferOverflows;

    protected:

      // ----------------------------------------------------------------------
      // Channel: KR_NumZeroPktSize
      // ----------------------------------------------------------------------

      //! Handle channel KR_NumZeroPktSize
      //!
      virtual void tlmInput_KR_NumZeroPktSize(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel KR_NumZeroPktSize
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_KR_NumZeroPktSize;

      //! The history of KR_NumZeroPktSize values
      //!
      History<TlmEntry_KR_NumZeroPktSize> 
        *tlmHistory_KR_NumZeroPktSize;

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

      //! To port connected to HexPortsIn
      //!
      Fw::OutputSerializePort m_to_HexPortsIn[25];

      //! To port connected to Sched
      //!
      Svc::OutputSchedPort m_to_Sched[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to KraitPortsOut
      //!
      Fw::InputSerializePort m_from_KraitPortsOut[25];

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

      //! Static function for port from_KraitPortsOut
      //!
      static void from_KraitPortsOut_static(
          Fw::PassiveComponentBase *const callComp, //!< The component instance
          const NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer //!< serialized data buffer
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

} // end namespace SnapdragonFlight

#endif
