// ======================================================================
// \title  STIM300/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for STIM300 component test harness base class
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

#ifndef STIM300_TESTER_BASE_HPP
#define STIM300_TESTER_BASE_HPP

#include <Drv/IMU/STIM300/STIM300ComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Drv {

  //! \class STIM300TesterBase
  //! \brief Auto-generated base class for STIM300 component test harness
  //!
  class STIM300TesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object STIM300TesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect sched to to_sched[portNum]
      //!
      void connect_to_sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const sched /*!< The port*/
      );

      //! Connect serialRead to to_serialRead[portNum]
      //!
      void connect_to_serialRead(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputSerialReadPort *const serialRead /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from IMU
      //!
      //! \return from_IMU[portNum]
      //!
      ROS::sensor_msgs::InputImuNoCovPort* get_from_IMU(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from packetTime
      //!
      //! \return from_packetTime[portNum]
      //!
      Fw::InputTimePort* get_from_packetTime(
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

      //! Construct object STIM300TesterBase
      //!
      STIM300TesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object STIM300TesterBase
      //!
      virtual ~STIM300TesterBase(void);

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

      //! Handler prototype for from_IMU
      //!
      virtual void from_IMU_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &ImuNoCov 
      ) = 0;

      //! Handler base function for from_IMU
      //!
      void from_IMU_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &ImuNoCov 
      );

      //! Handler prototype for from_packetTime
      //!
      virtual void from_packetTime_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      ) = 0;

      //! Handler base function for from_packetTime
      //!
      void from_packetTime_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
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

      //! Push an entry on the history for from_IMU
      void pushFromPortEntry_IMU(
          ROS::sensor_msgs::ImuNoCov &ImuNoCov 
      );

      //! A history entry for from_IMU
      //!
      typedef struct {
        ROS::sensor_msgs::ImuNoCov ImuNoCov;
      } FromPortEntry_IMU;

      //! The history for from_IMU
      //!
      History<FromPortEntry_IMU> 
        *fromPortHistory_IMU;

      //! Push an entry on the history for from_packetTime
      void pushFromPortEntry_packetTime(
          Fw::Time &time /*!< The U32 cmd argument*/
      );

      //! A history entry for from_packetTime
      //!
      typedef struct {
        Fw::Time time;
      } FromPortEntry_packetTime;

      //! The history for from_packetTime
      //!
      History<FromPortEntry_packetTime> 
        *fromPortHistory_packetTime;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to sched
      //!
      void invoke_to_sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Invoke the to port connected to serialRead
      //!
      void invoke_to_serialRead(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          SerialReadStatus &status /*!< Status of read*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of from_IMU ports
      //!
      //! \return The number of from_IMU ports
      //!
      NATIVE_INT_TYPE getNum_from_IMU(void) const;

      //! Get the number of to_sched ports
      //!
      //! \return The number of to_sched ports
      //!
      NATIVE_INT_TYPE getNum_to_sched(void) const;

      //! Get the number of from_packetTime ports
      //!
      //! \return The number of from_packetTime ports
      //!
      NATIVE_INT_TYPE getNum_from_packetTime(void) const;

      //! Get the number of to_serialRead ports
      //!
      //! \return The number of to_serialRead ports
      //!
      NATIVE_INT_TYPE getNum_to_serialRead(void) const;

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
      //! Whether to_sched[portNum] is connected
      //!
      bool isConnected_to_sched(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_serialRead[portNum] is connected
      //!
      bool isConnected_to_serialRead(
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
      // Event: BufferFull
      // ----------------------------------------------------------------------

      //! Handle event BufferFull
      //!
      virtual void logIn_WARNING_HI_BufferFull(
          void
      );

      //! Size of history for event BufferFull
      //!
      U32 eventsSize_BufferFull;

    protected:

      // ----------------------------------------------------------------------
      // Event: UartError
      // ----------------------------------------------------------------------

      //! Handle event UartError
      //!
      virtual void logIn_WARNING_HI_UartError(
          void
      );

      //! Size of history for event UartError
      //!
      U32 eventsSize_UartError;

    protected:

      // ----------------------------------------------------------------------
      // Event: NoEvents
      // ----------------------------------------------------------------------

      //! Handle event NoEvents
      //!
      virtual void logIn_WARNING_HI_NoEvents(
          void
      );

      //! Size of history for event NoEvents
      //!
      U32 eventsSize_NoEvents;

    protected:

      // ----------------------------------------------------------------------
      // Event: InvalidCounter
      // ----------------------------------------------------------------------

      //! Handle event InvalidCounter
      //!
      virtual void logIn_WARNING_HI_InvalidCounter(
          U32 actualCount, 
          U32 exptecCount 
      );

      //! A history entry for event InvalidCounter
      //!
      typedef struct {
        U32 actualCount;
        U32 exptecCount;
      } EventEntry_InvalidCounter;

      //! The history of InvalidCounter events
      //!
      History<EventEntry_InvalidCounter> 
        *eventHistory_InvalidCounter;

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
      // Channel: NumPackets
      // ----------------------------------------------------------------------

      //! Handle channel NumPackets
      //!
      virtual void tlmInput_NumPackets(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel NumPackets
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_NumPackets;

      //! The history of NumPackets values
      //!
      History<TlmEntry_NumPackets> 
        *tlmHistory_NumPackets;

    protected:

      // ----------------------------------------------------------------------
      // Channel: ImuPacket
      // ----------------------------------------------------------------------

      //! Handle channel ImuPacket
      //!
      virtual void tlmInput_ImuPacket(
          const Fw::Time& timeTag, /*!< The time*/
          const ROS::sensor_msgs::ImuNoCov& val /*!< The channel value*/
      );

      //! A telemetry entry for channel ImuPacket
      //!
      typedef struct {
        Fw::Time timeTag;
        ROS::sensor_msgs::ImuNoCov arg;
      } TlmEntry_ImuPacket;

      //! The history of ImuPacket values
      //!
      History<TlmEntry_ImuPacket> 
        *tlmHistory_ImuPacket;

    protected:

      // ----------------------------------------------------------------------
      // Channel: TimeSyncStatus
      // ----------------------------------------------------------------------

      //! Handle channel TimeSyncStatus
      //!
      virtual void tlmInput_TimeSyncStatus(
          const Fw::Time& timeTag, /*!< The time*/
          const STIM300ComponentBase::STIM300TimeSync& val /*!< The channel value*/
      );

      //! A telemetry entry for channel TimeSyncStatus
      //!
      typedef struct {
        Fw::Time timeTag;
        STIM300ComponentBase::STIM300TimeSync arg;
      } TlmEntry_TimeSyncStatus;

      //! The history of TimeSyncStatus values
      //!
      History<TlmEntry_TimeSyncStatus> 
        *tlmHistory_TimeSyncStatus;

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

      //! To port connected to sched
      //!
      Svc::OutputSchedPort m_to_sched[1];

      //! To port connected to serialRead
      //!
      Drv::OutputSerialReadPort m_to_serialRead[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to IMU
      //!
      ROS::sensor_msgs::InputImuNoCovPort m_from_IMU[1];

      //! From port connected to packetTime
      //!
      Fw::InputTimePort m_from_packetTime[1];

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

      //! Static function for port from_IMU
      //!
      static void from_IMU_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &ImuNoCov 
      );

      //! Static function for port from_packetTime
      //!
      static void from_packetTime_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
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

} // end namespace Drv

#endif
