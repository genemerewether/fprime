// ======================================================================
// \title  LIDARLiteV3/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for LIDARLiteV3 component test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef LIDARLiteV3_TESTER_BASE_HPP
#define LIDARLiteV3_TESTER_BASE_HPP

#include <Drv/Altimeter/LIDARLiteV3/LIDARLiteV3ComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Drv {

  //! \class LIDARLiteV3TesterBase
  //! \brief Auto-generated base class for LIDARLiteV3 component test harness
  //!
  class LIDARLiteV3TesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object LIDARLiteV3TesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect SchedIn to to_SchedIn[portNum]
      //!
      void connect_to_SchedIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const SchedIn /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from I2CWriteRead
      //!
      //! \return from_I2CWriteRead[portNum]
      //!
      Drv::InputI2CWriteReadPort* get_from_I2CWriteRead(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from I2CWriteReadStatus
      //!
      //! \return from_I2CWriteReadStatus[portNum]
      //!
      Drv::InputI2CWriteReadStatusPort* get_from_I2CWriteReadStatus(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from I2CConfig
      //!
      //! \return from_I2CConfig[portNum]
      //!
      Drv::InputI2CConfigPort* get_from_I2CConfig(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from AltimeterSend
      //!
      //! \return from_AltimeterSend[portNum]
      //!
      ROS::sensor_msgs::InputRangePort* get_from_AltimeterSend(
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

      //! Construct object LIDARLiteV3TesterBase
      //!
      LIDARLiteV3TesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object LIDARLiteV3TesterBase
      //!
      virtual ~LIDARLiteV3TesterBase(void);

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

      //! Handler prototype for from_I2CWriteRead
      //!
      virtual void from_I2CWriteRead_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &writeBuffer, 
          Fw::Buffer &readBuffer 
      ) = 0;

      //! Handler base function for from_I2CWriteRead
      //!
      void from_I2CWriteRead_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &writeBuffer, 
          Fw::Buffer &readBuffer 
      );

      //! Handler prototype for from_I2CWriteReadStatus
      //!
      virtual Drv::I2CStatus from_I2CWriteReadStatus_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool shouldBlock 
      ) = 0;

      //! Handler base function for from_I2CWriteReadStatus
      //!
      Drv::I2CStatus from_I2CWriteReadStatus_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool shouldBlock 
      );

      //! Handler prototype for from_I2CConfig
      //!
      virtual void from_I2CConfig_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 busSpeed, 
          U32 slaveAddr, 
          U32 timeout 
      ) = 0;

      //! Handler base function for from_I2CConfig
      //!
      void from_I2CConfig_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 busSpeed, 
          U32 slaveAddr, 
          U32 timeout 
      );

      //! Handler prototype for from_AltimeterSend
      //!
      virtual void from_AltimeterSend_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::Range &Range 
      ) = 0;

      //! Handler base function for from_AltimeterSend
      //!
      void from_AltimeterSend_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::Range &Range 
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

      //! Push an entry on the history for from_I2CWriteRead
      void pushFromPortEntry_I2CWriteRead(
          Fw::Buffer &writeBuffer, 
          Fw::Buffer &readBuffer 
      );

      //! A history entry for from_I2CWriteRead
      //!
      typedef struct {
        Fw::Buffer writeBuffer;
        Fw::Buffer readBuffer;
      } FromPortEntry_I2CWriteRead;

      //! The history for from_I2CWriteRead
      //!
      History<FromPortEntry_I2CWriteRead>
        *fromPortHistory_I2CWriteRead;

      //! Push an entry on the history for from_I2CWriteReadStatus
      void pushFromPortEntry_I2CWriteReadStatus(
          bool shouldBlock 
      );

      //! A history entry for from_I2CWriteReadStatus
      //!
      typedef struct {
        bool shouldBlock;
      } FromPortEntry_I2CWriteReadStatus;

      //! The history for from_I2CWriteReadStatus
      //!
      History<FromPortEntry_I2CWriteReadStatus>
        *fromPortHistory_I2CWriteReadStatus;

      //! Push an entry on the history for from_I2CConfig
      void pushFromPortEntry_I2CConfig(
          U32 busSpeed, 
          U32 slaveAddr, 
          U32 timeout 
      );

      //! A history entry for from_I2CConfig
      //!
      typedef struct {
        U32 busSpeed;
        U32 slaveAddr;
        U32 timeout;
      } FromPortEntry_I2CConfig;

      //! The history for from_I2CConfig
      //!
      History<FromPortEntry_I2CConfig>
        *fromPortHistory_I2CConfig;

      //! Push an entry on the history for from_AltimeterSend
      void pushFromPortEntry_AltimeterSend(
          ROS::sensor_msgs::Range &Range 
      );

      //! A history entry for from_AltimeterSend
      //!
      typedef struct {
        ROS::sensor_msgs::Range Range;
      } FromPortEntry_AltimeterSend;

      //! The history for from_AltimeterSend
      //!
      History<FromPortEntry_AltimeterSend>
        *fromPortHistory_AltimeterSend;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to SchedIn
      //!
      void invoke_to_SchedIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_SchedIn ports
      //!
      //! \return The number of to_SchedIn ports
      //!
      NATIVE_INT_TYPE getNum_to_SchedIn(void) const;

      //! Get the number of from_I2CWriteRead ports
      //!
      //! \return The number of from_I2CWriteRead ports
      //!
      NATIVE_INT_TYPE getNum_from_I2CWriteRead(void) const;

      //! Get the number of from_I2CWriteReadStatus ports
      //!
      //! \return The number of from_I2CWriteReadStatus ports
      //!
      NATIVE_INT_TYPE getNum_from_I2CWriteReadStatus(void) const;

      //! Get the number of from_I2CConfig ports
      //!
      //! \return The number of from_I2CConfig ports
      //!
      NATIVE_INT_TYPE getNum_from_I2CConfig(void) const;

      //! Get the number of from_AltimeterSend ports
      //!
      //! \return The number of from_AltimeterSend ports
      //!
      NATIVE_INT_TYPE getNum_from_AltimeterSend(void) const;

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
      //! Whether to_SchedIn[portNum] is connected
      //!
      bool isConnected_to_SchedIn(
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
      // Event: LLV3_InitComplete
      // ----------------------------------------------------------------------

      //! Handle event LLV3_InitComplete
      //!
      virtual void logIn_ACTIVITY_LO_LLV3_InitComplete(
          void
      );

      //! Size of history for event LLV3_InitComplete
      //!
      U32 eventsSize_LLV3_InitComplete;

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
      // Channel: LLV3_Distance
      // ----------------------------------------------------------------------

      //! Handle channel LLV3_Distance
      //!
      virtual void tlmInput_LLV3_Distance(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLV3_Distance
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_LLV3_Distance;

      //! The history of LLV3_Distance values
      //!
      History<TlmEntry_LLV3_Distance>
        *tlmHistory_LLV3_Distance;

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

      //! To port connected to SchedIn
      //!
      Svc::OutputSchedPort m_to_SchedIn[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to I2CWriteRead
      //!
      Drv::InputI2CWriteReadPort m_from_I2CWriteRead[1];

      //! From port connected to I2CWriteReadStatus
      //!
      Drv::InputI2CWriteReadStatusPort m_from_I2CWriteReadStatus[1];

      //! From port connected to I2CConfig
      //!
      Drv::InputI2CConfigPort m_from_I2CConfig[1];

      //! From port connected to AltimeterSend
      //!
      ROS::sensor_msgs::InputRangePort m_from_AltimeterSend[1];

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

      //! Static function for port from_I2CWriteRead
      //!
      static void from_I2CWriteRead_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &writeBuffer, 
          Fw::Buffer &readBuffer 
      );

      //! Static function for port from_I2CWriteReadStatus
      //!
      static Drv::I2CStatus from_I2CWriteReadStatus_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool shouldBlock 
      );

      //! Static function for port from_I2CConfig
      //!
      static void from_I2CConfig_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 busSpeed, 
          U32 slaveAddr, 
          U32 timeout 
      );

      //! Static function for port from_AltimeterSend
      //!
      static void from_AltimeterSend_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::Range &Range 
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
