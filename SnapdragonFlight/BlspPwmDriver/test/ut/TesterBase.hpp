// ======================================================================
// \title  BlspPwmDriver/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for BlspPwmDriver component test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef BlspPwmDriver_TESTER_BASE_HPP
#define BlspPwmDriver_TESTER_BASE_HPP

#include <SnapdragonFlight/BlspPwmDriver/BlspPwmDriverComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace SnapdragonFlight {

  //! \class BlspPwmDriverTesterBase
  //! \brief Auto-generated base class for BlspPwmDriver component test harness
  //!
  class BlspPwmDriverTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object BlspPwmDriverTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect pwmConfig to to_pwmConfig[portNum]
      //!
      void connect_to_pwmConfig(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputPwmConfigDataPort *const pwmConfig /*!< The port*/
      );

      //! Connect pwmSetDuty to to_pwmSetDuty[portNum]
      //!
      void connect_to_pwmSetDuty(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputPwmSetDutyCycleDataPort *const pwmSetDuty /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

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

      //! Construct object BlspPwmDriverTesterBase
      //!
      BlspPwmDriverTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object BlspPwmDriverTesterBase
      //!
      virtual ~BlspPwmDriverTesterBase(void);

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
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to pwmConfig
      //!
      void invoke_to_pwmConfig(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::PwmConfig pwmConfig 
      );

      //! Invoke the to port connected to pwmSetDuty
      //!
      void invoke_to_pwmSetDuty(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::PwmSetDutyCycle pwmSetDutyCycle 
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_pwmConfig ports
      //!
      //! \return The number of to_pwmConfig ports
      //!
      NATIVE_INT_TYPE getNum_to_pwmConfig(void) const;

      //! Get the number of to_pwmSetDuty ports
      //!
      //! \return The number of to_pwmSetDuty ports
      //!
      NATIVE_INT_TYPE getNum_to_pwmSetDuty(void) const;

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
      //! Whether to_pwmConfig[portNum] is connected
      //!
      bool isConnected_to_pwmConfig(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_pwmSetDuty[portNum] is connected
      //!
      bool isConnected_to_pwmSetDuty(
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
      // Event: PWM_OpenError
      // ----------------------------------------------------------------------

      //! Handle event PWM_OpenError
      //!
      virtual void logIn_WARNING_HI_PWM_OpenError(
          I32 chip, /*!< The PWM chip*/
          I32 line, /*!< The PWM line*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event PWM_OpenError
      //!
      typedef struct {
        I32 chip;
        I32 line;
        I32 error;
      } EventEntry_PWM_OpenError;

      //! The history of PWM_OpenError events
      //!
      History<EventEntry_PWM_OpenError>
        *eventHistory_PWM_OpenError;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_ConfigError
      // ----------------------------------------------------------------------

      //! Handle event PWM_ConfigError
      //!
      virtual void logIn_WARNING_HI_PWM_ConfigError(
          I32 chip, /*!< The PWM chip*/
          I32 line, /*!< The PWM line*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event PWM_ConfigError
      //!
      typedef struct {
        I32 chip;
        I32 line;
        I32 error;
      } EventEntry_PWM_ConfigError;

      //! The history of PWM_ConfigError events
      //!
      History<EventEntry_PWM_ConfigError>
        *eventHistory_PWM_ConfigError;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_WriteError
      // ----------------------------------------------------------------------

      //! Handle event PWM_WriteError
      //!
      virtual void logIn_WARNING_HI_PWM_WriteError(
          I32 chip, /*!< The PWM chip*/
          I32 line, /*!< The PWM line*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event PWM_WriteError
      //!
      typedef struct {
        I32 chip;
        I32 line;
        I32 error;
      } EventEntry_PWM_WriteError;

      //! The history of PWM_WriteError events
      //!
      History<EventEntry_PWM_WriteError>
        *eventHistory_PWM_WriteError;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_ReadError
      // ----------------------------------------------------------------------

      //! Handle event PWM_ReadError
      //!
      virtual void logIn_WARNING_HI_PWM_ReadError(
          I32 chip, /*!< The PWM chip*/
          I32 line, /*!< The PWM line*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event PWM_ReadError
      //!
      typedef struct {
        I32 chip;
        I32 line;
        I32 error;
      } EventEntry_PWM_ReadError;

      //! The history of PWM_ReadError events
      //!
      History<EventEntry_PWM_ReadError>
        *eventHistory_PWM_ReadError;

    protected:

      // ----------------------------------------------------------------------
      // Event: PWM_PortOpened
      // ----------------------------------------------------------------------

      //! Handle event PWM_PortOpened
      //!
      virtual void logIn_ACTIVITY_HI_PWM_PortOpened(
          I32 chip, /*!< The PWM chip*/
          I32 line /*!< The PWM line*/
      );

      //! A history entry for event PWM_PortOpened
      //!
      typedef struct {
        I32 chip;
        I32 line;
      } EventEntry_PWM_PortOpened;

      //! The history of PWM_PortOpened events
      //!
      History<EventEntry_PWM_PortOpened>
        *eventHistory_PWM_PortOpened;

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

      //! To port connected to pwmConfig
      //!
      Drv::OutputPwmConfigDataPort m_to_pwmConfig[1];

      //! To port connected to pwmSetDuty
      //!
      Drv::OutputPwmSetDutyCycleDataPort m_to_pwmSetDuty[2];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

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
