// ======================================================================
// \title  BlspI2CDriver/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for BlspI2CDriver component test harness base class
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

#ifndef BlspI2CDriver_TESTER_BASE_HPP
#define BlspI2CDriver_TESTER_BASE_HPP

#include <SnapdragonFlight/BlspI2CDriver/BlspI2CDriverComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace SnapdragonFlight {

  //! \class BlspI2CDriverTesterBase
  //! \brief Auto-generated base class for BlspI2CDriver component test harness
  //!
  class BlspI2CDriverTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object BlspI2CDriverTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect I2CReadWrite to to_I2CReadWrite[portNum]
      //!
      void connect_to_I2CReadWrite(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputI2CWriteReadPort *const I2CReadWrite /*!< The port*/
      );

      //! Connect I2CConfig to to_I2CConfig[portNum]
      //!
      void connect_to_I2CConfig(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputI2CConfigPort *const I2CConfig /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

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

      //! Get the port that receives input from Time
      //!
      //! \return from_Time[portNum]
      //!
      Fw::InputTimePort* get_from_Time(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object BlspI2CDriverTesterBase
      //!
      BlspI2CDriverTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object BlspI2CDriverTesterBase
      //!
      virtual ~BlspI2CDriverTesterBase(void);

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

      //! Invoke the to port connected to I2CReadWrite
      //!
      void invoke_to_I2CReadWrite(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &writeBuffer, 
          Fw::Buffer &readBuffer 
      );

      //! Invoke the to port connected to I2CConfig
      //!
      void invoke_to_I2CConfig(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 busSpeed, 
          U32 slaveAddr, 
          U32 timeout 
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

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

      //! Get the number of to_I2CReadWrite ports
      //!
      //! \return The number of to_I2CReadWrite ports
      //!
      NATIVE_INT_TYPE getNum_to_I2CReadWrite(void) const;

      //! Get the number of to_I2CConfig ports
      //!
      //! \return The number of to_I2CConfig ports
      //!
      NATIVE_INT_TYPE getNum_to_I2CConfig(void) const;

#if FW_ENABLE_TEXT_LOGGING == 1
      //! Get the number of from_LogText ports
      //!
      //! \return The number of from_LogText ports
      //!
      NATIVE_INT_TYPE getNum_from_LogText(void) const;
#endif

      //! Get the number of from_Time ports
      //!
      //! \return The number of from_Time ports
      //!
      NATIVE_INT_TYPE getNum_from_Time(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_I2CReadWrite[portNum] is connected
      //!
      bool isConnected_to_I2CReadWrite(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_I2CConfig[portNum] is connected
      //!
      bool isConnected_to_I2CConfig(
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
      // Event: I2C_OpenError
      // ----------------------------------------------------------------------

      //! Handle event I2C_OpenError
      //!
      virtual void logIn_WARNING_HI_I2C_OpenError(
          I32 device, /*!< The device*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event I2C_OpenError
      //!
      typedef struct {
        I32 device;
        I32 error;
      } EventEntry_I2C_OpenError;

      //! The history of I2C_OpenError events
      //!
      History<EventEntry_I2C_OpenError> 
        *eventHistory_I2C_OpenError;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_ConfigError
      // ----------------------------------------------------------------------

      //! Handle event I2C_ConfigError
      //!
      virtual void logIn_WARNING_HI_I2C_ConfigError(
          I32 device, /*!< The device*/
          I32 error /*!< The error code*/
      );

      //! A history entry for event I2C_ConfigError
      //!
      typedef struct {
        I32 device;
        I32 error;
      } EventEntry_I2C_ConfigError;

      //! The history of I2C_ConfigError events
      //!
      History<EventEntry_I2C_ConfigError> 
        *eventHistory_I2C_ConfigError;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_ReadWriteError
      // ----------------------------------------------------------------------

      //! Handle event I2C_ReadWriteError
      //!
      virtual void logIn_WARNING_HI_I2C_ReadWriteError(
          I32 device, /*!< The device*/
          U32 addr, 
          I32 error, /*!< The error code*/
          I32 bytesAct, 
          I32 bytesDes 
      );

      //! A history entry for event I2C_ReadWriteError
      //!
      typedef struct {
        I32 device;
        U32 addr;
        I32 error;
        I32 bytesAct;
        I32 bytesDes;
      } EventEntry_I2C_ReadWriteError;

      //! The history of I2C_ReadWriteError events
      //!
      History<EventEntry_I2C_ReadWriteError> 
        *eventHistory_I2C_ReadWriteError;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_WriteError
      // ----------------------------------------------------------------------

      //! Handle event I2C_WriteError
      //!
      virtual void logIn_WARNING_HI_I2C_WriteError(
          I32 device, /*!< The device*/
          U32 addr, 
          I32 error, /*!< The error code*/
          I32 bytesAct, 
          I32 bytesDes 
      );

      //! A history entry for event I2C_WriteError
      //!
      typedef struct {
        I32 device;
        U32 addr;
        I32 error;
        I32 bytesAct;
        I32 bytesDes;
      } EventEntry_I2C_WriteError;

      //! The history of I2C_WriteError events
      //!
      History<EventEntry_I2C_WriteError> 
        *eventHistory_I2C_WriteError;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_PortOpened
      // ----------------------------------------------------------------------

      //! Handle event I2C_PortOpened
      //!
      virtual void logIn_ACTIVITY_HI_I2C_PortOpened(
          I32 device /*!< The device*/
      );

      //! A history entry for event I2C_PortOpened
      //!
      typedef struct {
        I32 device;
      } EventEntry_I2C_PortOpened;

      //! The history of I2C_PortOpened events
      //!
      History<EventEntry_I2C_PortOpened> 
        *eventHistory_I2C_PortOpened;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_ReadError
      // ----------------------------------------------------------------------

      //! Handle event I2C_ReadError
      //!
      virtual void logIn_WARNING_HI_I2C_ReadError(
          I32 device, /*!< The device*/
          U32 addr, 
          I32 error, /*!< The error code*/
          I32 bytesAct, 
          I32 bytesDes 
      );

      //! A history entry for event I2C_ReadError
      //!
      typedef struct {
        I32 device;
        U32 addr;
        I32 error;
        I32 bytesAct;
        I32 bytesDes;
      } EventEntry_I2C_ReadError;

      //! The history of I2C_ReadError events
      //!
      History<EventEntry_I2C_ReadError> 
        *eventHistory_I2C_ReadError;

    protected:

      // ----------------------------------------------------------------------
      // Event: I2C_BadReadWriteCall
      // ----------------------------------------------------------------------

      //! Handle event I2C_BadReadWriteCall
      //!
      virtual void logIn_WARNING_HI_I2C_BadReadWriteCall(
          I32 device /*!< The device*/
      );

      //! A history entry for event I2C_BadReadWriteCall
      //!
      typedef struct {
        I32 device;
      } EventEntry_I2C_BadReadWriteCall;

      //! The history of I2C_BadReadWriteCall events
      //!
      History<EventEntry_I2C_BadReadWriteCall> 
        *eventHistory_I2C_BadReadWriteCall;

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
      // Channel: I2C_ReadBytes
      // ----------------------------------------------------------------------

      //! Handle channel I2C_ReadBytes
      //!
      virtual void tlmInput_I2C_ReadBytes(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel I2C_ReadBytes
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_I2C_ReadBytes;

      //! The history of I2C_ReadBytes values
      //!
      History<TlmEntry_I2C_ReadBytes> 
        *tlmHistory_I2C_ReadBytes;

    protected:

      // ----------------------------------------------------------------------
      // Channel: I2C_WriteBytes
      // ----------------------------------------------------------------------

      //! Handle channel I2C_WriteBytes
      //!
      virtual void tlmInput_I2C_WriteBytes(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel I2C_WriteBytes
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_I2C_WriteBytes;

      //! The history of I2C_WriteBytes values
      //!
      History<TlmEntry_I2C_WriteBytes> 
        *tlmHistory_I2C_WriteBytes;

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

      //! To port connected to I2CReadWrite
      //!
      Drv::OutputI2CWriteReadPort m_to_I2CReadWrite[1];

      //! To port connected to I2CConfig
      //!
      Drv::OutputI2CConfigPort m_to_I2CConfig[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

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

      //! From port connected to Time
      //!
      Fw::InputTimePort m_from_Time[1];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

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

      //! Static function for port from_Time
      //!
      static void from_Time_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      );

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
