// ======================================================================
// \title  SnapdragonHealth/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for SnapdragonHealth component test harness base class
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

#ifndef SnapdragonHealth_TESTER_BASE_HPP
#define SnapdragonHealth_TESTER_BASE_HPP

#include <SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace SnapdragonFlight {

  //! \class SnapdragonHealthTesterBase
  //! \brief Auto-generated base class for SnapdragonHealth component test harness
  //!
  class SnapdragonHealthTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object SnapdragonHealthTesterBase
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

      //! Get the port that receives input from PowerReport
      //!
      //! \return from_PowerReport[portNum]
      //!
      Svc::InputPolyPort* get_from_PowerReport(
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

      //! Construct object SnapdragonHealthTesterBase
      //!
      SnapdragonHealthTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object SnapdragonHealthTesterBase
      //!
      virtual ~SnapdragonHealthTesterBase(void);

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

      //! Handler prototype for from_PowerReport
      //!
      virtual void from_PowerReport_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
      ) = 0;

      //! Handler base function for from_PowerReport
      //!
      void from_PowerReport_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
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

      //! Push an entry on the history for from_PowerReport
      void pushFromPortEntry_PowerReport(
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
      );

      //! A history entry for from_PowerReport
      //!
      typedef struct {
        U32 entry;
        Svc::MeasurementStatus status;
        Fw::Time time;
        Fw::PolyType val;
      } FromPortEntry_PowerReport;

      //! The history for from_PowerReport
      //!
      History<FromPortEntry_PowerReport> 
        *fromPortHistory_PowerReport;

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

      //! Invoke the to port connected to SchedIn
      //!
      void invoke_to_SchedIn(
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

      //! Get the number of to_SchedIn ports
      //!
      //! \return The number of to_SchedIn ports
      //!
      NATIVE_INT_TYPE getNum_to_SchedIn(void) const;

      //! Get the number of from_PowerReport ports
      //!
      //! \return The number of from_PowerReport ports
      //!
      NATIVE_INT_TYPE getNum_from_PowerReport(void) const;

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
      //! Whether to_SchedIn[portNum] is connected
      //!
      bool isConnected_to_SchedIn(
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

      //! Send a SnapdragonHealth_SetPowerSaver command
      //!
      void sendCmd_SnapdragonHealth_SetPowerSaver(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq, /*!< The command sequence number*/
          SnapdragonHealthComponentBase::PowerSaverMode Mode 
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
      // Event: SH_FileOpenError
      // ----------------------------------------------------------------------

      //! Handle event SH_FileOpenError
      //!
      virtual void logIn_WARNING_LO_SH_FileOpenError(
          I32 type, /*!< File type*/
          I32 stat /*!< File error status*/
      );

      //! A history entry for event SH_FileOpenError
      //!
      typedef struct {
        I32 type;
        I32 stat;
      } EventEntry_SH_FileOpenError;

      //! The history of SH_FileOpenError events
      //!
      History<EventEntry_SH_FileOpenError> 
        *eventHistory_SH_FileOpenError;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_FileReadError
      // ----------------------------------------------------------------------

      //! Handle event SH_FileReadError
      //!
      virtual void logIn_WARNING_LO_SH_FileReadError(
          I32 type, /*!< File type*/
          I32 stat /*!< File error status*/
      );

      //! A history entry for event SH_FileReadError
      //!
      typedef struct {
        I32 type;
        I32 stat;
      } EventEntry_SH_FileReadError;

      //! The history of SH_FileReadError events
      //!
      History<EventEntry_SH_FileReadError> 
        *eventHistory_SH_FileReadError;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_FileWriteError
      // ----------------------------------------------------------------------

      //! Handle event SH_FileWriteError
      //!
      virtual void logIn_WARNING_HI_SH_FileWriteError(
          I32 type, /*!< File type*/
          I32 stat /*!< File error status*/
      );

      //! A history entry for event SH_FileWriteError
      //!
      typedef struct {
        I32 type;
        I32 stat;
      } EventEntry_SH_FileWriteError;

      //! The history of SH_FileWriteError events
      //!
      History<EventEntry_SH_FileWriteError> 
        *eventHistory_SH_FileWriteError;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_FswVersion
      // ----------------------------------------------------------------------

      //! Handle event SH_FswVersion
      //!
      virtual void logIn_ACTIVITY_HI_SH_FswVersion(
          U32 hash, /*!< FSW GIT hash*/
          Fw::LogStringArg& branch /*!< FSW GIT branch*/
      );

      //! A history entry for event SH_FswVersion
      //!
      typedef struct {
        U32 hash;
        Fw::LogStringArg branch;
      } EventEntry_SH_FswVersion;

      //! The history of SH_FswVersion events
      //!
      History<EventEntry_SH_FswVersion> 
        *eventHistory_SH_FswVersion;

    protected:

      // ----------------------------------------------------------------------
      // Event: SH_PowerSaver
      // ----------------------------------------------------------------------

      //! Handle event SH_PowerSaver
      //!
      virtual void logIn_ACTIVITY_HI_SH_PowerSaver(
          SnapdragonHealthComponentBase::PowerSaverModeEvr Mode 
      );

      //! A history entry for event SH_PowerSaver
      //!
      typedef struct {
        SnapdragonHealthComponentBase::PowerSaverModeEvr Mode;
      } EventEntry_SH_PowerSaver;

      //! The history of SH_PowerSaver events
      //!
      History<EventEntry_SH_PowerSaver> 
        *eventHistory_SH_PowerSaver;

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
      // Channel: SnapTempZone0DspCpuStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone0DspCpuStat
      //!
      virtual void tlmInput_SnapTempZone0DspCpuStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone0DspCpuStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone0DspCpuStat;

      //! The history of SnapTempZone0DspCpuStat values
      //!
      History<TlmEntry_SnapTempZone0DspCpuStat> 
        *tlmHistory_SnapTempZone0DspCpuStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone0DspCpuTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone0DspCpuTemp
      //!
      virtual void tlmInput_SnapTempZone0DspCpuTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone0DspCpuTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone0DspCpuTemp;

      //! The history of SnapTempZone0DspCpuTemp values
      //!
      History<TlmEntry_SnapTempZone0DspCpuTemp> 
        *tlmHistory_SnapTempZone0DspCpuTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone1DspModemStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone1DspModemStat
      //!
      virtual void tlmInput_SnapTempZone1DspModemStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone1DspModemStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone1DspModemStat;

      //! The history of SnapTempZone1DspModemStat values
      //!
      History<TlmEntry_SnapTempZone1DspModemStat> 
        *tlmHistory_SnapTempZone1DspModemStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone1DspModemTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone1DspModemTemp
      //!
      virtual void tlmInput_SnapTempZone1DspModemTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone1DspModemTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone1DspModemTemp;

      //! The history of SnapTempZone1DspModemTemp values
      //!
      History<TlmEntry_SnapTempZone1DspModemTemp> 
        *tlmHistory_SnapTempZone1DspModemTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone2ModemStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone2ModemStat
      //!
      virtual void tlmInput_SnapTempZone2ModemStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone2ModemStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone2ModemStat;

      //! The history of SnapTempZone2ModemStat values
      //!
      History<TlmEntry_SnapTempZone2ModemStat> 
        *tlmHistory_SnapTempZone2ModemStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone2ModemTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone2ModemTemp
      //!
      virtual void tlmInput_SnapTempZone2ModemTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone2ModemTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone2ModemTemp;

      //! The history of SnapTempZone2ModemTemp values
      //!
      History<TlmEntry_SnapTempZone2ModemTemp> 
        *tlmHistory_SnapTempZone2ModemTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone3VideoStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone3VideoStat
      //!
      virtual void tlmInput_SnapTempZone3VideoStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone3VideoStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone3VideoStat;

      //! The history of SnapTempZone3VideoStat values
      //!
      History<TlmEntry_SnapTempZone3VideoStat> 
        *tlmHistory_SnapTempZone3VideoStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone3VideoTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone3VideoTemp
      //!
      virtual void tlmInput_SnapTempZone3VideoTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone3VideoTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone3VideoTemp;

      //! The history of SnapTempZone3VideoTemp values
      //!
      History<TlmEntry_SnapTempZone3VideoTemp> 
        *tlmHistory_SnapTempZone3VideoTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone4WlanCpuStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone4WlanCpuStat
      //!
      virtual void tlmInput_SnapTempZone4WlanCpuStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone4WlanCpuStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone4WlanCpuStat;

      //! The history of SnapTempZone4WlanCpuStat values
      //!
      History<TlmEntry_SnapTempZone4WlanCpuStat> 
        *tlmHistory_SnapTempZone4WlanCpuStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone4WlanCpuTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone4WlanCpuTemp
      //!
      virtual void tlmInput_SnapTempZone4WlanCpuTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone4WlanCpuTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone4WlanCpuTemp;

      //! The history of SnapTempZone4WlanCpuTemp values
      //!
      History<TlmEntry_SnapTempZone4WlanCpuTemp> 
        *tlmHistory_SnapTempZone4WlanCpuTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone5Cpu0Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone5Cpu0Stat
      //!
      virtual void tlmInput_SnapTempZone5Cpu0Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone5Cpu0Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone5Cpu0Stat;

      //! The history of SnapTempZone5Cpu0Stat values
      //!
      History<TlmEntry_SnapTempZone5Cpu0Stat> 
        *tlmHistory_SnapTempZone5Cpu0Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone5Cpu0Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone5Cpu0Temp
      //!
      virtual void tlmInput_SnapTempZone5Cpu0Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone5Cpu0Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone5Cpu0Temp;

      //! The history of SnapTempZone5Cpu0Temp values
      //!
      History<TlmEntry_SnapTempZone5Cpu0Temp> 
        *tlmHistory_SnapTempZone5Cpu0Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone6Cpu1Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone6Cpu1Stat
      //!
      virtual void tlmInput_SnapTempZone6Cpu1Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone6Cpu1Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone6Cpu1Stat;

      //! The history of SnapTempZone6Cpu1Stat values
      //!
      History<TlmEntry_SnapTempZone6Cpu1Stat> 
        *tlmHistory_SnapTempZone6Cpu1Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone6Cpu1Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone6Cpu1Temp
      //!
      virtual void tlmInput_SnapTempZone6Cpu1Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone6Cpu1Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone6Cpu1Temp;

      //! The history of SnapTempZone6Cpu1Temp values
      //!
      History<TlmEntry_SnapTempZone6Cpu1Temp> 
        *tlmHistory_SnapTempZone6Cpu1Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone7Cpu2Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone7Cpu2Stat
      //!
      virtual void tlmInput_SnapTempZone7Cpu2Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone7Cpu2Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone7Cpu2Stat;

      //! The history of SnapTempZone7Cpu2Stat values
      //!
      History<TlmEntry_SnapTempZone7Cpu2Stat> 
        *tlmHistory_SnapTempZone7Cpu2Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone7Cpu2Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone7Cpu2Temp
      //!
      virtual void tlmInput_SnapTempZone7Cpu2Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone7Cpu2Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone7Cpu2Temp;

      //! The history of SnapTempZone7Cpu2Temp values
      //!
      History<TlmEntry_SnapTempZone7Cpu2Temp> 
        *tlmHistory_SnapTempZone7Cpu2Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone8Cpu3Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone8Cpu3Stat
      //!
      virtual void tlmInput_SnapTempZone8Cpu3Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone8Cpu3Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone8Cpu3Stat;

      //! The history of SnapTempZone8Cpu3Stat values
      //!
      History<TlmEntry_SnapTempZone8Cpu3Stat> 
        *tlmHistory_SnapTempZone8Cpu3Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone8Cpu3Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone8Cpu3Temp
      //!
      virtual void tlmInput_SnapTempZone8Cpu3Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone8Cpu3Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone8Cpu3Temp;

      //! The history of SnapTempZone8Cpu3Temp values
      //!
      History<TlmEntry_SnapTempZone8Cpu3Temp> 
        *tlmHistory_SnapTempZone8Cpu3Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone9GpuAStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone9GpuAStat
      //!
      virtual void tlmInput_SnapTempZone9GpuAStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone9GpuAStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone9GpuAStat;

      //! The history of SnapTempZone9GpuAStat values
      //!
      History<TlmEntry_SnapTempZone9GpuAStat> 
        *tlmHistory_SnapTempZone9GpuAStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone9GpuATemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone9GpuATemp
      //!
      virtual void tlmInput_SnapTempZone9GpuATemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone9GpuATemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone9GpuATemp;

      //! The history of SnapTempZone9GpuATemp values
      //!
      History<TlmEntry_SnapTempZone9GpuATemp> 
        *tlmHistory_SnapTempZone9GpuATemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone10GpuBStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone10GpuBStat
      //!
      virtual void tlmInput_SnapTempZone10GpuBStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone10GpuBStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone10GpuBStat;

      //! The history of SnapTempZone10GpuBStat values
      //!
      History<TlmEntry_SnapTempZone10GpuBStat> 
        *tlmHistory_SnapTempZone10GpuBStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone10GpuBTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone10GpuBTemp
      //!
      virtual void tlmInput_SnapTempZone10GpuBTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone10GpuBTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone10GpuBTemp;

      //! The history of SnapTempZone10GpuBTemp values
      //!
      History<TlmEntry_SnapTempZone10GpuBTemp> 
        *tlmHistory_SnapTempZone10GpuBTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonTotalCpuUtil
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonTotalCpuUtil
      //!
      virtual void tlmInput_SnapdragonTotalCpuUtil(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonTotalCpuUtil
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_SnapdragonTotalCpuUtil;

      //! The history of SnapdragonTotalCpuUtil values
      //!
      History<TlmEntry_SnapdragonTotalCpuUtil> 
        *tlmHistory_SnapdragonTotalCpuUtil;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu0Util
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu0Util
      //!
      virtual void tlmInput_SnapdragonCpu0Util(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu0Util
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_SnapdragonCpu0Util;

      //! The history of SnapdragonCpu0Util values
      //!
      History<TlmEntry_SnapdragonCpu0Util> 
        *tlmHistory_SnapdragonCpu0Util;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu1Util
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu1Util
      //!
      virtual void tlmInput_SnapdragonCpu1Util(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu1Util
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_SnapdragonCpu1Util;

      //! The history of SnapdragonCpu1Util values
      //!
      History<TlmEntry_SnapdragonCpu1Util> 
        *tlmHistory_SnapdragonCpu1Util;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu2Util
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu2Util
      //!
      virtual void tlmInput_SnapdragonCpu2Util(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu2Util
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_SnapdragonCpu2Util;

      //! The history of SnapdragonCpu2Util values
      //!
      History<TlmEntry_SnapdragonCpu2Util> 
        *tlmHistory_SnapdragonCpu2Util;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu3Util
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu3Util
      //!
      virtual void tlmInput_SnapdragonCpu3Util(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu3Util
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_SnapdragonCpu3Util;

      //! The history of SnapdragonCpu3Util values
      //!
      History<TlmEntry_SnapdragonCpu3Util> 
        *tlmHistory_SnapdragonCpu3Util;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu0Freq
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu0Freq
      //!
      virtual void tlmInput_SnapdragonCpu0Freq(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu0Freq
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_SnapdragonCpu0Freq;

      //! The history of SnapdragonCpu0Freq values
      //!
      History<TlmEntry_SnapdragonCpu0Freq> 
        *tlmHistory_SnapdragonCpu0Freq;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu1Freq
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu1Freq
      //!
      virtual void tlmInput_SnapdragonCpu1Freq(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu1Freq
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_SnapdragonCpu1Freq;

      //! The history of SnapdragonCpu1Freq values
      //!
      History<TlmEntry_SnapdragonCpu1Freq> 
        *tlmHistory_SnapdragonCpu1Freq;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu2Freq
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu2Freq
      //!
      virtual void tlmInput_SnapdragonCpu2Freq(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu2Freq
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_SnapdragonCpu2Freq;

      //! The history of SnapdragonCpu2Freq values
      //!
      History<TlmEntry_SnapdragonCpu2Freq> 
        *tlmHistory_SnapdragonCpu2Freq;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonCpu3Freq
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonCpu3Freq
      //!
      virtual void tlmInput_SnapdragonCpu3Freq(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonCpu3Freq
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_SnapdragonCpu3Freq;

      //! The history of SnapdragonCpu3Freq values
      //!
      History<TlmEntry_SnapdragonCpu3Freq> 
        *tlmHistory_SnapdragonCpu3Freq;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonPowerSaver
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonPowerSaver
      //!
      virtual void tlmInput_SnapdragonPowerSaver(
          const Fw::Time& timeTag, /*!< The time*/
          const SnapdragonHealthComponentBase::PowerSaverModeTlm& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonPowerSaver
      //!
      typedef struct {
        Fw::Time timeTag;
        SnapdragonHealthComponentBase::PowerSaverModeTlm arg;
      } TlmEntry_SnapdragonPowerSaver;

      //! The history of SnapdragonPowerSaver values
      //!
      History<TlmEntry_SnapdragonPowerSaver> 
        *tlmHistory_SnapdragonPowerSaver;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapdragonBootCount
      // ----------------------------------------------------------------------

      //! Handle channel SnapdragonBootCount
      //!
      virtual void tlmInput_SnapdragonBootCount(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapdragonBootCount
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_SnapdragonBootCount;

      //! The history of SnapdragonBootCount values
      //!
      History<TlmEntry_SnapdragonBootCount> 
        *tlmHistory_SnapdragonBootCount;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone11PM8841Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone11PM8841Stat
      //!
      virtual void tlmInput_SnapTempZone11PM8841Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone11PM8841Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone11PM8841Stat;

      //! The history of SnapTempZone11PM8841Stat values
      //!
      History<TlmEntry_SnapTempZone11PM8841Stat> 
        *tlmHistory_SnapTempZone11PM8841Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone11PM8841Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone11PM8841Temp
      //!
      virtual void tlmInput_SnapTempZone11PM8841Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone11PM8841Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_SnapTempZone11PM8841Temp;

      //! The history of SnapTempZone11PM8841Temp values
      //!
      History<TlmEntry_SnapTempZone11PM8841Temp> 
        *tlmHistory_SnapTempZone11PM8841Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone12PM8941Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone12PM8941Stat
      //!
      virtual void tlmInput_SnapTempZone12PM8941Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone12PM8941Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone12PM8941Stat;

      //! The history of SnapTempZone12PM8941Stat values
      //!
      History<TlmEntry_SnapTempZone12PM8941Stat> 
        *tlmHistory_SnapTempZone12PM8941Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone12PM8941Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone12PM8941Temp
      //!
      virtual void tlmInput_SnapTempZone12PM8941Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const F32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone12PM8941Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        F32 arg;
      } TlmEntry_SnapTempZone12PM8941Temp;

      //! The history of SnapTempZone12PM8941Temp values
      //!
      History<TlmEntry_SnapTempZone12PM8941Temp> 
        *tlmHistory_SnapTempZone12PM8941Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone13PA0Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone13PA0Stat
      //!
      virtual void tlmInput_SnapTempZone13PA0Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone13PA0Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone13PA0Stat;

      //! The history of SnapTempZone13PA0Stat values
      //!
      History<TlmEntry_SnapTempZone13PA0Stat> 
        *tlmHistory_SnapTempZone13PA0Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone13PA0Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone13PA0Temp
      //!
      virtual void tlmInput_SnapTempZone13PA0Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone13PA0Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone13PA0Temp;

      //! The history of SnapTempZone13PA0Temp values
      //!
      History<TlmEntry_SnapTempZone13PA0Temp> 
        *tlmHistory_SnapTempZone13PA0Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone14PA1Stat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone14PA1Stat
      //!
      virtual void tlmInput_SnapTempZone14PA1Stat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone14PA1Stat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone14PA1Stat;

      //! The history of SnapTempZone14PA1Stat values
      //!
      History<TlmEntry_SnapTempZone14PA1Stat> 
        *tlmHistory_SnapTempZone14PA1Stat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone14PA1Temp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone14PA1Temp
      //!
      virtual void tlmInput_SnapTempZone14PA1Temp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone14PA1Temp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone14PA1Temp;

      //! The history of SnapTempZone14PA1Temp values
      //!
      History<TlmEntry_SnapTempZone14PA1Temp> 
        *tlmHistory_SnapTempZone14PA1Temp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone15EmmcStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone15EmmcStat
      //!
      virtual void tlmInput_SnapTempZone15EmmcStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone15EmmcStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone15EmmcStat;

      //! The history of SnapTempZone15EmmcStat values
      //!
      History<TlmEntry_SnapTempZone15EmmcStat> 
        *tlmHistory_SnapTempZone15EmmcStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone15EmmcTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone15EmmcTemp
      //!
      virtual void tlmInput_SnapTempZone15EmmcTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone15EmmcTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone15EmmcTemp;

      //! The history of SnapTempZone15EmmcTemp values
      //!
      History<TlmEntry_SnapTempZone15EmmcTemp> 
        *tlmHistory_SnapTempZone15EmmcTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone16MsmStat
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone16MsmStat
      //!
      virtual void tlmInput_SnapTempZone16MsmStat(
          const Fw::Time& timeTag, /*!< The time*/
          const bool& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone16MsmStat
      //!
      typedef struct {
        Fw::Time timeTag;
        bool arg;
      } TlmEntry_SnapTempZone16MsmStat;

      //! The history of SnapTempZone16MsmStat values
      //!
      History<TlmEntry_SnapTempZone16MsmStat> 
        *tlmHistory_SnapTempZone16MsmStat;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SnapTempZone16MsmTemp
      // ----------------------------------------------------------------------

      //! Handle channel SnapTempZone16MsmTemp
      //!
      virtual void tlmInput_SnapTempZone16MsmTemp(
          const Fw::Time& timeTag, /*!< The time*/
          const I32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SnapTempZone16MsmTemp
      //!
      typedef struct {
        Fw::Time timeTag;
        I32 arg;
      } TlmEntry_SnapTempZone16MsmTemp;

      //! The history of SnapTempZone16MsmTemp values
      //!
      History<TlmEntry_SnapTempZone16MsmTemp> 
        *tlmHistory_SnapTempZone16MsmTemp;

    protected:

      // ----------------------------------------------------------------------
      // Channel: SH_FswVersion
      // ----------------------------------------------------------------------

      //! Handle channel SH_FswVersion
      //!
      virtual void tlmInput_SH_FswVersion(
          const Fw::Time& timeTag, /*!< The time*/
          const U32& val /*!< The channel value*/
      );

      //! A telemetry entry for channel SH_FswVersion
      //!
      typedef struct {
        Fw::Time timeTag;
        U32 arg;
      } TlmEntry_SH_FswVersion;

      //! The history of SH_FswVersion values
      //!
      History<TlmEntry_SH_FswVersion> 
        *tlmHistory_SH_FswVersion;

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

      //! From port connected to PowerReport
      //!
      Svc::InputPolyPort m_from_PowerReport[1];

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

      //! Static function for port from_PowerReport
      //!
      static void from_PowerReport_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 entry, /*!< The entry to access*/
          Svc::MeasurementStatus &status, /*!< The command response argument*/
          Fw::Time &time, /*!< The time of the measurement*/
          Fw::PolyType &val /*!< The value to be passed*/
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
