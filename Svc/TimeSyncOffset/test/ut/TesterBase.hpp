// ======================================================================
// \title  TimeSyncOffset/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for TimeSyncOffset component test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef TimeSyncOffset_TESTER_BASE_HPP
#define TimeSyncOffset_TESTER_BASE_HPP

#include <Svc/TimeSyncOffset/TimeSyncOffsetComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Svc {

  //! \class TimeSyncOffsetTesterBase
  //! \brief Auto-generated base class for TimeSyncOffset component test harness
  //!
  class TimeSyncOffsetTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object TimeSyncOffsetTesterBase
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

      //! Connect LLTime to to_LLTime[portNum]
      //!
      void connect_to_LLTime(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputTimePort *const LLTime /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from GPIOPulse
      //!
      //! \return from_GPIOPulse[portNum]
      //!
      Drv::InputGpioWritePort* get_from_GPIOPulse(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from ClockTimes
      //!
      //! \return from_ClockTimes[portNum]
      //!
      Fw::InputTimePairPort* get_from_ClockTimes(
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

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object TimeSyncOffsetTesterBase
      //!
      TimeSyncOffsetTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object TimeSyncOffsetTesterBase
      //!
      virtual ~TimeSyncOffsetTesterBase(void);

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

      //! Handler prototype for from_GPIOPulse
      //!
      virtual void from_GPIOPulse_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool state 
      ) = 0;

      //! Handler base function for from_GPIOPulse
      //!
      void from_GPIOPulse_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool state 
      );

      //! Handler prototype for from_ClockTimes
      //!
      virtual void from_ClockTimes_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time time1, /*!< first time*/
          Fw::Time time2 /*!< second time*/
      ) = 0;

      //! Handler base function for from_ClockTimes
      //!
      void from_ClockTimes_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time time1, /*!< first time*/
          Fw::Time time2 /*!< second time*/
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

      //! Push an entry on the history for from_GPIOPulse
      void pushFromPortEntry_GPIOPulse(
          bool state 
      );

      //! A history entry for from_GPIOPulse
      //!
      typedef struct {
        bool state;
      } FromPortEntry_GPIOPulse;

      //! The history for from_GPIOPulse
      //!
      History<FromPortEntry_GPIOPulse>
        *fromPortHistory_GPIOPulse;

      //! Push an entry on the history for from_ClockTimes
      void pushFromPortEntry_ClockTimes(
          Fw::Time time1, /*!< first time*/
          Fw::Time time2 /*!< second time*/
      );

      //! A history entry for from_ClockTimes
      //!
      typedef struct {
        Fw::Time time1;
        Fw::Time time2;
      } FromPortEntry_ClockTimes;

      //! The history for from_ClockTimes
      //!
      History<FromPortEntry_ClockTimes>
        *fromPortHistory_ClockTimes;

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

      //! Invoke the to port connected to LLTime
      //!
      void invoke_to_LLTime(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
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

      //! Get the number of to_LLTime ports
      //!
      //! \return The number of to_LLTime ports
      //!
      NATIVE_INT_TYPE getNum_to_LLTime(void) const;

      //! Get the number of from_GPIOPulse ports
      //!
      //! \return The number of from_GPIOPulse ports
      //!
      NATIVE_INT_TYPE getNum_from_GPIOPulse(void) const;

      //! Get the number of from_ClockTimes ports
      //!
      //! \return The number of from_ClockTimes ports
      //!
      NATIVE_INT_TYPE getNum_from_ClockTimes(void) const;

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
      //! Whether to_LLTime[portNum] is connected
      //!
      bool isConnected_to_LLTime(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

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
      // Channel: LLTime
      // ----------------------------------------------------------------------

      //! Handle channel LLTime
      //!
      virtual void tlmInput_LLTime(
          const Fw::Time& timeTag, /*!< The time*/
          const F64& val /*!< The channel value*/
      );

      //! A telemetry entry for channel LLTime
      //!
      typedef struct {
        Fw::Time timeTag;
        F64 arg;
      } TlmEntry_LLTime;

      //! The history of LLTime values
      //!
      History<TlmEntry_LLTime>
        *tlmHistory_LLTime;

    protected:

      // ----------------------------------------------------------------------
      // Channel: HLTime
      // ----------------------------------------------------------------------

      //! Handle channel HLTime
      //!
      virtual void tlmInput_HLTime(
          const Fw::Time& timeTag, /*!< The time*/
          const F64& val /*!< The channel value*/
      );

      //! A telemetry entry for channel HLTime
      //!
      typedef struct {
        Fw::Time timeTag;
        F64 arg;
      } TlmEntry_HLTime;

      //! The history of HLTime values
      //!
      History<TlmEntry_HLTime>
        *tlmHistory_HLTime;

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

      //! To port connected to LLTime
      //!
      Fw::OutputTimePort m_to_LLTime[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to GPIOPulse
      //!
      Drv::InputGpioWritePort m_from_GPIOPulse[1];

      //! From port connected to ClockTimes
      //!
      Fw::InputTimePairPort m_from_ClockTimes[1];

      //! From port connected to Tlm
      //!
      Fw::InputTlmPort m_from_Tlm[1];

      //! From port connected to Time
      //!
      Fw::InputTimePort m_from_Time[1];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_GPIOPulse
      //!
      static void from_GPIOPulse_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool state 
      );

      //! Static function for port from_ClockTimes
      //!
      static void from_ClockTimes_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time time1, /*!< first time*/
          Fw::Time time2 /*!< second time*/
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
