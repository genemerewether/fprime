// ======================================================================
// \title  HLRosIface/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for HLRosIface component test harness base class
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

#ifndef HLRosIface_TESTER_BASE_HPP
#define HLRosIface_TESTER_BASE_HPP

#include <HLProc/HLRosIface/HLRosIfaceComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace HLProc {

  //! \class HLRosIfaceTesterBase
  //! \brief Auto-generated base class for HLRosIface component test harness
  //!
  class HLRosIfaceTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object HLRosIfaceTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect Imu to to_Imu[portNum]
      //!
      void connect_to_Imu(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::InputImuNoCovPort *const Imu /*!< The port*/
      );

      //! Connect sched to to_sched[portNum]
      //!
      void connect_to_sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const sched /*!< The port*/
      );

      //! Connect Odometry to to_Odometry[portNum]
      //!
      void connect_to_Odometry(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::nav_msgs::InputOdometryNoCovPort *const Odometry /*!< The port*/
      );

      //! Connect pingIn to to_pingIn[portNum]
      //!
      void connect_to_pingIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputPingPort *const pingIn /*!< The port*/
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

      //! Get the port that receives input from LogText
      //!
      //! \return from_LogText[portNum]
      //!
      Fw::InputLogTextPort* get_from_LogText(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from pingOut
      //!
      //! \return from_pingOut[portNum]
      //!
      Svc::InputPingPort* get_from_pingOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from Time
      //!
      //! \return from_Time[portNum]
      //!
      Fw::InputTimePort* get_from_Time(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from ImuStateUpdate
      //!
      //! \return from_ImuStateUpdate[portNum]
      //!
      ROS::mav_msgs::InputImuStateUpdatePort* get_from_ImuStateUpdate(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from ActuatorsData
      //!
      //! \return from_ActuatorsData[portNum]
      //!
      ROS::mav_msgs::InputActuatorsPort* get_from_ActuatorsData(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object HLRosIfaceTesterBase
      //!
      HLRosIfaceTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object HLRosIfaceTesterBase
      //!
      virtual ~HLRosIfaceTesterBase(void);

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

      //! Handler prototype for from_Tlm
      //!
      virtual void from_Tlm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwChanIdType id, /*!< Telemetry Channel ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TlmBuffer &val /*!< Buffer containing serialized telemetry value*/
      ) = 0;

      //! Handler base function for from_Tlm
      //!
      void from_Tlm_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwChanIdType id, /*!< Telemetry Channel ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TlmBuffer &val /*!< Buffer containing serialized telemetry value*/
      );

      //! Handler prototype for from_Log
      //!
      virtual void from_Log_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::LogSeverity severity, /*!< The severity argument*/
          Fw::LogBuffer &args /*!< Buffer containing serialized log entry*/
      ) = 0;

      //! Handler base function for from_Log
      //!
      void from_Log_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::LogSeverity severity, /*!< The severity argument*/
          Fw::LogBuffer &args /*!< Buffer containing serialized log entry*/
      );

      //! Handler prototype for from_LogText
      //!
      virtual void from_LogText_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TextLogSeverity severity, /*!< The severity argument*/
          Fw::TextLogString &text /*!< Text of log message*/
      ) = 0;

      //! Handler base function for from_LogText
      //!
      void from_LogText_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TextLogSeverity severity, /*!< The severity argument*/
          Fw::TextLogString &text /*!< Text of log message*/
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

      //! Handler prototype for from_Time
      //!
      virtual void from_Time_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      ) = 0;

      //! Handler base function for from_Time
      //!
      void from_Time_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      );

      //! Handler prototype for from_ImuStateUpdate
      //!
      virtual void from_ImuStateUpdate_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate 
      ) = 0;

      //! Handler base function for from_ImuStateUpdate
      //!
      void from_ImuStateUpdate_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate 
      );

      //! Handler prototype for from_ActuatorsData
      //!
      virtual void from_ActuatorsData_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::Actuators &Actuators 
      ) = 0;

      //! Handler base function for from_ActuatorsData
      //!
      void from_ActuatorsData_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::Actuators &Actuators 
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

      //! Push an entry on the history for from_Tlm
      void pushFromPortEntry_Tlm(
          FwChanIdType id, /*!< Telemetry Channel ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TlmBuffer &val /*!< Buffer containing serialized telemetry value*/
      );

      //! A history entry for from_Tlm
      //!
      typedef struct {
        FwChanIdType id;
        Fw::Time timeTag;
        Fw::TlmBuffer val;
      } FromPortEntry_Tlm;

      //! The history for from_Tlm
      //!
      History<FromPortEntry_Tlm> 
        *fromPortHistory_Tlm;

      //! Push an entry on the history for from_Log
      void pushFromPortEntry_Log(
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::LogSeverity severity, /*!< The severity argument*/
          Fw::LogBuffer &args /*!< Buffer containing serialized log entry*/
      );

      //! A history entry for from_Log
      //!
      typedef struct {
        FwEventIdType id;
        Fw::Time timeTag;
        Fw::LogSeverity severity;
        Fw::LogBuffer args;
      } FromPortEntry_Log;

      //! The history for from_Log
      //!
      History<FromPortEntry_Log> 
        *fromPortHistory_Log;

      //! Push an entry on the history for from_LogText
      void pushFromPortEntry_LogText(
          FwEventIdType id, /*!< Log ID*/
          Fw::Time &timeTag, /*!< Time Tag*/
          Fw::TextLogSeverity severity, /*!< The severity argument*/
          Fw::TextLogString &text /*!< Text of log message*/
      );

      //! A history entry for from_LogText
      //!
      typedef struct {
        FwEventIdType id;
        Fw::Time timeTag;
        Fw::TextLogSeverity severity;
        Fw::TextLogString text;
      } FromPortEntry_LogText;

      //! The history for from_LogText
      //!
      History<FromPortEntry_LogText> 
        *fromPortHistory_LogText;

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

      //! Push an entry on the history for from_Time
      void pushFromPortEntry_Time(
          Fw::Time &time /*!< The U32 cmd argument*/
      );

      //! A history entry for from_Time
      //!
      typedef struct {
        Fw::Time time;
      } FromPortEntry_Time;

      //! The history for from_Time
      //!
      History<FromPortEntry_Time> 
        *fromPortHistory_Time;

      //! Push an entry on the history for from_ImuStateUpdate
      void pushFromPortEntry_ImuStateUpdate(
          ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate 
      );

      //! A history entry for from_ImuStateUpdate
      //!
      typedef struct {
        ROS::mav_msgs::ImuStateUpdate ImuStateUpdate;
      } FromPortEntry_ImuStateUpdate;

      //! The history for from_ImuStateUpdate
      //!
      History<FromPortEntry_ImuStateUpdate> 
        *fromPortHistory_ImuStateUpdate;

      //! Push an entry on the history for from_ActuatorsData
      void pushFromPortEntry_ActuatorsData(
          ROS::mav_msgs::Actuators &Actuators 
      );

      //! A history entry for from_ActuatorsData
      //!
      typedef struct {
        ROS::mav_msgs::Actuators Actuators;
      } FromPortEntry_ActuatorsData;

      //! The history for from_ActuatorsData
      //!
      History<FromPortEntry_ActuatorsData> 
        *fromPortHistory_ActuatorsData;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to Imu
      //!
      void invoke_to_Imu(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::ImuNoCov &ImuNoCov 
      );

      //! Invoke the to port connected to sched
      //!
      void invoke_to_sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Invoke the to port connected to Odometry
      //!
      void invoke_to_Odometry(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::nav_msgs::OdometryNoCov &OdometryNoCov 
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

      //! Get the number of to_Imu ports
      //!
      //! \return The number of to_Imu ports
      //!
      NATIVE_INT_TYPE getNum_to_Imu(void) const;

      //! Get the number of from_Tlm ports
      //!
      //! \return The number of from_Tlm ports
      //!
      NATIVE_INT_TYPE getNum_from_Tlm(void) const;

      //! Get the number of to_sched ports
      //!
      //! \return The number of to_sched ports
      //!
      NATIVE_INT_TYPE getNum_to_sched(void) const;

      //! Get the number of from_Log ports
      //!
      //! \return The number of from_Log ports
      //!
      NATIVE_INT_TYPE getNum_from_Log(void) const;

      //! Get the number of from_LogText ports
      //!
      //! \return The number of from_LogText ports
      //!
      NATIVE_INT_TYPE getNum_from_LogText(void) const;

      //! Get the number of from_pingOut ports
      //!
      //! \return The number of from_pingOut ports
      //!
      NATIVE_INT_TYPE getNum_from_pingOut(void) const;

      //! Get the number of from_Time ports
      //!
      //! \return The number of from_Time ports
      //!
      NATIVE_INT_TYPE getNum_from_Time(void) const;

      //! Get the number of to_Odometry ports
      //!
      //! \return The number of to_Odometry ports
      //!
      NATIVE_INT_TYPE getNum_to_Odometry(void) const;

      //! Get the number of from_ImuStateUpdate ports
      //!
      //! \return The number of from_ImuStateUpdate ports
      //!
      NATIVE_INT_TYPE getNum_from_ImuStateUpdate(void) const;

      //! Get the number of from_ActuatorsData ports
      //!
      //! \return The number of from_ActuatorsData ports
      //!
      NATIVE_INT_TYPE getNum_from_ActuatorsData(void) const;

      //! Get the number of to_pingIn ports
      //!
      //! \return The number of to_pingIn ports
      //!
      NATIVE_INT_TYPE getNum_to_pingIn(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_Imu[portNum] is connected
      //!
      bool isConnected_to_Imu(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_sched[portNum] is connected
      //!
      bool isConnected_to_sched(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_Odometry[portNum] is connected
      //!
      bool isConnected_to_Odometry(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_pingIn[portNum] is connected
      //!
      bool isConnected_to_pingIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to Imu
      //!
      ROS::sensor_msgs::OutputImuNoCovPort m_to_Imu[1];

      //! To port connected to sched
      //!
      Svc::OutputSchedPort m_to_sched[1];

      //! To port connected to Odometry
      //!
      ROS::nav_msgs::OutputOdometryNoCovPort m_to_Odometry[1];

      //! To port connected to pingIn
      //!
      Svc::OutputPingPort m_to_pingIn[1];

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

      //! From port connected to LogText
      //!
      Fw::InputLogTextPort m_from_LogText[1];

      //! From port connected to pingOut
      //!
      Svc::InputPingPort m_from_pingOut[1];

      //! From port connected to Time
      //!
      Fw::InputTimePort m_from_Time[1];

      //! From port connected to ImuStateUpdate
      //!
      ROS::mav_msgs::InputImuStateUpdatePort m_from_ImuStateUpdate[1];

      //! From port connected to ActuatorsData
      //!
      ROS::mav_msgs::InputActuatorsPort m_from_ActuatorsData[2];

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

      //! Static function for port from_pingOut
      //!
      static void from_pingOut_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Static function for port from_Time
      //!
      static void from_Time_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time &time /*!< The U32 cmd argument*/
      );

      //! Static function for port from_ImuStateUpdate
      //!
      static void from_ImuStateUpdate_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::ImuStateUpdate &ImuStateUpdate 
      );

      //! Static function for port from_ActuatorsData
      //!
      static void from_ActuatorsData_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::Actuators &Actuators 
      );

  };

} // end namespace HLProc

#endif
