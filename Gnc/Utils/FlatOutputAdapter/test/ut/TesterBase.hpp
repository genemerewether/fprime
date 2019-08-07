// ======================================================================
// \title  FlatOutputAdapter/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for FlatOutputAdapter component test harness base class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef FlatOutputAdapter_TESTER_BASE_HPP
#define FlatOutputAdapter_TESTER_BASE_HPP

#include <Gnc/Utils/FlatOutputAdapter/FlatOutputAdapterComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Gnc {

  //! \class FlatOutputAdapterTesterBase
  //! \brief Auto-generated base class for FlatOutputAdapter component test harness
  //!
  class FlatOutputAdapterTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object FlatOutputAdapterTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect se3Cmd to to_se3Cmd[portNum]
      //!
      void connect_to_se3Cmd(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::InputSe3FeedForwardPort *const se3Cmd /*!< The port*/
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

      //! Get the port that receives input from flatOutput
      //!
      //! \return from_flatOutput[portNum]
      //!
      ROS::mav_msgs::InputFlatOutputPort* get_from_flatOutput(
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

      //! Get the port that receives input from ParamGet
      //!
      //! \return from_ParamGet[portNum]
      //!
      Fw::InputPrmGetPort* get_from_ParamGet(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from ParamSet
      //!
      //! \return from_ParamSet[portNum]
      //!
      Fw::InputPrmSetPort* get_from_ParamSet(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object FlatOutputAdapterTesterBase
      //!
      FlatOutputAdapterTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object FlatOutputAdapterTesterBase
      //!
      virtual ~FlatOutputAdapterTesterBase(void);

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

      //! Handler prototype for from_flatOutput
      //!
      virtual void from_flatOutput_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
      ) = 0;

      //! Handler base function for from_flatOutput
      //!
      void from_flatOutput_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
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

      //! Push an entry on the history for from_flatOutput
      void pushFromPortEntry_flatOutput(
          ROS::mav_msgs::FlatOutput &FlatOutput 
      );

      //! A history entry for from_flatOutput
      //!
      typedef struct {
        ROS::mav_msgs::FlatOutput FlatOutput;
      } FromPortEntry_flatOutput;

      //! The history for from_flatOutput
      //!
      History<FromPortEntry_flatOutput>
        *fromPortHistory_flatOutput;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to se3Cmd
      //!
      void invoke_to_se3Cmd(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::Se3FeedForward &Se3FeedForward 
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_se3Cmd ports
      //!
      //! \return The number of to_se3Cmd ports
      //!
      NATIVE_INT_TYPE getNum_to_se3Cmd(void) const;

      //! Get the number of from_flatOutput ports
      //!
      //! \return The number of from_flatOutput ports
      //!
      NATIVE_INT_TYPE getNum_from_flatOutput(void) const;

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

      //! Get the number of from_ParamGet ports
      //!
      //! \return The number of from_ParamGet ports
      //!
      NATIVE_INT_TYPE getNum_from_ParamGet(void) const;

      //! Get the number of from_ParamSet ports
      //!
      //! \return The number of from_ParamSet ports
      //!
      NATIVE_INT_TYPE getNum_from_ParamSet(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_se3Cmd[portNum] is connected
      //!
      bool isConnected_to_se3Cmd(
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
      // Parameter: w_x
      // ----------------------------------------------------------------------

      void paramSet_w_x(
          const F64& val, /*!< The parameter value*/
          Fw::ParamValid valid /*!< The parameter valid flag*/
      );

      void paramSend_w_x(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      void paramSave_w_x(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Parameter: w_y
      // ----------------------------------------------------------------------

      void paramSet_w_y(
          const F64& val, /*!< The parameter value*/
          Fw::ParamValid valid /*!< The parameter valid flag*/
      );

      void paramSend_w_y(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      void paramSave_w_y(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Parameter: w_z
      // ----------------------------------------------------------------------

      void paramSet_w_z(
          const F64& val, /*!< The parameter value*/
          Fw::ParamValid valid /*!< The parameter valid flag*/
      );

      void paramSend_w_z(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      void paramSave_w_z(
          const NATIVE_INT_TYPE instance, /*!< The instance number*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to se3Cmd
      //!
      ROS::mav_msgs::OutputSe3FeedForwardPort m_to_se3Cmd[1];

      //! To port connected to CmdDisp
      //!
      Fw::OutputCmdPort m_to_CmdDisp[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to flatOutput
      //!
      ROS::mav_msgs::InputFlatOutputPort m_from_flatOutput[1];

      //! From port connected to CmdStatus
      //!
      Fw::InputCmdResponsePort m_from_CmdStatus[1];

      //! From port connected to CmdReg
      //!
      Fw::InputCmdRegPort m_from_CmdReg[1];

      //! From port connected to ParamGet
      //!
      Fw::InputPrmGetPort m_from_ParamGet[1];

      //! From port connected to ParamSet
      //!
      Fw::InputPrmSetPort m_from_ParamSet[1];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_flatOutput
      //!
      static void from_flatOutput_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
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

      //! Static function for port from_ParamGet
      //!
      static Fw::ParamValid from_ParamGet_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, /*!< Parameter ID*/
          Fw::ParamBuffer &val /*!< Buffer containing serialized parameter value*/
      );

      //! Static function for port from_ParamSet
      //!
      static void from_ParamSet_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, /*!< Parameter ID*/
          Fw::ParamBuffer &val /*!< Buffer containing serialized parameter value*/
      );

    private:

      // ----------------------------------------------------------------------
      // Parameter validity flags
      // ----------------------------------------------------------------------

      //! True if parameter w_x was successfully received
      //!
      Fw::ParamValid m_param_w_x_valid;

      //! True if parameter w_y was successfully received
      //!
      Fw::ParamValid m_param_w_y_valid;

      //! True if parameter w_z was successfully received
      //!
      Fw::ParamValid m_param_w_z_valid;

    private:

      // ----------------------------------------------------------------------
      // Parameter variables
      // ----------------------------------------------------------------------

      //! Parameter w_x
      //!
      F64 m_param_w_x;

      //! Parameter w_y
      //!
      F64 m_param_w_y;

      //! Parameter w_z
      //!
      F64 m_param_w_z;

  };

} // end namespace Gnc

#endif
