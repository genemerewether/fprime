// ======================================================================
// \title  GPSPosAdapter/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for GPSPosAdapter component test harness base class
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

#ifndef GPSPosAdapter_TESTER_BASE_HPP
#define GPSPosAdapter_TESTER_BASE_HPP

#include <Drv/Mavlink/GPSPosAdapter/GPSPosAdapterComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Drv {

  //! \class GPSPosAdapterTesterBase
  //! \brief Auto-generated base class for GPSPosAdapter component test harness
  //!
  class GPSPosAdapterTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object GPSPosAdapterTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect Guid to to_Guid[portNum]
      //!
      void connect_to_Guid(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::InputFlatOutputPort *const Guid /*!< The port*/
      );

      //! Connect Nav to to_Nav[portNum]
      //!
      void connect_to_Nav(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::InputFlatOutputPort *const Nav /*!< The port*/
      );

      //! Connect sched to to_sched[portNum]
      //!
      void connect_to_sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputSchedPort *const sched /*!< The port*/
      );

      //! Connect SerReadPort to to_SerReadPort[portNum]
      //!
      void connect_to_SerReadPort(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputSerialReadPort *const SerReadPort /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from SerWritePort
      //!
      //! \return from_SerWritePort[portNum]
      //!
      Drv::InputSerialWritePort* get_from_SerWritePort(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object GPSPosAdapterTesterBase
      //!
      GPSPosAdapterTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object GPSPosAdapterTesterBase
      //!
      virtual ~GPSPosAdapterTesterBase(void);

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

      //! Handler prototype for from_SerWritePort
      //!
      virtual void from_SerWritePort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      ) = 0;

      //! Handler base function for from_SerWritePort
      //!
      void from_SerWritePort_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
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

      //! Push an entry on the history for from_SerWritePort
      void pushFromPortEntry_SerWritePort(
          Fw::Buffer &serBuffer 
      );

      //! A history entry for from_SerWritePort
      //!
      typedef struct {
        Fw::Buffer serBuffer;
      } FromPortEntry_SerWritePort;

      //! The history for from_SerWritePort
      //!
      History<FromPortEntry_SerWritePort> 
        *fromPortHistory_SerWritePort;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to Guid
      //!
      void invoke_to_Guid(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
      );

      //! Invoke the to port connected to Nav
      //!
      void invoke_to_Nav(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::FlatOutput &FlatOutput 
      );

      //! Invoke the to port connected to sched
      //!
      void invoke_to_sched(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Invoke the to port connected to SerReadPort
      //!
      void invoke_to_SerReadPort(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          SerialReadStatus &status /*!< Status of read*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_Guid ports
      //!
      //! \return The number of to_Guid ports
      //!
      NATIVE_INT_TYPE getNum_to_Guid(void) const;

      //! Get the number of to_Nav ports
      //!
      //! \return The number of to_Nav ports
      //!
      NATIVE_INT_TYPE getNum_to_Nav(void) const;

      //! Get the number of to_sched ports
      //!
      //! \return The number of to_sched ports
      //!
      NATIVE_INT_TYPE getNum_to_sched(void) const;

      //! Get the number of to_SerReadPort ports
      //!
      //! \return The number of to_SerReadPort ports
      //!
      NATIVE_INT_TYPE getNum_to_SerReadPort(void) const;

      //! Get the number of from_SerWritePort ports
      //!
      //! \return The number of from_SerWritePort ports
      //!
      NATIVE_INT_TYPE getNum_from_SerWritePort(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_Guid[portNum] is connected
      //!
      bool isConnected_to_Guid(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_Nav[portNum] is connected
      //!
      bool isConnected_to_Nav(
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
      //! Whether to_SerReadPort[portNum] is connected
      //!
      bool isConnected_to_SerReadPort(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to Guid
      //!
      ROS::mav_msgs::OutputFlatOutputPort m_to_Guid[1];

      //! To port connected to Nav
      //!
      ROS::mav_msgs::OutputFlatOutputPort m_to_Nav[1];

      //! To port connected to sched
      //!
      Svc::OutputSchedPort m_to_sched[1];

      //! To port connected to SerReadPort
      //!
      Drv::OutputSerialReadPort m_to_SerReadPort[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to SerWritePort
      //!
      Drv::InputSerialWritePort m_from_SerWritePort[1];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_SerWritePort
      //!
      static void from_SerWritePort_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      );

  };

} // end namespace Drv

#endif
