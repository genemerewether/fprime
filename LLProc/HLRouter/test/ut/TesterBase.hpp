// ======================================================================
// \title  HLRouter/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for HLRouter component test harness base class
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

#ifndef HLRouter_TESTER_BASE_HPP
#define HLRouter_TESTER_BASE_HPP

#include <LLProc/HLRouter/HLRouterComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace LLProc {

  //! \class HLRouterTesterBase
  //! \brief Auto-generated base class for HLRouter component test harness
  //!
  class HLRouterTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object HLRouterTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect LLPortsIn to to_LLPortsIn[portNum]
      //!
      void connect_to_LLPortsIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputSerializePort *const LLPortsIn /*!< The port*/
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

      //! Get the port that receives input from HLPortsOut
      //!
      //! \return from_HLPortsOut[portNum]
      //!
      Fw::InputSerializePort* get_from_HLPortsOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from SerReadPort
      //!
      //! \return from_SerReadPort[portNum]
      //!
      Drv::InputSerialReadPort* get_from_SerReadPort(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from SerWritePort
      //!
      //! \return from_SerWritePort[portNum]
      //!
      Drv::InputSerialWritePort* get_from_SerWritePort(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Get the port that receives input from DebugOut
      //!
      //! \return from_DebugOut[portNum]
      //!
      LLProc::InputDebugStringPort* get_from_DebugOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object HLRouterTesterBase
      //!
      HLRouterTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object HLRouterTesterBase
      //!
      virtual ~HLRouterTesterBase(void);

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

      //! Handler prototype for from_SerReadPort
      //!
      virtual void from_SerReadPort_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      ) = 0;

      //! Handler base function for from_SerReadPort
      //!
      void from_SerReadPort_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

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

      //! Handler prototype for from_DebugOut
      //!
      virtual void from_DebugOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U8 *string, /*!< string*/
          U32 size /*!< size of string*/
      ) = 0;

      //! Handler base function for from_DebugOut
      //!
      void from_DebugOut_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U8 *string, /*!< string*/
          U32 size /*!< size of string*/
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

      //! Push an entry on the history for from_SerReadPort
      void pushFromPortEntry_SerReadPort(
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

      //! A history entry for from_SerReadPort
      //!
      typedef struct {
        Fw::Buffer serBuffer;
        Drv::SerialReadStatus status;
      } FromPortEntry_SerReadPort;

      //! The history for from_SerReadPort
      //!
      History<FromPortEntry_SerReadPort> 
        *fromPortHistory_SerReadPort;

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

      //! Push an entry on the history for from_DebugOut
      void pushFromPortEntry_DebugOut(
          U8 *string, /*!< string*/
          U32 size /*!< size of string*/
      );

      //! A history entry for from_DebugOut
      //!
      typedef struct {
        U8 *string;
        U32 size;
      } FromPortEntry_DebugOut;

      //! The history for from_DebugOut
      //!
      History<FromPortEntry_DebugOut> 
        *fromPortHistory_DebugOut;

    protected:

      // ----------------------------------------------------------------------
      // Handler prototypes for serial from ports
      // ----------------------------------------------------------------------

      //! Handler prototype for from_HLPortsOut
      //!
      virtual void from_HLPortsOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      ) = 0;

      //! Handler base class function prototype for from_HLPortsOut
      //!
      void from_HLPortsOut_handlerBase (
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );


    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to LLPortsIn
      //!
      void invoke_to_LLPortsIn(
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

      //! Get the number of to_LLPortsIn ports
      //!
      //! \return The number of to_LLPortsIn ports
      //!
      NATIVE_INT_TYPE getNum_to_LLPortsIn(void) const;

      //! Get the number of from_HLPortsOut ports
      //!
      //! \return The number of from_HLPortsOut ports
      //!
      NATIVE_INT_TYPE getNum_from_HLPortsOut(void) const;

      //! Get the number of from_SerReadPort ports
      //!
      //! \return The number of from_SerReadPort ports
      //!
      NATIVE_INT_TYPE getNum_from_SerReadPort(void) const;

      //! Get the number of from_SerWritePort ports
      //!
      //! \return The number of from_SerWritePort ports
      //!
      NATIVE_INT_TYPE getNum_from_SerWritePort(void) const;

      //! Get the number of to_Sched ports
      //!
      //! \return The number of to_Sched ports
      //!
      NATIVE_INT_TYPE getNum_to_Sched(void) const;

      //! Get the number of from_DebugOut ports
      //!
      //! \return The number of from_DebugOut ports
      //!
      NATIVE_INT_TYPE getNum_from_DebugOut(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_LLPortsIn[portNum] is connected
      //!
      bool isConnected_to_LLPortsIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_Sched[portNum] is connected
      //!
      bool isConnected_to_Sched(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to LLPortsIn
      //!
      Fw::OutputSerializePort m_to_LLPortsIn[25];

      //! To port connected to Sched
      //!
      Svc::OutputSchedPort m_to_Sched[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to HLPortsOut
      //!
      Fw::InputSerializePort m_from_HLPortsOut[25];

      //! From port connected to SerReadPort
      //!
      Drv::InputSerialReadPort m_from_SerReadPort[1];

      //! From port connected to SerWritePort
      //!
      Drv::InputSerialWritePort m_from_SerWritePort[1];

      //! From port connected to DebugOut
      //!
      LLProc::InputDebugStringPort m_from_DebugOut[1];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_HLPortsOut
      //!
      static void from_HLPortsOut_static(
          Fw::PassiveComponentBase *const callComp, //!< The component instance
          const NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer //!< serialized data buffer
      );

      //! Static function for port from_SerReadPort
      //!
      static void from_SerReadPort_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

      //! Static function for port from_SerWritePort
      //!
      static void from_SerWritePort_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      );

      //! Static function for port from_DebugOut
      //!
      static void from_DebugOut_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U8 *string, /*!< string*/
          U32 size /*!< size of string*/
      );

  };

} // end namespace LLProc

#endif
