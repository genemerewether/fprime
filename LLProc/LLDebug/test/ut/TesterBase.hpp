// ======================================================================
// \title  LLDebug/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for LLDebug component test harness base class
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

#ifndef LLDebug_TESTER_BASE_HPP
#define LLDebug_TESTER_BASE_HPP

#include <LLProc/LLDebug/LLDebugComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace LLProc {

  //! \class LLDebugTesterBase
  //! \brief Auto-generated base class for LLDebug component test harness
  //!
  class LLDebugTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object LLDebugTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect DebugStringIn to to_DebugStringIn[portNum]
      //!
      void connect_to_DebugStringIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          LLProc::InputDebugStringPort *const DebugStringIn /*!< The port*/
      );

      //! Connect AssertEnable to to_AssertEnable[portNum]
      //!
      void connect_to_AssertEnable(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Common::InputEnablePort *const AssertEnable /*!< The port*/
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

      //! Get the port that receives input from SwFault
      //!
      //! \return from_SwFault[portNum]
      //!
      R5::InputGpioSetPort* get_from_SwFault(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object LLDebugTesterBase
      //!
      LLDebugTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object LLDebugTesterBase
      //!
      virtual ~LLDebugTesterBase(void);

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

      //! Handler prototype for from_SwFault
      //!
      virtual void from_SwFault_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::GpioSetBank bank, 
          U32 bit, 
          R5::GpioSetVal val 
      ) = 0;

      //! Handler base function for from_SwFault
      //!
      void from_SwFault_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::GpioSetBank bank, 
          U32 bit, 
          R5::GpioSetVal val 
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

      //! Push an entry on the history for from_SwFault
      void pushFromPortEntry_SwFault(
          R5::GpioSetBank bank, 
          U32 bit, 
          R5::GpioSetVal val 
      );

      //! A history entry for from_SwFault
      //!
      typedef struct {
        R5::GpioSetBank bank;
        U32 bit;
        R5::GpioSetVal val;
      } FromPortEntry_SwFault;

      //! The history for from_SwFault
      //!
      History<FromPortEntry_SwFault> 
        *fromPortHistory_SwFault;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to DebugStringIn
      //!
      void invoke_to_DebugStringIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U8 *string, /*!< string*/
          U32 size /*!< size of string*/
      );

      //! Invoke the to port connected to AssertEnable
      //!
      void invoke_to_AssertEnable(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Common::EnableState enabled 
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of from_SerWritePort ports
      //!
      //! \return The number of from_SerWritePort ports
      //!
      NATIVE_INT_TYPE getNum_from_SerWritePort(void) const;

      //! Get the number of from_SwFault ports
      //!
      //! \return The number of from_SwFault ports
      //!
      NATIVE_INT_TYPE getNum_from_SwFault(void) const;

      //! Get the number of to_DebugStringIn ports
      //!
      //! \return The number of to_DebugStringIn ports
      //!
      NATIVE_INT_TYPE getNum_to_DebugStringIn(void) const;

      //! Get the number of to_AssertEnable ports
      //!
      //! \return The number of to_AssertEnable ports
      //!
      NATIVE_INT_TYPE getNum_to_AssertEnable(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_DebugStringIn[portNum] is connected
      //!
      bool isConnected_to_DebugStringIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_AssertEnable[portNum] is connected
      //!
      bool isConnected_to_AssertEnable(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to DebugStringIn
      //!
      LLProc::OutputDebugStringPort m_to_DebugStringIn[1];

      //! To port connected to AssertEnable
      //!
      Common::OutputEnablePort m_to_AssertEnable[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to SerWritePort
      //!
      Drv::InputSerialWritePort m_from_SerWritePort[1];

      //! From port connected to SwFault
      //!
      R5::InputGpioSetPort m_from_SwFault[1];

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

      //! Static function for port from_SwFault
      //!
      static void from_SwFault_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::GpioSetBank bank, 
          U32 bit, 
          R5::GpioSetVal val 
      );

  };

} // end namespace LLProc

#endif
