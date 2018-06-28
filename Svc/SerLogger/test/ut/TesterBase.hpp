// ======================================================================
// \title  SerLogger/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for SerLogger component test harness base class
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

#ifndef SerLogger_TESTER_BASE_HPP
#define SerLogger_TESTER_BASE_HPP

#include <Svc/SerLogger/SerLoggerComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Svc {

  //! \class SerLoggerTesterBase
  //! \brief Auto-generated base class for SerLogger component test harness
  //!
  class SerLoggerTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object SerLoggerTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect SerPortIn to to_SerPortIn[portNum]
      //!
      void connect_to_SerPortIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputSerializePort *const SerPortIn /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from LogOut
      //!
      //! \return from_LogOut[portNum]
      //!
      Svc::InputActiveFileLogPortPort* get_from_LogOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object SerLoggerTesterBase
      //!
      SerLoggerTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object SerLoggerTesterBase
      //!
      virtual ~SerLoggerTesterBase(void);

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

      //! Handler prototype for from_LogOut
      //!
      virtual void from_LogOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::ActiveFileLoggerPacket &data /*!< The buffer being transferred*/
      ) = 0;

      //! Handler base function for from_LogOut
      //!
      void from_LogOut_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::ActiveFileLoggerPacket &data /*!< The buffer being transferred*/
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

      //! Push an entry on the history for from_LogOut
      void pushFromPortEntry_LogOut(
          Svc::ActiveFileLoggerPacket &data /*!< The buffer being transferred*/
      );

      //! A history entry for from_LogOut
      //!
      typedef struct {
        Svc::ActiveFileLoggerPacket data;
      } FromPortEntry_LogOut;

      //! The history for from_LogOut
      //!
      History<FromPortEntry_LogOut> 
        *fromPortHistory_LogOut;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to SerPortIn
      //!
      void invoke_to_SerPortIn(
          NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_SerPortIn ports
      //!
      //! \return The number of to_SerPortIn ports
      //!
      NATIVE_INT_TYPE getNum_to_SerPortIn(void) const;

      //! Get the number of from_LogOut ports
      //!
      //! \return The number of from_LogOut ports
      //!
      NATIVE_INT_TYPE getNum_from_LogOut(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_SerPortIn[portNum] is connected
      //!
      bool isConnected_to_SerPortIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to SerPortIn
      //!
      Fw::OutputSerializePort m_to_SerPortIn[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to LogOut
      //!
      Svc::InputActiveFileLogPortPort m_from_LogOut[1];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_LogOut
      //!
      static void from_LogOut_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::ActiveFileLoggerPacket &data /*!< The buffer being transferred*/
      );

  };

} // end namespace Svc

#endif
