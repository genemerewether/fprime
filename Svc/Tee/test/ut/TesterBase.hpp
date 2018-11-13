// ======================================================================
// \title  Tee/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for Tee component test harness base class
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

#ifndef Tee_TESTER_BASE_HPP
#define Tee_TESTER_BASE_HPP

#include <Svc/Tee/TeeComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Svc {

  //! \class TeeTesterBase
  //! \brief Auto-generated base class for Tee component test harness
  //!
  class TeeTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object TeeTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect DataIn to to_DataIn[portNum]
      //!
      void connect_to_DataIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::InputSerializePort *const DataIn /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from DataOut
      //!
      //! \return from_DataOut[portNum]
      //!
      Fw::InputSerializePort* get_from_DataOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object TeeTesterBase
      //!
      TeeTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object TeeTesterBase
      //!
      virtual ~TeeTesterBase(void);

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
      // Handler prototypes for serial from ports
      // ----------------------------------------------------------------------

      //! Handler prototype for from_DataOut
      //!
      virtual void from_DataOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      ) = 0;

      //! Handler base class function prototype for from_DataOut
      //!
      void from_DataOut_handlerBase (
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
      );


    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to DataIn
      //!
      void invoke_to_DataIn(
          NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_DataIn ports
      //!
      //! \return The number of to_DataIn ports
      //!
      NATIVE_INT_TYPE getNum_to_DataIn(void) const;

      //! Get the number of from_DataOut ports
      //!
      //! \return The number of from_DataOut ports
      //!
      NATIVE_INT_TYPE getNum_from_DataOut(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_DataIn[portNum] is connected
      //!
      bool isConnected_to_DataIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to DataIn
      //!
      Fw::OutputSerializePort m_to_DataIn[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to DataOut
      //!
      Fw::InputSerializePort m_from_DataOut[10];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_DataOut
      //!
      static void from_DataOut_static(
          Fw::PassiveComponentBase *const callComp, //!< The component instance
          const NATIVE_INT_TYPE portNum, //!< The port number
          Fw::SerializeBufferBase& Buffer //!< serialized data buffer
      );

  };

} // end namespace Svc

#endif
