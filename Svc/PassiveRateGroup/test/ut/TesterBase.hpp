// ======================================================================
// \title  PassiveRateGroup/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for PassiveRateGroup component test harness base class
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

#ifndef PassiveRateGroup_TESTER_BASE_HPP
#define PassiveRateGroup_TESTER_BASE_HPP

#include <Svc/PassiveRateGroup/PassiveRateGroupComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace Svc {

  //! \class PassiveRateGroupTesterBase
  //! \brief Auto-generated base class for PassiveRateGroup component test harness
  //!
  class PassiveRateGroupTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object PassiveRateGroupTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect CycleIn to to_CycleIn[portNum]
      //!
      void connect_to_CycleIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::InputCyclePort *const CycleIn /*!< The port*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for 'from' ports
      // Connect these input ports to the output ports under test
      // ----------------------------------------------------------------------

      //! Get the port that receives input from RateGroupMemberOut
      //!
      //! \return from_RateGroupMemberOut[portNum]
      //!
      Svc::InputSchedPort* get_from_RateGroupMemberOut(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object PassiveRateGroupTesterBase
      //!
      PassiveRateGroupTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object PassiveRateGroupTesterBase
      //!
      virtual ~PassiveRateGroupTesterBase(void);

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

      //! Handler prototype for from_RateGroupMemberOut
      //!
      virtual void from_RateGroupMemberOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      ) = 0;

      //! Handler base function for from_RateGroupMemberOut
      //!
      void from_RateGroupMemberOut_handlerBase(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
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

      //! Push an entry on the history for from_RateGroupMemberOut
      void pushFromPortEntry_RateGroupMemberOut(
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! A history entry for from_RateGroupMemberOut
      //!
      typedef struct {
        NATIVE_UINT_TYPE context;
      } FromPortEntry_RateGroupMemberOut;

      //! The history for from_RateGroupMemberOut
      //!
      History<FromPortEntry_RateGroupMemberOut> 
        *fromPortHistory_RateGroupMemberOut;

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to CycleIn
      //!
      void invoke_to_CycleIn(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Svc::TimerVal &cycleStart /*!< Cycle start timer value*/
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_CycleIn ports
      //!
      //! \return The number of to_CycleIn ports
      //!
      NATIVE_INT_TYPE getNum_to_CycleIn(void) const;

      //! Get the number of from_RateGroupMemberOut ports
      //!
      //! \return The number of from_RateGroupMemberOut ports
      //!
      NATIVE_INT_TYPE getNum_from_RateGroupMemberOut(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_CycleIn[portNum] is connected
      //!
      bool isConnected_to_CycleIn(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to CycleIn
      //!
      Svc::OutputCyclePort m_to_CycleIn[1];

    private:

      // ----------------------------------------------------------------------
      // From ports
      // ----------------------------------------------------------------------

      //! From port connected to RateGroupMemberOut
      //!
      Svc::InputSchedPort m_from_RateGroupMemberOut[20];

    private:

      // ----------------------------------------------------------------------
      // Static functions for output ports
      // ----------------------------------------------------------------------

      //! Static function for port from_RateGroupMemberOut
      //!
      static void from_RateGroupMemberOut_static(
          Fw::PassiveComponentBase *const callComp, /*!< The component instance*/
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

  };

} // end namespace Svc

#endif
