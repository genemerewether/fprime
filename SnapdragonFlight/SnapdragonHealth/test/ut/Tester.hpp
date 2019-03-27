// ======================================================================
// \title  SnapdragonHealth/test/ut/Tester.hpp
// \author gcgandhi
// \brief  hpp file for SnapdragonHealth test harness implementation class
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

#ifndef TESTER_HPP
#define TESTER_HPP

#include "GTestBase.hpp"
#include "SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentImpl.hpp"

namespace SnapdragonFlight {

  class Tester :
    public SnapdragonHealthGTestBase
  {

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

    public:

      //! Construct object Tester
      //!
      Tester(void);

      //! Destroy object Tester
      //!
      ~Tester(void);

    public:

      // ----------------------------------------------------------------------
      // Tests
      // ----------------------------------------------------------------------

      void run_nominal_tests(void);
      void run_off_nominal_tests(void);

      private:

        // ----------------------------------------------------------------------
        // Handlers for typed from ports
        // ----------------------------------------------------------------------

        //! Handler for from_PowerReport
        //!
        void from_PowerReport_handler(
            const NATIVE_INT_TYPE portNum, /*!< The port number*/
            U32 entry, /*!< The entry to access*/
            Svc::MeasurementStatus &status, /*!< The command response argument*/
            Fw::Time &time, /*!< The time of the measurement*/
            Fw::PolyType &val /*!< The value to be passed*/
        );

        //! Handler for from_pingOut
        //!
        void from_pingOut_handler(
            const NATIVE_INT_TYPE portNum, /*!< The port number*/
            U32 key /*!< Value to return to pinger*/
        );



    private:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

      void textLogIn(
                       const FwEventIdType id, //!< The event ID
                       Fw::Time& timeTag, //!< The time
                       const Fw::TextLogSeverity severity, //!< The severity
                       const Fw::TextLogString& text //!< The event string
                   );

      void dispatch(void);

      //! Connect ports
      //!
      void connectPorts(void);

      //! Initialize components
      //!
      void initComponents(void);

    private:

      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      //!
      SnapdragonHealthComponentImpl component;

  };

} // end namespace SnapdragonFlight

#endif
