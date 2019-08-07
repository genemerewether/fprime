// ======================================================================
// \title  TimeSyncOffset/test/ut/Tester.hpp
// \author kedelson
// \brief  hpp file for TimeSyncOffset test harness implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================

#ifndef TESTER_HPP
#define TESTER_HPP

#include "GTestBase.hpp"
#include "Svc/TimeSyncOffset/TimeSyncOffsetComponentImpl.hpp"

namespace Svc {

  class Tester :
    public TimeSyncOffsetGTestBase
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

      //! To do
      //!
      void SendPulseRecieveTimeTest(void);
      void RecieveTimeNoPulseSentTest(void);
      void SendPulseNoRecieveTimeTest(void);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_GPIOPulse
      //!
      void from_GPIOPulse_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool state 
      );

      //! Handler for from_ClockTimes
      //!
      void from_ClockTimes_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Time time1, /*!< first time*/
          Fw::Time time2 /*!< second time*/
      );

    private:

      // ----------------------------------------------------------------------
      // Helper methods
      // ----------------------------------------------------------------------

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
      TimeSyncOffsetComponentImpl component;

  };

} // end namespace Svc

#endif
