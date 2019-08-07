// ======================================================================
// \title  BlspPwmDriver/test/ut/Tester.hpp
// \author mereweth
// \brief  hpp file for BlspPwmDriver test harness implementation class
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
#include "SnapdragonFlight/BlspPwmDriver/BlspPwmDriverComponentImpl.hpp"

namespace SnapdragonFlight {

  class Tester :
    public BlspPwmDriverGTestBase
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
      void testPwm(NATIVE_UINT_TYPE pwm,
                   NATIVE_UINT_TYPE gpio, NATIVE_INT_TYPE cycles, F32 duty);

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
      BlspPwmDriverComponentImpl component;

  };

} // end namespace SnapdragonFlight

#endif
