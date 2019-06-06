// ====================================================================== 
// \title  BlspI2CDriver/test/ut/Tester.hpp
// \author mereweth
// \brief  hpp file for BlspI2CDriver test harness implementation class
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
#include "SnapdragonFlight/BlspI2CDriver/BlspI2CDriverComponentImpl.hpp"

namespace SnapdragonFlight {

  class Tester :
    public BlspI2CDriverGTestBase
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
      void testWriteRead(BYTE* buffer, NATIVE_INT_TYPE size,
                         NATIVE_INT_TYPE readSize);

      void openAndConfig(int i2c, U32 slave, U32 busSpeed);

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

      void textLogIn(const FwEventIdType id, //!< The event ID
            Fw::Time& timeTag, //!< The time
            const Fw::TextLogSeverity severity, //!< The severity
            const Fw::TextLogString& text //!< The event string
            );

    private:

      // ----------------------------------------------------------------------
      // Variables
      // ----------------------------------------------------------------------

      //! The component under test
      //!
      BlspI2CDriverComponentImpl component;

  };

} // end namespace SnapdragonFlight

#endif
