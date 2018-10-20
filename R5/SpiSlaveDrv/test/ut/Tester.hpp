// ======================================================================
// \title  FcSpiSlaveDriver/test/ut/Tester.hpp
// \author iuchenik
// \brief  hpp file for FcSpiSlaveDriver test harness implementation class
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

#include "TesterBase.hpp"
#include "R5/SpiSlaveDrv/R5SpiSlaveDriverComponentImpl.hpp"
#include "R5/SpiMasterDrv/R5SpiMasterDriverComponentImpl.hpp"

namespace R5 {

  class Tester :
    public R5SpiSlaveDriverTesterBase
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
      void toDo(void);

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
      R5SpiSlaveDriverComponentImpl component;

      //! The helper component
      R5SpiMasterDriverComponentImpl spi_master_component;

      //! To port connected to spiSend
      R5::OutputSpiSendPort m_to_spiSend[1];

  };

} // end namespace R5

#endif
