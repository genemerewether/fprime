// ======================================================================
// \title  R5SpiSlaveDriver/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for R5SpiSlaveDriver component test harness base class
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

#ifndef R5SpiSlaveDriver_TESTER_BASE_HPP
#define R5SpiSlaveDriver_TESTER_BASE_HPP

#include <R5/SpiSlaveDrv/R5SpiSlaveDriverComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace R5 {

  //! \class R5SpiSlaveDriverTesterBase
  //! \brief Auto-generated base class for R5SpiSlaveDriver component test harness
  //!
  class R5SpiSlaveDriverTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object R5SpiSlaveDriverTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect spiRecv to to_spiRecv[portNum]
      //!
      void connect_to_spiRecv(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputSpiReceivePort *const spiRecv /*!< The port*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5SpiSlaveDriverTesterBase
      //!
      R5SpiSlaveDriverTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object R5SpiSlaveDriverTesterBase
      //!
      virtual ~R5SpiSlaveDriverTesterBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to spiRecv
      //!
      void invoke_to_spiRecv(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &buff
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_spiRecv ports
      //!
      //! \return The number of to_spiRecv ports
      //!
      NATIVE_INT_TYPE getNum_to_spiRecv(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_spiRecv[portNum] is connected
      //!
      bool isConnected_to_spiRecv(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to spiRecv
      //!
        R5::OutputSpiReceivePort m_to_spiRecv[1];

  };

} // end namespace R5

#endif
