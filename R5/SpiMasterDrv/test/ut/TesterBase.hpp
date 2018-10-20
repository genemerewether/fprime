// ======================================================================
// \title  R5SpiMasterDriver/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for R5SpiMasterDriver component test harness base class
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

#ifndef R5SpiMasterDriver_TESTER_BASE_HPP
#define R5SpiMasterDriver_TESTER_BASE_HPP

#include <R5/SpiMasterDrv/R5SpiMasterDriverComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace R5 {

  //! \class R5SpiMasterDriverTesterBase
  //! \brief Auto-generated base class for R5SpiMasterDriver component test harness
  //!
  class R5SpiMasterDriverTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object R5SpiMasterDriverTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect spiSend to to_spiSend[portNum]
      //!
      void connect_to_spiSend(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputSpiSendPort *const spiSend /*!< The port*/
      );

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

      //! Construct object R5SpiMasterDriverTesterBase
      //!
      R5SpiMasterDriverTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object R5SpiMasterDriverTesterBase
      //!
      virtual ~R5SpiMasterDriverTesterBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to spiSend
      //!
      void invoke_to_spiSend(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer *buff, 
          U32 numBuffs 
      );

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

      //! Get the number of to_spiSend ports
      //!
      //! \return The number of to_spiSend ports
      //!
      NATIVE_INT_TYPE getNum_to_spiSend(void) const;

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
      //! Whether to_spiSend[portNum] is connected
      //!
      bool isConnected_to_spiSend(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

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

      //! To port connected to spiSend
      //!
      R5::OutputSpiSendPort m_to_spiSend[1];

      //! To port connected to spiRecv
      //!
      R5::OutputSpiReceivePort m_to_spiRecv[1];

  };

} // end namespace R5

#endif
