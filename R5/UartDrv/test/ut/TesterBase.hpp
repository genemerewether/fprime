// ======================================================================
// \title  R5UartDriver/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for R5UartDriver component test harness base class
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

#ifndef R5UartDriver_TESTER_BASE_HPP
#define R5UartDriver_TESTER_BASE_HPP

#include <R5/UartDrv/R5UartDriverComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace R5 {

  //! \class R5UartDriverTesterBase
  //! \brief Auto-generated base class for R5UartDriver component test harness
  //!
  class R5UartDriverTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object R5UartDriverTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect serialRecv to to_serialRecv[portNum]
      //!
      void connect_to_serialRecv(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputSerialReadPort *const serialRecv /*!< The port*/
      );

      //! Connect serialSend to to_serialSend[portNum]
      //!
      void connect_to_serialSend(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::InputSerialWritePort *const serialSend /*!< The port*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5UartDriverTesterBase
      //!
      R5UartDriverTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object R5UartDriverTesterBase
      //!
      virtual ~R5UartDriverTesterBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to serialRecv
      //!
      void invoke_to_serialRecv(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer, /*!< Buffer containing data*/
          Drv::SerialReadStatus &status /*!< Status of read*/
      );

      //! Invoke the to port connected to serialSend
      //!
      void invoke_to_serialSend(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &serBuffer 
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_serialRecv ports
      //!
      //! \return The number of to_serialRecv ports
      //!
      NATIVE_INT_TYPE getNum_to_serialRecv(void) const;

      //! Get the number of to_serialSend ports
      //!
      //! \return The number of to_serialSend ports
      //!
      NATIVE_INT_TYPE getNum_to_serialSend(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_serialRecv[portNum] is connected
      //!
      bool isConnected_to_serialRecv(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_serialSend[portNum] is connected
      //!
      bool isConnected_to_serialSend(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to serialRecv
      //!
      Drv::OutputSerialReadPort m_to_serialRecv[1];

      //! To port connected to serialSend
      //!
      Drv::OutputSerialWritePort m_to_serialSend[1];

  };

} // end namespace R5

#endif
