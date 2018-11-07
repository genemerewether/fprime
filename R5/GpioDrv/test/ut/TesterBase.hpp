// ======================================================================
// \title  R5GpioDriver/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for R5GpioDriver component test harness base class
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

#ifndef R5GpioDriver_TESTER_BASE_HPP
#define R5GpioDriver_TESTER_BASE_HPP

#include <R5/GpioDrv/R5GpioDriverComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace R5 {

  //! \class R5GpioDriverTesterBase
  //! \brief Auto-generated base class for R5GpioDriver component test harness
  //!
  class R5GpioDriverTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object R5GpioDriverTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect set to to_set[portNum]
      //!
      void connect_to_set(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputGpioSetPort *const set /*!< The port*/
      );

      //! Connect get to to_get[portNum]
      //!
      void connect_to_get(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputGpioGetPort *const get /*!< The port*/
      );

      //! Connect wait to to_wait[portNum]
      //!
      void connect_to_wait(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputGpioWaitPort *const wait /*!< The port*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5GpioDriverTesterBase
      //!
      R5GpioDriverTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object R5GpioDriverTesterBase
      //!
      virtual ~R5GpioDriverTesterBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to set
      //!
      void invoke_to_set(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          GpioSetBank bank,
          U32 bit,
          GpioSetVal val
      );

      //! Invoke the to port connected to get
      //!
      void invoke_to_get(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          GpioGetBank bank,
          U32 bit,
          GpioGetVal &val
      );

      //! Invoke the to port connected to wait
      //!
      void invoke_to_wait(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          GpioWaitBank bank,
          U32 bit,
          bool &alreadySet
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_set ports
      //!
      //! \return The number of to_set ports
      //!
      NATIVE_INT_TYPE getNum_to_set(void) const;

      //! Get the number of to_get ports
      //!
      //! \return The number of to_get ports
      //!
      NATIVE_INT_TYPE getNum_to_get(void) const;

      //! Get the number of to_wait ports
      //!
      //! \return The number of to_wait ports
      //!
      NATIVE_INT_TYPE getNum_to_wait(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_set[portNum] is connected
      //!
      bool isConnected_to_set(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_get[portNum] is connected
      //!
      bool isConnected_to_get(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_wait[portNum] is connected
      //!
      bool isConnected_to_wait(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to set
      //!
      R5::OutputGpioSetPort m_to_set[1];

      //! To port connected to get
      //!
      R5::OutputGpioGetPort m_to_get[1];

      //! To port connected to wait
      //!
      R5::OutputGpioWaitPort m_to_wait[1];

  };

} // end namespace R5

#endif
