// ======================================================================
// \title  R5A2DDriver/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for R5A2DDriver component test harness base class
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

#ifndef R5A2DDriver_TESTER_BASE_HPP
#define R5A2DDriver_TESTER_BASE_HPP

#include <R5/A2DDrv/R5A2DDriverComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace R5 {

  //! \class R5A2DDriverTesterBase
  //! \brief Auto-generated base class for R5A2DDriver component test harness
  //!
  class R5A2DDriverTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object R5A2DDriverTesterBase
      //!
      virtual void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

    public:

      // ----------------------------------------------------------------------
      // Connectors for 'to' ports
      // Connect these output ports to the input ports under test
      // ----------------------------------------------------------------------

      //! Connect get to to_get[portNum]
      //!
      void connect_to_get(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputA2DGetPort *const get /*!< The port*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5A2DDriverTesterBase
      //!
      R5A2DDriverTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object R5A2DDriverTesterBase
      //!
      virtual ~R5A2DDriverTesterBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to get
      //!
      void invoke_to_get(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          D2AGetBank bank, 
          U32 channel, 
          F32 &val 
      );

    public:

      // ----------------------------------------------------------------------
      // Getters for port counts
      // ----------------------------------------------------------------------

      //! Get the number of to_get ports
      //!
      //! \return The number of to_get ports
      //!
      NATIVE_INT_TYPE getNum_to_get(void) const;

    protected:

      // ----------------------------------------------------------------------
      // Connection status for to ports
      // ----------------------------------------------------------------------

      //! Check whether port is connected
      //!
      //! Whether to_get[portNum] is connected
      //!
      bool isConnected_to_get(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to get
      //!
      R5::OutputA2DGetPort m_to_get[1];

  };

} // end namespace R5

#endif
