// ======================================================================
// \title  R5Prm/test/ut/TesterBase.hpp
// \author Auto-generated
// \brief  hpp file for R5Prm component test harness base class
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

#ifndef R5Prm_TESTER_BASE_HPP
#define R5Prm_TESTER_BASE_HPP

#include <R5/R5Prm/R5PrmComponentAc.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <stdio.h>
#include <Fw/Port/InputSerializePort.hpp>

namespace R5 {

  //! \class R5PrmTesterBase
  //! \brief Auto-generated base class for R5Prm component test harness
  //!
  class R5PrmTesterBase :
    public Fw::PassiveComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Initialization
      // ----------------------------------------------------------------------

      //! Initialize object R5PrmTesterBase
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
          R5::InputPrmSetPort *const set /*!< The port*/
      );

      //! Connect get to to_get[portNum]
      //!
      void connect_to_get(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputPrmGetPort *const get /*!< The port*/
      );

      //! Connect write to to_write[portNum]
      //!
      void connect_to_write(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputPrmWritePort *const write /*!< The port*/
      );

      //! Connect check to to_check[portNum]
      //!
      void connect_to_check(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::InputPrmCheckPort *const check /*!< The port*/
      );

    protected:

      // ----------------------------------------------------------------------
      // Construction and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5PrmTesterBase
      //!
      R5PrmTesterBase(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const U32 maxHistorySize /*!< The maximum size of each history*/
#else
          const U32 maxHistorySize /*!< The maximum size of each history*/
#endif
      );

      //! Destroy object R5PrmTesterBase
      //!
      virtual ~R5PrmTesterBase(void);

    protected:

      // ----------------------------------------------------------------------
      // Invocation functions for to ports
      // ----------------------------------------------------------------------

      //! Invoke the to port connected to set
      //!
      void invoke_to_set(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::PrmEntry &prm, 
          bool &valid 
      );

      //! Invoke the to port connected to get
      //!
      void invoke_to_get(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::PrmEntry &prm, 
          bool &valid 
      );

      //! Invoke the to port connected to write
      //!
      void invoke_to_write(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool force 
      );

      //! Invoke the to port connected to check
      //!
      void invoke_to_check(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          PrmWriteStatus &stat 
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

      //! Get the number of to_write ports
      //!
      //! \return The number of to_write ports
      //!
      NATIVE_INT_TYPE getNum_to_write(void) const;

      //! Get the number of to_check ports
      //!
      //! \return The number of to_check ports
      //!
      NATIVE_INT_TYPE getNum_to_check(void) const;

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
      //! Whether to_write[portNum] is connected
      //!
      bool isConnected_to_write(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

      //! Check whether port is connected
      //!
      //! Whether to_check[portNum] is connected
      //!
      bool isConnected_to_check(
          const NATIVE_INT_TYPE portNum /*!< The port number*/
      );

    private:

      // ----------------------------------------------------------------------
      // To ports
      // ----------------------------------------------------------------------

      //! To port connected to set
      //!
      R5::OutputPrmSetPort m_to_set[1];

      //! To port connected to get
      //!
      R5::OutputPrmGetPort m_to_get[1];

      //! To port connected to write
      //!
      R5::OutputPrmWritePort m_to_write[1];

      //! To port connected to check
      //!
      R5::OutputPrmCheckPort m_to_check[1];

  };

} // end namespace R5

#endif
