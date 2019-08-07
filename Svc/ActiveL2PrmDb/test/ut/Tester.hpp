// ====================================================================== 
// \title  ActiveL2PrmDb/test/ut/Tester.hpp
// \author kubiak
// \brief  hpp file for ActiveL2PrmDb test harness implementation class
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
#include "Svc/ActiveL2PrmDb/ActiveL2PrmDbImpl.hpp"

namespace Svc {

  class Tester :
    public ActiveL2PrmDbGTestBase
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

      void requestPrmsTest(void);
      void setGetTest(void);
      void setSendTest(void);
      void sendTooLargeTest(void);
      void sendBuffOvfTest(void);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_sendPrm
      //!
      void from_sendPrm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool morePrms, 
          Fw::ParamList val 
      );

      //! Handler for from_recvPrmReady
      //!
      void from_recvPrmReady_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 maxSize, 
          bool reload 
      );

      //! Handler for from_triggerPrms
      //!
      void from_triggerPrms_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType dummy 
      );

      //! Handler for from_pingOut
      //!
      void from_pingOut_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
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
      ActiveL2PrmDbComponentImpl component;

  };

} // end namespace Svc

#endif
