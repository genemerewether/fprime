// ====================================================================== 
// \title  ShortLogQueue/test/ut/Tester.hpp
// \author tcanham
// \brief  hpp file for ShortLogQueue test harness implementation class
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
#include "LLProc/ShortLogQueue/ShortLogQueueComponentImpl.hpp"

namespace LLProc {

  class Tester :
    public ShortLogQueueGTestBase
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

      // Test case with one in, one out
      void doNominalOneIn(void);
      // Test case with all in, all out
      void doNominalAllIn(void);
      // Test case with some in, one out at a time with wraps
      void doNominalSomeIn(void);
    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_LogSend
      //!
      void from_LogSend_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwEventIdType id, /*!< Log ID*/
          U8 severity, /*!< The shorter severity argument*/
          Fw::LogBuffer &args /*!< Buffer containing serialized log entry*/
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
      ShortLogQueueComponentImpl component;

  };

} // end namespace LLProc

#endif
