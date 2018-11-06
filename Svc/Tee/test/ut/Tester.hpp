// ====================================================================== 
// \title  Tee/test/ut/Tester.hpp
// \author parallels
// \brief  hpp file for Tee test harness implementation class
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
#include "Svc/Tee/TeeComponentImpl.hpp"

namespace Svc {

  class Tester :
    public TeeGTestBase
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

      void twoOutputTest(void);

      class TestBuffer :
        public Fw::SerializeBufferBase
      {
  

        public:

#ifdef BUILD_UT
          void operator=(const Fw::SerializeBufferBase& other);
          TestBuffer(const Fw::SerializeBufferBase& other);
          TestBuffer(const TestBuffer& other);
          TestBuffer()
              : Fw::SerializeBufferBase()
          {
              memset(m_buff, 0, sizeof(m_buff));
          }
#endif

          NATIVE_UINT_TYPE getBuffCapacity(void) const {
            return sizeof(m_buff);
          }

          // Get the max number of bytes that can be serialized
          NATIVE_UINT_TYPE getBuffSerLeft(void) const {

            const NATIVE_UINT_TYPE size  = getBuffCapacity();
            const NATIVE_UINT_TYPE loc = getBuffLength();

            if (loc >= (size-1) ) {
                return 0;
            }
            else {
                return (size - loc - 1);
            }
          }

          U8* getBuffAddr(void) {
            return m_buff;
          }

          const U8* getBuffAddr(void) const {
            return m_buff;
          }

        private:
          // Should be the max of all the input ports serialized sizes...
          U8 m_buff[64];

      };

    private:

      // ----------------------------------------------------------------------
      // Handlers for serial from ports
      // ----------------------------------------------------------------------

      //! Handler for from_DataOut
      //!
      void from_DataOut_handler(
        NATIVE_INT_TYPE portNum, /*!< The port number*/
        Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
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
      TeeComponentImpl component;

      TestBuffer m_buff[2];

      NATIVE_INT_TYPE m_expected_buffer;

      NATIVE_INT_TYPE m_saw_buff[2];

  };

} // end namespace Svc

#endif
