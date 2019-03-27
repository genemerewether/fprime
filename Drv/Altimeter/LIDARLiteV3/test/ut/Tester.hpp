// ====================================================================== 
// \title  LIDARLiteV3/test/ut/Tester.hpp
// \author kubiak
// \brief  hpp file for LIDARLiteV3 test harness implementation class
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
#include "Drv/Altimeter/LIDARLiteV3/LIDARLiteV3Impl.hpp"

namespace Drv {

  class Tester :
    public LIDARLiteV3GTestBase
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
      void nominal_test(void);
      void status_err_test(void);
      void nack_test(void);
      void early_rg_test(void);

    private:

      // ----------------------------------------------------------------------
      // Handlers for typed from ports
      // ----------------------------------------------------------------------

      //! Handler for from_AltimeterSend
      //!
      void from_AltimeterSend_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Drv::Altimeter altimeterEuData /*!< Altimeter EU data*/
      );

      //! Handler for from_I2CConfig
      //!
      void from_I2CConfig_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 busSpeed, 
          U32 slaveAddr, 
          U32 timeout 
      );

      //! Handler for from_I2CWriteRead
      //!
      void from_I2CWriteRead_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::Buffer &writeBuffer, 
          Fw::Buffer &readBuffer 
      );

      //! Handler for from_I2CWriteReadStatus
      //!
      Drv::I2CStatus from_I2CWriteReadStatus_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool shouldBlock 
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
      LIDARLiteV3ComponentImpl component;

      static const U8 exp_init_sig_count[];
      static const U8 exp_init_acq_config[];
      static const U8 exp_init_threshold_bypass[];

      static const U8 exp_acq_cmd[];
      static const U8 exp_status_cmd[];
      static const U8 exp_measure_cmd[];

      I2CStatus ret_i2c_status;

      Fw::Buffer* writeBuffer_ptr;
      Fw::Buffer* readBuffer_ptr;

      Altimeter last_alt;

  };

} // end namespace Drv

#endif
