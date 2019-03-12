// ====================================================================== 
// \title  LIDARLiteV3.hpp
// \author kubiak
// \brief  cpp file for LIDARLiteV3 test harness implementation class
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

#include "Tester.hpp"

#include <string.h>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 10

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction and destruction 
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) : 
#if FW_OBJECT_NAMES == 1
      LIDARLiteV3GTestBase("Tester", MAX_HISTORY_SIZE),
      component("LIDARLiteV3")
#else
      LIDARLiteV3GTestBase(MAX_HISTORY_SIZE),
      component()
#endif
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester(void) 
  {
    
  }

  const U8 Tester::exp_init_sig_count[] = {
    LLV3_SIG_COUNT_VAL_REG, LLV3_SIG_COUNT_VAL
  };

  const U8 Tester::exp_init_acq_config[] = {
    LLV3_ACQ_CONFIG_REG, LLV3_ACQ_CONFIG_VAL
  };

  const U8 Tester::exp_init_threshold_bypass[] = {
    LLV3_THRESHOLD_BYPASS_REG, LLV3_THRESHOLD_BYPASS_VAL
  };

  const U8 Tester::exp_acq_cmd[] = {
    LLV3_ACQ_REG, LLV3_ACQ_CMD
  };

  const U8 Tester::exp_status_cmd[] = {
    LLV3_STATUS_REG
  };

  const U8 Tester::exp_measure_cmd[] = {
    LLV3_MEASURE_REG | LLV3_READ_SEQ
  };

  // ----------------------------------------------------------------------
  // Tests 
  // ----------------------------------------------------------------------

  void Tester ::
    nominal_test(void) 
  {
    U8* i2c_read_data;

    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_CONFIG, this->component.init_state);

    // Send a Config port message
    this->clearHistory();
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_REG, this->component.init_state);

    ASSERT_from_I2CConfig_SIZE(1);
    ASSERT_from_I2CConfig(0, 100000, LLV3_ADDR, 0);


    // Send a write to SIG_COUNT_VAL
    this->clearHistory();
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_STATUS, this->component.init_state);

    ASSERT_from_I2CWriteRead_SIZE(1);

    ASSERT_EQ(2, this->writeBuffer_ptr->getsize());
    ASSERT_EQ(0, this->readBuffer_ptr->getsize());

    ASSERT_EQ(0, memcmp(reinterpret_cast<U8*>(this->writeBuffer_ptr->getdata()),
                        exp_init_sig_count,
                        sizeof(exp_init_sig_count)));


    // Get the status of the SIG_COUNT_VAL write
    this->clearHistory();
    this->ret_i2c_status = I2C_STATUS_OK;
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_REG, this->component.init_state);

    ASSERT_from_I2CWriteReadStatus_SIZE(1);
    ASSERT_from_I2CWriteReadStatus(0, false);

    // Send a write to ACQ_CONFIG_REG
    this->clearHistory();
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_STATUS, this->component.init_state);

    ASSERT_from_I2CWriteRead_SIZE(1);

    ASSERT_EQ(2, this->writeBuffer_ptr->getsize());
    ASSERT_EQ(0, this->readBuffer_ptr->getsize());

    ASSERT_EQ(0, memcmp(reinterpret_cast<U8*>(this->writeBuffer_ptr->getdata()),
                        exp_init_acq_config,
                        sizeof(exp_init_acq_config)));

    // Get the status of the ACQ_CONFIG_REG busy
    this->clearHistory();
    this->ret_i2c_status = I2C_STATUS_BUSY;
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_STATUS, this->component.init_state);

    ASSERT_from_I2CWriteReadStatus_SIZE(1);
    ASSERT_from_I2CWriteReadStatus(0, false);

    // Get the status of the ACQ_CONFIG_REG write
    this->clearHistory();
    this->ret_i2c_status = I2C_STATUS_OK;
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_REG, this->component.init_state);

    ASSERT_from_I2CWriteReadStatus_SIZE(1);
    ASSERT_from_I2CWriteReadStatus(0, false);

    // Confirm that a slow sched call doesn't interrupt initialization
    this->clearHistory();
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_SLOW);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_REG, this->component.init_state);

    ASSERT_from_I2CWriteReadStatus_SIZE(0);
    ASSERT_from_I2CWriteRead_SIZE(0);

    // Send a write to THRESHOLD_BYPASS
    this->clearHistory();
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_WRITE_STATUS, this->component.init_state);

    ASSERT_from_I2CWriteRead_SIZE(1);

    ASSERT_EQ(2, this->writeBuffer_ptr->getsize());
    ASSERT_EQ(0, this->readBuffer_ptr->getsize());

    ASSERT_EQ(0, memcmp(reinterpret_cast<U8*>(this->writeBuffer_ptr->getdata()),
                        exp_init_threshold_bypass,
                        sizeof(exp_init_threshold_bypass)));

    // Get the status of THRESHOLD_BYPASS
    this->clearHistory();
    this->ret_i2c_status = I2C_STATUS_OK;
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_COMPLETE, this->component.init_state);

    ASSERT_from_I2CWriteReadStatus_SIZE(1);
    ASSERT_from_I2CWriteReadStatus(0, false);

    // Done with initialization. Confirm more FAST sched calls do nothing
    this->clearHistory();
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_COMPLETE, this->component.init_state);

    ASSERT_from_I2CWriteReadStatus_SIZE(0);
    ASSERT_from_I2CWriteRead_SIZE(0);

    this->clearHistory();
    this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
    ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_INIT_COMPLETE, this->component.init_state);

    ASSERT_from_I2CWriteReadStatus_SIZE(0);
    ASSERT_from_I2CWriteRead_SIZE(0);

    int i;
    for (i = 0; i < 5; i++) {
        // Trigger a read with a SLOW sched call
        this->clearHistory();
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_SLOW);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_ACQ_CMD, this->component.i2c_state);


        // Start the I2C transaction. Send the ACQ Cmd
        this->clearHistory();
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_ACQ_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteRead_SIZE(1);

        ASSERT_EQ(2, this->writeBuffer_ptr->getsize());
        ASSERT_EQ(0, this->readBuffer_ptr->getsize());

        ASSERT_EQ(0, memcmp(reinterpret_cast<U8*>(this->writeBuffer_ptr->getdata()),
                            exp_acq_cmd,
                            sizeof(exp_acq_cmd)));


        // Get ACQ Cmd Status. Not ready
        this->clearHistory();
        this->ret_i2c_status = I2C_STATUS_BUSY;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_ACQ_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);

        // Get ACQ Cmd Status. Ready
        this->clearHistory();
        this->ret_i2c_status = I2C_STATUS_OK;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_STATUS_CMD, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);

        // Send Status Cmd
        this->clearHistory();
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_STATUS_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteRead_SIZE(1);

        ASSERT_EQ(1, this->writeBuffer_ptr->getsize());
        ASSERT_EQ(1, this->readBuffer_ptr->getsize());

        ASSERT_EQ(0, memcmp(reinterpret_cast<U8*>(this->writeBuffer_ptr->getdata()),
                            exp_status_cmd,
                            sizeof(exp_status_cmd)));


        // Get Status Cmd Status. Not ready
        this->clearHistory();
        this->ret_i2c_status = I2C_STATUS_BUSY;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_STATUS_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);


        // Get Status Cmd Status. Ready, but busy
        this->clearHistory();
        i2c_read_data = reinterpret_cast<U8*>(this->readBuffer_ptr->getdata());
        i2c_read_data[0] = LLV3_STATUS_BUSY | LLV3_STATUS_HEALTH_OK;
        this->ret_i2c_status = I2C_STATUS_OK;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_STATUS_CMD, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);


        // Send Status cmd
        this->clearHistory();
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_STATUS_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteRead_SIZE(1);

        ASSERT_EQ(1, this->writeBuffer_ptr->getsize());
        ASSERT_EQ(1, this->readBuffer_ptr->getsize());

        ASSERT_EQ(0, memcmp(reinterpret_cast<U8*>(this->writeBuffer_ptr->getdata()),
                            exp_status_cmd,
                            sizeof(exp_status_cmd)));

        // Get Status Cmd Status. Not ready
        this->clearHistory();
        this->ret_i2c_status = I2C_STATUS_BUSY;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_STATUS_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);

        // Get Status Cmd Status. Ready
        this->clearHistory();
        this->ret_i2c_status = I2C_STATUS_OK;
        i2c_read_data = reinterpret_cast<U8*>(this->readBuffer_ptr->getdata());
        i2c_read_data[0] = LLV3_STATUS_HEALTH_OK;
        this->ret_i2c_status = I2C_STATUS_OK;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_MEASURE_CMD, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);

        // Send Measure Cmd
        this->clearHistory();
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_MEASURE_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteRead_SIZE(1);

        ASSERT_EQ(1, this->writeBuffer_ptr->getsize());
        ASSERT_EQ(2, this->readBuffer_ptr->getsize());

        ASSERT_EQ(0, memcmp(reinterpret_cast<U8*>(this->writeBuffer_ptr->getdata()),
                            exp_measure_cmd,
                            sizeof(exp_measure_cmd)));

        // Get Measure Cmd Status. Not Ready
        this->clearHistory();
        this->ret_i2c_status = I2C_STATUS_BUSY;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_MEASURE_CMD_STATUS, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);


        // Get Measure Cmd Status. Ready
        this->clearHistory();
        this->ret_i2c_status = I2C_STATUS_OK;
        i2c_read_data = reinterpret_cast<U8*>(this->readBuffer_ptr->getdata());

        // Send 1316 cm, or 13.16m
        i2c_read_data[0] = 0x5;
        i2c_read_data[1] = 0x24;
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_WAITING, this->component.i2c_state);

        ASSERT_from_I2CWriteReadStatus_SIZE(1);
        ASSERT_from_I2CWriteReadStatus(0, false);

        ASSERT_from_AltimeterSend_SIZE(1);

        ASSERT_FLOAT_EQ(this->last_alt.getdistance(), 13.16);
        ASSERT_EQ(this->last_alt.getstatus(), 0);
        ASSERT_EQ(this->last_alt.gettime_secs(), 0);
        ASSERT_EQ(this->last_alt.gettime_nsecs(), 0);

        // Further Fast calls should be No-ops
        this->clearHistory();
        this->invoke_to_SchedIn(0,LIDARLiteV3ComponentImpl::LLV3_RG_FAST);
        ASSERT_EQ(LIDARLiteV3ComponentImpl::LLV3_I2C_WAITING, this->component.i2c_state);

        ASSERT_from_AltimeterSend_SIZE(0);
        ASSERT_from_I2CWriteRead_SIZE(0);
        ASSERT_from_I2CWriteReadStatus_SIZE(0);
    }
  }

  void Tester ::
    status_err_test(void) 
  {
    ASSERT_TRUE(false);
  }

  void Tester ::
    nack_test(void) 
  {
    ASSERT_TRUE(false);
  }

  void Tester ::
    early_rg_test(void)
  {
    ASSERT_TRUE(false);
  }

  // ----------------------------------------------------------------------
  // Handlers for typed from ports
  // ----------------------------------------------------------------------

  void Tester ::
    from_AltimeterSend_handler(
        const NATIVE_INT_TYPE portNum,
        Drv::Altimeter altimeterEuData
    )
  {
    this->pushFromPortEntry_AltimeterSend(altimeterEuData);

    last_alt = altimeterEuData;
  }

  void Tester ::
    from_I2CConfig_handler(
        const NATIVE_INT_TYPE portNum,
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
    this->pushFromPortEntry_I2CConfig(busSpeed, slaveAddr, timeout);
  }

  void Tester ::
    from_I2CWriteRead_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    this->pushFromPortEntry_I2CWriteRead(writeBuffer, readBuffer);

    this->writeBuffer_ptr = &writeBuffer;
    this->readBuffer_ptr = &readBuffer;
  }

  Drv::I2CStatus Tester ::
    from_I2CWriteReadStatus_handler(
        const NATIVE_INT_TYPE portNum,
        bool shouldBlock
    )
  {
    this->pushFromPortEntry_I2CWriteReadStatus(shouldBlock);

    return this->ret_i2c_status;
  }

  // ----------------------------------------------------------------------
  // Helper methods 
  // ----------------------------------------------------------------------

  void Tester ::
    connectPorts(void) 
  {

    // SchedIn
    this->connect_to_SchedIn(0,
        this->component.get_SchedIn_InputPort(0)
    );

    // AltimeterSend
    this->component.set_AltimeterSend_OutputPort(
        0, 
        this->get_from_AltimeterSend(0)
    );

    // I2CConfig
    this->component.set_I2CConfig_OutputPort(
        0, 
        this->get_from_I2CConfig(0)
    );

    // I2CWriteRead
    this->component.set_I2CWriteRead_OutputPort(
        0, 
        this->get_from_I2CWriteRead(0)
    );

    // I2CWriteReadStatus
    this->component.set_I2CWriteReadStatus_OutputPort(
        0, 
        this->get_from_I2CWriteReadStatus(0)
    );




  }

  void Tester ::
    initComponents(void) 
  {
    this->init();
    this->component.init(
        INSTANCE
    );
  }

} // end namespace Drv
