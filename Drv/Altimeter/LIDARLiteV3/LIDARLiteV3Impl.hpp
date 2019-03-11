// ====================================================================== 
// \title  LIDARLiteV3Impl.hpp
// \author kubiak
// \brief  hpp file for LIDARLiteV3 component implementation class
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

#ifndef LIDARLiteV3_HPP
#define LIDARLiteV3_HPP

#include "Drv/Altimeter/LIDARLiteV3/LIDARLiteV3ComponentAc.hpp"

#include <Drv/Altimeter/LIDARLiteV3/LIDARLiteV3ImplCfg.hpp>

namespace Drv {

  class LIDARLiteV3ComponentImpl :
    public LIDARLiteV3ComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object LIDARLiteV3
      //!
      LIDARLiteV3ComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object LIDARLiteV3
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object LIDARLiteV3
      //!
      ~LIDARLiteV3ComponentImpl(void);

      struct LLV3InitReg {
          U8 reg, value;
      };

    PRIVATE:

      void reset_i2c_buffers();

      void dn_to_eu();

      void initialize_sm();

      void send_measurement();

      bool verify_status();

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for SchedIn
      //!
      void SchedIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      enum LLV3RateGroup {
          LLV3_RG_FAST = 0,
          LLV3_RG_SLOW = 1
      };

      enum LLV3InitState {
          LLV3_INIT_WAITING,
          LLV3_INIT_CONFIG,
          LLV3_INIT_WRITE_REG,
          LLV3_INIT_WRITE_STATUS,
          LLV3_INIT_COMPLETE
      };

      enum LLV3I2CState {
          LLV3_I2C_WAITING,
          LLV3_I2C_ACQ_CMD,
          LLV3_I2C_ACQ_CMD_STATUS,
          LLV3_I2C_STATUS_CMD,
          LLV3_I2C_STATUS_CMD_STATUS,
          LLV3_I2C_MEASURE_CMD,
          LLV3_I2C_MEASURE_CMD_STATUS
      };

      static const LLV3InitReg init_registers[3];
      LLV3InitState init_state;
      U32 init_registers_idx;

      LLV3I2CState i2c_state;

      U8 i2cWriteBufferArr[LLV3_WRITE_BUFF_LEN];
      U8 i2cReadBufferArr[LLV3_READ_BUFF_LEN];

      Fw::Buffer i2cWriteBuffer;
      Fw::Buffer i2cReadBuffer;

      Drv::Altimeter altimeter_eu;
    };

} // end namespace Drv

#endif
