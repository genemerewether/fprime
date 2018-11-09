// ====================================================================== 
// \title  ActuatorAdapterImpl.cpp
// \author mereweth
// \brief  cpp file for ActuatorAdapter component implementation class
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


#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ActuatorAdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ActuatorAdapterComponentImpl(
        const char *const compName
    ) :
      ActuatorAdapterComponentBase(compName)
#else
    ActuatorAdapterImpl(void)
#endif
  {
      for (U32 i = 0; i < AA_MAX_ACTUATORS; i++) {
          this->outputInfo[i].type = OUTPUT_UNSET;
      }
  }

  void ActuatorAdapterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    ActuatorAdapterComponentBase::init(instance);
  }

  ActuatorAdapterComponentImpl ::
    ~ActuatorAdapterComponentImpl(void)
  {

  }

  bool ActuatorAdapterComponentImpl ::
    setupI2C(
             U32 actuator,
             I2CMetadata meta
    )
  {
      if (actuator >= AA_MAX_ACTUATORS) {
          return false;
      }

      this->outputInfo[actuator].type = OUTPUT_I2C;
      this->outputInfo[actuator].i2cMeta = meta;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ActuatorAdapterComponentImpl ::
    motor_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Actuators &Actuators
    )
  {

      // TODO(mereweth) - add size arg to Actuators message
      for (U32 i = 0; i < FW_MIN(4, AA_MAX_ACTUATORS); i++) {
          switch (this->outputInfo[i].type) {
              case OUTPUT_UNSET:
              {
                  //TODO(mereweth) - issue error
              }
                  break;
              case OUTPUT_PWM:
              {
                if (this->isConnected_pwmSetDuty_OutputPort(0)) {
                    
                  }
                  else {
                      //TODO(mereweth) - issue error
                  }
              }
                  break;
              case OUTPUT_I2C:
              {
                  if (this->isConnected_escConfig_OutputPort(0) &&
                      this->isConnected_escReadWrite_OutputPort(0)) {
                      // TODO(mereweth) - put the I2C clock speed in config header? separate config ports?
                      this->escConfig_out(0, 400, this->outputInfo[i].i2cMeta.addr, 9000);

                      I2CMetadata i2c = this->outputInfo[i].i2cMeta;
                      U32 out = (/*val*/0.0f - i2c.minIn) / (i2c.maxIn - i2c.minIn) * (i2c.maxOut - i2c.minOut) + i2c.minOut;
                      
                      U8 readBuf[2] = { (U8) (out / 8), (U8) (out % 8) };
                      U8 writeBuf[2] = {0};
                      Fw::Buffer readBufObj(0, 0, (U64) readBuf, 2);
                      Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, 2);
                      this->escReadWrite_out(0, writeBufObj, readBufObj);
                  }
                  else {
                      //TODO(mereweth) - issue error
                  }
              }
                  break;
              default:
                  //TODO(mereweth) - DEBUG_PRINT
                  FW_ASSERT(0, this->outputInfo[i].type);
          }
      }
  }

} // end namespace Gnc
