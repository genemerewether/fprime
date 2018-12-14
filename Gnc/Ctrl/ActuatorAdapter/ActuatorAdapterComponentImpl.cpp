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

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#include <stdio.h>
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

#undef DEBUG_PRINT
#define DEBUG_PRINT(x,...)

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ActuatorAdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ActuatorAdapterComponentImpl(
        const char *const compName
    ) :
      ActuatorAdapterComponentBase(compName),
#else
    ActuatorAdapterImpl(void),
#endif
    outputInfo(),
    armedState(DISARMED),
    opCode(0u),
    cmdSeq(0u),
    armCount(0u)
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
      if (ARMED != this->armedState) {
          return;
      }

      NATIVE_INT_TYPE angVelSize = FW_MIN(4, AA_MAX_ACTUATORS);
      const F64* angVels = Actuators.getangular_velocities(angVelSize);

      // TODO(mereweth) - add size arg to Actuators message fields
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
                      this->escConfig_out(0, 400, this->outputInfo[i].i2cMeta.addr, 100);

                      I2CMetadata i2c = this->outputInfo[i].i2cMeta;
                      F64 inVal = angVels[i];
                      if (inVal > i2c.maxIn) {  inVal = i2c.maxIn;  }
                      if (inVal < i2c.minIn) {  inVal = i2c.minIn;  }
                      U32 out = (inVal - i2c.minIn) / (i2c.maxIn - i2c.minIn) * (i2c.maxOut - i2c.minOut) + i2c.minOut;

                      DEBUG_PRINT("esc addr %u, in %f, out %u\n", i2c.addr, angVels[i], out);

                      U8 readBuf[2] = { 0 };
                      U8 writeBuf[2] = { (U8) (out / 8), (U8) (out % 8) };
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

  void ActuatorAdapterComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      if (ARMING == this->armedState) {
          if (++this->armCount > AA_ARM_COUNT) {
              this->armedState = ARMED;
              this->cmdResponse_out(this->opCode, this->cmdSeq, Fw::COMMAND_OK);
              return;
          }
          for (U32 i = 0; i < AA_MAX_ACTUATORS; i++) {
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
                          this->escConfig_out(0, 400, this->outputInfo[i].i2cMeta.addr, 100);

                          U8 readBuf[1] = { 0 };
                          U8 writeBuf[1] = { 0 };
                          Fw::Buffer writeObj = Fw::Buffer(0, 0, (U64) writeBuf, 1);
                          Fw::Buffer readObj = Fw::Buffer(0, 0, (U64) readBuf, 1);
                          this->escReadWrite_out(0, writeObj, readObj);
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
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void ActuatorAdapterComponentImpl ::
    ACTADAP_Arm_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        bool armState
    )
  {
      // can only arm if disarmed
      if ((DISARMED != this->armedState) && armState) {
          this->log_WARNING_LO_ACTADAP_AlreadyArmed();
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }

      // if we get here, either we are disarmed now, or about to disarm
      this->armCount = 0u;

      // can disarm immediately - change this if hardware changes
      if (!armState) {
          this->armedState = DISARMED;
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
          return;
      }

      // if we get here, we are disarmed, and want to arm
      this->armedState = ARMING;

      this->opCode = opCode;
      this->cmdSeq = cmdSeq;
  }

} // end namespace Gnc
