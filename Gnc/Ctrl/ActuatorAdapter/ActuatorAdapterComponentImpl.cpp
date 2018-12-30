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
#include "Fw/Types/SerialBuffer.hpp"

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
    armCount(0u),
    numActuators(0u),
    paramsInited(false)
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

      return true;
  }

  void ActuatorAdapterComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
#ifndef BUILD_TIR5
    printf("prm %d updated\n", id);
#endif
  }

  void ActuatorAdapterComponentImpl ::
    parametersLoaded()
  {
      this->paramsInited = false;
      Fw::ParamValid valid[6];
      this->numActuators = paramGet_numActuators(valid[0]);
      if (Fw::PARAM_VALID != valid[0]) {  return;  }
      if (this->numActuators >= AA_MAX_ACTUATORS) {  return;  }

      OutputType outType = OUTPUT_UNSET;
      I2CMetadata i2c;
      for (U32 i = 0; i < this->numActuators; i++) {
          switch (i) {
              case 0:
                  outType = (OutputType) paramGet_a1_type(valid[0]);
                  if ((outType < OUTPUT_VALID_MIN) || 
                      (outType > OUTPUT_VALID_MAX)) {
                      return;
                  }
                  switch (outType) {
                      case OUTPUT_I2C:
                          i2c.addr = paramGet_a1_addr(valid[0]);
                          i2c.minIn = paramGet_a1_minCmd(valid[1]);
                          i2c.maxIn = paramGet_a1_maxCmd(valid[2]);
                          i2c.minOut = (U32) paramGet_a1_minOut(valid[3]);
                          i2c.maxOut = (U32) paramGet_a1_maxOut(valid[4]);
                          i2c.reverse = paramGet_a1_reverse(valid[5]);
                          if (!setupI2C(i, i2c)) {  return;  }
                          break;
                      default:
                          DEBUG_PRINT("Unhandled act adap type %u\n", outType);
                          FW_ASSERT(0, outType);
                          break;
                  }
                  break;
              case 1:
                  outType = (OutputType) paramGet_a2_type(valid[0]);
                  if ((outType < OUTPUT_VALID_MIN) || 
                      (outType > OUTPUT_VALID_MAX)) {
                      return;
                  }
                  switch (outType) {
                      case OUTPUT_I2C:
                          i2c.addr = paramGet_a2_addr(valid[0]);
                          i2c.minIn = paramGet_a2_minCmd(valid[1]);
                          i2c.maxIn = paramGet_a2_maxCmd(valid[2]);
                          i2c.minOut = (U32) paramGet_a2_minOut(valid[3]);
                          i2c.maxOut = (U32) paramGet_a2_maxOut(valid[4]);
                          i2c.reverse = paramGet_a2_reverse(valid[5]);
                          if (!setupI2C(i, i2c)) {  return;  }
                          break;
                      default:
                          DEBUG_PRINT("Unhandled act adap type %u\n", outType);
                          FW_ASSERT(0, outType);
                          break;
                  }
                  break;
              case 2:
                  outType = (OutputType) paramGet_a3_type(valid[0]);
                  if ((outType < OUTPUT_VALID_MIN) || 
                      (outType > OUTPUT_VALID_MAX)) {
                      return;
                  }
                  switch (outType) {
                      case OUTPUT_I2C:
                          i2c.addr = paramGet_a3_addr(valid[0]);
                          i2c.minIn = paramGet_a3_minCmd(valid[1]);
                          i2c.maxIn = paramGet_a3_maxCmd(valid[2]);
                          i2c.minOut = (U32) paramGet_a3_minOut(valid[3]);
                          i2c.maxOut = (U32) paramGet_a3_maxOut(valid[4]);
                          i2c.reverse = paramGet_a3_reverse(valid[5]);
                          if (!setupI2C(i, i2c)) {  return;  }
                          break;
                      default:
                          DEBUG_PRINT("Unhandled act adap type %u\n", outType);
                          FW_ASSERT(0, outType);
                          break;
                  }
                  break;
              case 3:
                  outType = (OutputType) paramGet_a4_type(valid[0]);
                  if ((outType < OUTPUT_VALID_MIN) || 
                      (outType > OUTPUT_VALID_MAX)) {
                      return;
                  }
                  switch (outType) {
                      case OUTPUT_I2C:
                          i2c.addr = paramGet_a4_addr(valid[0]);
                          i2c.minIn = paramGet_a4_minCmd(valid[1]);
                          i2c.maxIn = paramGet_a4_maxCmd(valid[2]);
                          i2c.minOut = (U32) paramGet_a4_minOut(valid[3]);
                          i2c.maxOut = (U32) paramGet_a4_maxOut(valid[4]);
                          i2c.reverse = paramGet_a4_reverse(valid[5]);
                          if (!setupI2C(i, i2c)) {  return;  }
                          break;
                      default:
                          DEBUG_PRINT("Unhandled act adap type %u\n", outType);
                          FW_ASSERT(0, outType);
                          break;
                  }
                  break;
              case 4:
                  outType = (OutputType) paramGet_a5_type(valid[0]);
                  if ((outType < OUTPUT_VALID_MIN) || 
                      (outType > OUTPUT_VALID_MAX)) {
                      return;
                  }
                  switch (outType) {
                      case OUTPUT_I2C:
                          i2c.addr = paramGet_a5_addr(valid[0]);
                          i2c.minIn = paramGet_a5_minCmd(valid[1]);
                          i2c.maxIn = paramGet_a5_maxCmd(valid[2]);
                          i2c.minOut = (U32) paramGet_a5_minOut(valid[3]);
                          i2c.maxOut = (U32) paramGet_a5_maxOut(valid[4]);
                          i2c.reverse = paramGet_a5_reverse(valid[5]);
                          if (!setupI2C(i, i2c)) {  return;  }
                          break;
                      default:
                          DEBUG_PRINT("Unhandled act adap type %u\n", outType);
                          FW_ASSERT(0, outType);
                          break;
                  }
                  break;
              case 5:
                  outType = (OutputType) paramGet_a6_type(valid[0]);
                  if ((outType < OUTPUT_VALID_MIN) || 
                      (outType > OUTPUT_VALID_MAX)) {
                      return;
                  }
                  switch (outType) {
                      case OUTPUT_I2C:
                          i2c.addr = paramGet_a6_addr(valid[0]);
                          i2c.minIn = paramGet_a6_minCmd(valid[1]);
                          i2c.maxIn = paramGet_a6_maxCmd(valid[2]);
                          i2c.minOut = (U32) paramGet_a6_minOut(valid[3]);
                          i2c.maxOut = (U32) paramGet_a6_maxOut(valid[4]);
                          i2c.reverse = paramGet_a6_reverse(valid[5]);
                          if (!setupI2C(i, i2c)) {  return;  }
                          break;
                      default:
                          DEBUG_PRINT("Unhandled act adap type %u\n", outType);
                          FW_ASSERT(0, outType);
                          break;
                  }
                  break;
              default:
                  FW_ASSERT(0, i);
          }

          for (U32 j = 0; j < FW_NUM_ARRAY_ELEMENTS(valid); j++) {
              if (Fw::PARAM_VALID != valid[j]) {  return;  }
          }
      }

      this->paramsInited = true;
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
      if ((ARMED != this->armedState) ||
          !paramsInited) {
          return;
      }

      U32 angVelCount = FW_MIN(Actuators.getangular_velocities_count(),
                               FW_MIN(AA_MAX_ACTUATORS,
                                      this->numActuators));
      NATIVE_INT_TYPE angVelSize = 0;
      const F64* angVels = Actuators.getangular_velocities(angVelSize);

      for (U32 i = 0; i < FW_MIN(angVelCount, angVelSize); i++) {
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
                      Fw::Time now = this->getTime();

                      // TODO(mereweth) - put the I2C clock speed in config header? separate config ports?
                      this->escConfig_out(0, 400, this->outputInfo[i].i2cMeta.addr, 100);

                      I2CMetadata i2c = this->outputInfo[i].i2cMeta;
                      F64 inVal = angVels[i];
                      if (inVal > i2c.maxIn) {  inVal = i2c.maxIn;  }
                      if (inVal < i2c.minIn) {  inVal = i2c.minIn;  }
                      U32 out = (inVal - i2c.minIn) / (i2c.maxIn - i2c.minIn) * (i2c.maxOut - i2c.minOut) + i2c.minOut;

                      DEBUG_PRINT("esc addr %u, in %f, out %u\n", i2c.addr, angVels[i], out);

                      // TODO(mereweth) - run controller here

                      U8 readBuf[9] = { 0 };
                      // MSB is reverse bit
                      U8 writeBuf[2] = { (U8) ((out / 8) | ((i2c.reverse ? 1 : 0) << 7)),
                                         (U8) (out % 8) };
                      Fw::Buffer readBufObj(0, 0, (U64) readBuf, FW_NUM_ARRAY_ELEMENTS(readBuf));
                      Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                      this->escReadWrite_out(0, writeBufObj, readBufObj);
                      
                      if (0xab == readBuf[8]) {
                          this->outputInfo[i].feedback.counts      = (readBuf[0] << 8) + readBuf[1];
                          this->outputInfo[i].feedback.voltage     = (F32) ((readBuf[2] << 8) + readBuf[3]) / 1000.0;
                          this->outputInfo[i].feedback.temperature = (F32) ((readBuf[4] << 8) + readBuf[5]) / 1000.0;
                          this->outputInfo[i].feedback.current     = (F32) ((readBuf[6] << 8) + readBuf[7]) / 1000.0;
                          this->outputInfo[i].feedback.sec         = now.getSeconds();
                          this->outputInfo[i].feedback.usec        = now.getUSeconds();
                      }

                      // TODO(mereweth) - run rate estimator here
                      
                      Fw::SerializeStatus status;
                      // sec, usec, motor id, command, response
                      U8 buff[2 * sizeof(U32) + sizeof(U8) + 2 * sizeof(U8)];
                      Fw::SerialBuffer buffObj(buff, FW_NUM_ARRAY_ELEMENTS(buff));
                      status = buffObj.serialize(now.getSeconds());
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      status = buffObj.serialize(now.getUSeconds());
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      status = buffObj.serialize((U8) i);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));

                      NATIVE_INT_TYPE size = FW_NUM_ARRAY_ELEMENTS(writeBuf);
                      status = buffObj.serialize(writeBuf, size, false); // serialize length
                      FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(writeBuf) == size, size);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));

                      size = FW_NUM_ARRAY_ELEMENTS(readBuf);
                      status = buffObj.serialize(readBuf, size, false); // serialize length
                      FW_ASSERT(FW_NUM_ARRAY_ELEMENTS(readBuf) == size, size);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));

                      if (this->isConnected_serialDat_OutputPort(0)) {
                          this->serialDat_out(0, buffObj);
                      }
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
          if (!paramsInited) {
              this->armedState = DISARMED;
              this->cmdResponse_out(this->opCode, this->cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
              return;
          }
          if (++this->armCount > AA_ARM_COUNT) {
              this->armedState = ARMED;
              this->cmdResponse_out(this->opCode, this->cmdSeq, Fw::COMMAND_OK);
              return;
          }
          for (U32 i = 0; i < FW_MIN(this->numActuators, AA_MAX_ACTUATORS); i++) {
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
      if (!paramsInited) {
          this->armedState = DISARMED;
          this->cmdResponse_out(this->opCode, this->cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
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

  void ActuatorAdapterComponentImpl ::
    ACTADAP_InitParams_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
      this->parametersLoaded();
      if (this->paramsInited) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }
      else {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
      }
  }

} // end namespace Gnc
