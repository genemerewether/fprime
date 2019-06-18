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

#include <math.h>
#include <string.h>

#ifndef M_PI
#ifdef BUILD_DSPAL
#define M_PI 3.14159265358979323846
#endif
#endif

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
    flySafe(true),
    flySafeCheckCycles(false),
    flySafeCycles(0u),
    flySafeMaxElapsedCycles(0u),
    flySafeCheckTime(false),
    flySafeLastTime(TB_WORKSTATION_TIME, 0, 0, 0),
    flySafeMaxElapsedTime(TB_WORKSTATION_TIME, 0, 0, 0),
    paramsInited(false)
  {
      for (U32 i = 0; i < ACTADAP_MAX_ACTUATORS; i++) {
          this->outputInfo[i].type = OUTPUT_UNSET;
          this->outputInfo[i].inputActType = INPUTACT_UNSET;
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

  void ActuatorAdapterComponentImpl ::
    sendZeroCmdAll(void)
  {
      for (U32 i = 0; i < FW_MIN(this->numActuators, ACTADAP_MAX_ACTUATORS); i++) {
          if (INPUTACT_ANGLE != this->outputInfo[i].inputActType) {
              switch (this->outputInfo[i].type) {
                  case OUTPUT_UNSET:
                  {
                      //TODO(mereweth) - issue error
                  }
                      break;
                  case OUTPUT_PWM:
                  {
                      PwmMetadata pwm = this->outputInfo[i].pwmMeta;
                      if (this->isConnected_pwmSetDuty_OutputPort(pwm.port)) {

                          F32 duty[ACTADAP_MAX_ACTUATORS] = { 0.0 };
                          if (pwm.addr >= FW_NUM_ARRAY_ELEMENTS(duty)) {
                              //TODO(mereweth) - evr
                              break;
                          }

                          duty[pwm.addr] = this->outputInfo[i].pwmMeta.cmdOutputMap.offset;
                          U32 bitmask = 1 << pwm.addr;
                          Drv::PwmSetDutyCycle dutySer(duty, FW_NUM_ARRAY_ELEMENTS(duty), bitmask);
                          this->pwmSetDuty_out(pwm.port, dutySer);
                      }
                      else {
                          //TODO(mereweth) - issue error
                      }
                  }
                      break;
                  case OUTPUT_I2C:
                  case OUTPUT_I2C_SIMPLE:
                  case OUTPUT_I2C_SHORT:
                  {
                      I2CMetadata i2c = this->outputInfo[i].i2cMeta;
                      if (this->isConnected_escConfig_OutputPort(i2c.port) &&
                          this->isConnected_escReadWrite_OutputPort(i2c.port)) {
                          // TODO(mereweth) - put the I2C clock speed in config header? separate config ports?
                          this->escConfig_out(i2c.port, 400, i2c.addr, 500);

                          Fw::Buffer readBufObj(0, 0, 0, 0); // no read
                          if ((OUTPUT_I2C       == this->outputInfo[i].type) ||
                              (OUTPUT_I2C_SHORT == this->outputInfo[i].type)) {
                              // MSB is reverse bit
                              U8 writeBuf[3] = { 0 };
                              Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                              this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
                          }
                          else { // simple protocol
                              U8 writeBuf[2] = { 0 };
                              Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                              this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
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
  }

  bool ActuatorAdapterComponentImpl ::
    setupI2C(
             U32 actuator,
             I2CMetadata meta,
             I2CProtocol i2cProto,
             InputActuatorType inputActType,
             U32 inputActIdx
    )
  {
      if (actuator >= ACTADAP_MAX_ACTUATORS) {
          return false;
      }
      if ((inputActType > INPUTACT_VALID_MAX) ||
          (inputActType < INPUTACT_VALID_MIN)) {
          return false;
      }
      this->outputInfo[actuator].inputActType = inputActType;
      this->outputInfo[actuator].inputActIdx = inputActIdx;
      // TODO(mereweth) - factor out FeedbackMetadata and CmdOutputMapMetadata checking
      if (0 == meta.fbMeta.countsPerRev) {
          return false;
      }
      meta.fbMeta.kp = fabs(meta.fbMeta.kp);
      meta.fbMeta.maxErr = fabs(meta.fbMeta.maxErr);
      meta.cmdOutputMap.Vnom = fabs(meta.cmdOutputMap.Vnom);
      meta.cmdOutputMap.Vact = meta.cmdOutputMap.Vnom;

      switch (i2cProto) {
          case I2CProtoSimple:
              this->outputInfo[actuator].type = OUTPUT_I2C_SIMPLE;
              break;
          case I2CProtoShort:
              this->outputInfo[actuator].type = OUTPUT_I2C_SHORT;
              break;
          case I2CProtoLong:
              this->outputInfo[actuator].type = OUTPUT_I2C;
              break;
          default:
            return false;
      }
      this->outputInfo[actuator].i2cMeta = meta;

      return true;
  }

  bool ActuatorAdapterComponentImpl ::
    setupPwm(
             U32 actuator,
             PwmMetadata meta,
             InputActuatorType inputActType,
             U32 inputActIdx
    )
  {
      if (actuator >= ACTADAP_MAX_ACTUATORS) {
          return false;
      }
      if ((inputActType > INPUTACT_VALID_MAX) ||
          (inputActType < INPUTACT_VALID_MIN)) {
          return false;
      }
      this->outputInfo[actuator].inputActType = inputActType;
      this->outputInfo[actuator].inputActIdx = inputActIdx;
      // TODO(mereweth) - factor out FeedbackMetadata and CmdOutputMapMetadata checking
      meta.cmdOutputMap.Vnom = fabs(meta.cmdOutputMap.Vnom);
      meta.cmdOutputMap.Vact = meta.cmdOutputMap.Vnom;

      this->outputInfo[actuator].type = OUTPUT_PWM;
      this->outputInfo[actuator].pwmMeta = meta;

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
      Fw::ParamValid valid[7];
      this->numActuators = paramGet_numActuators(valid[0]);
      if (Fw::PARAM_VALID != valid[0]) {  return;  }
      if (this->numActuators > ACTADAP_MAX_ACTUATORS) {  return;  }

      // NOTE(mereweth) - start convenience defines
#define MAP_FROM_PARM_IDX(XXX) \
      { \
          memset(&cmdOutputMap, 0, sizeof(cmdOutputMap)); \
          cmdOutputMap.minIn = paramGet_p ## XXX ## _minCmd(valid[0]); \
          cmdOutputMap.maxIn = paramGet_p ## XXX ## _maxCmd(valid[1]); \
          for (U32 j = 0; j < 2; j++) { \
              if (Fw::PARAM_VALID != valid[j]) {  return;  } \
          } \
          \
          cmdOutputMap.minOut = paramGet_p ## XXX ## _minOut(valid[0]); \
          cmdOutputMap.maxOut = paramGet_p ## XXX ## _maxOut(valid[1]); \
          for (U32 j = 0; j < 2; j++) { \
              if (Fw::PARAM_VALID != valid[j]) {  return;  } \
          } \
          \
          cmdOutputMap.offset = paramGet_p ## XXX ## _map_offset(valid[0]); \
          \
          if ((Fw::PARAM_VALID != valid[0]) && \
              (Fw::PARAM_DEFAULT != valid[0])) { \
              return; \
          } \
          \
          cmdOutputMap.type = (CmdOutputMapType) paramGet_p ## XXX ## _mapType(valid[0]); \
          if ((Fw::PARAM_VALID != valid[0]) || \
              (cmdOutputMap.type < CMD_OUTPUT_MAP_VALID_MIN) || \
              (cmdOutputMap.type > CMD_OUTPUT_MAP_VALID_MAX)) { \
              return; \
          } \
          cmdOutputMap.Vnom = paramGet_p ## XXX ## _map_Vnom(valid[0]); \
          cmdOutputMap.Vact = cmdOutputMap.Vnom; \
          if (Fw::PARAM_VALID != valid[0]) {  return;  } \
          switch (cmdOutputMap.type) { \
              case CMD_OUTPUT_MAP_UNSET: \
                  DEBUG_PRINT("Command to output mapping type unset\n"); \
                  return; \
              case CMD_OUTPUT_MAP_LIN_MINMAX: \
                  /* NOTE(mereweth) - nothing else to do*/ \
                  break; \
              case CMD_OUTPUT_MAP_LIN_2BRK: \
                  cmdOutputMap.x1 = paramGet_p ## XXX ## _map_x1(valid[0]); \
                  cmdOutputMap.k2 = paramGet_p ## XXX ## _map_k2(valid[1]); \
                  for (U32 j = 0; j < 2; j++) { \
                      if (Fw::PARAM_VALID != valid[j]) {  return;  } \
                  } \
                  /* NOTE(mereweth) - fallthrough intended*/ \
              case CMD_OUTPUT_MAP_LIN_1BRK: \
                  cmdOutputMap.x0 = paramGet_p ## XXX ## _map_x0(valid[0]); \
                  cmdOutputMap.k1 = paramGet_p ## XXX ## _map_k1(valid[1]); \
                  for (U32 j = 0; j < 2; j++) { \
                      if (Fw::PARAM_VALID != valid[j]) {  return;  } \
                  } \
                  /* NOTE(mereweth) - fallthrough intended*/ \
              case CMD_OUTPUT_MAP_LIN_0BRK: \
                  cmdOutputMap.k0 = paramGet_p ## XXX ## _map_k0(valid[0]); \
                  cmdOutputMap.b = paramGet_p ## XXX ## _map_b(valid[1]); \
                  for (U32 j = 0; j < 2; j++) { \
                      if (Fw::PARAM_VALID != valid[j]) {  return;  } \
                  } \
                  break; \
              default: \
                  DEBUG_PRINT("Unhandled mapping type %u\n", cmdOutputMap.type); \
                  return; \
          } \
      }

#define FB_FROM_PARM_IDX(XXX) \
      { \
          memset(&fb, 0, sizeof(fb)); \
          fb.countsPerRev = paramGet_p ## XXX ## _counts(valid[0]); \
          fb.maxErr = paramGet_p ## XXX ## _ctrl_maxErr(valid[1]); \
          fb.ctrlType = (FeedbackCtrlType) paramGet_p ## XXX ## _ctrlType(valid[2]); \
          for (U32 j = 0; j < 3; j++) { \
              if (Fw::PARAM_VALID != valid[j]) {  return;  } \
          } \
          if ((fb.ctrlType < FEEDBACK_CTRL_VALID_MIN) || \
              (fb.ctrlType > FEEDBACK_CTRL_VALID_MAX)) { \
              return; \
          } \
          switch (fb.ctrlType) { \
              case FEEDBACK_CTRL_NONE: \
                  /* NOTE(mereweth) - nothing else to do*/ \
                  break; \
              case FEEDBACK_CTRL_PROP: \
                  fb.kp = paramGet_p ## XXX ## _ctrl_kp(valid[0]); \
                  if (Fw::PARAM_VALID != valid[0]) {  return;  } \
                  break; \
              default: \
                  DEBUG_PRINT("Unhandled feedback ctrl type %u\n", fb.ctrlType); \
                  return; \
          } \
      }

#define METADATA_FROM_ACT_IDX(XXX) \
      { \
          U8 parmSlot = paramGet_a ## XXX ## _parmSlot(valid[0]); \
          if (Fw::PARAM_VALID != valid[0]) {  return;  } \
          switch (parmSlot) { \
              case 0: \
                  return; \
              case 1: \
                  outType = (OutputType) paramGet_p1_outputType(valid[0]); \
                  inputActType = (InputActuatorType) paramGet_p1_inputActType(valid[1]); \
                  break; \
              case 2: \
                  outType = (OutputType) paramGet_p2_outputType(valid[0]); \
                  inputActType = (InputActuatorType) paramGet_p2_inputActType(valid[1]); \
                  break; \
              default: \
                  DEBUG_PRINT("Unhandled parm slot %u\n", parmSlot); \
                  return; \
          } \
          inputActIdx = paramGet_a ## XXX ## _inputActIdx(valid[2]); \
          for (U32 j = 0; j < 3; j++) { \
              if (Fw::PARAM_VALID != valid[j]) {  return;  } \
          } \
          if ((outType < OUTPUT_VALID_MIN) || \
              (outType > OUTPUT_VALID_MAX) || \
              (inputActType < INPUTACT_VALID_MIN) || \
              (inputActType > INPUTACT_VALID_MAX)) { \
              return; \
          } \
          DEBUG_PRINT("Parm slot %u first part ok\n", parmSlot); \
          \
          switch (outType) { \
              case OUTPUT_UNSET: \
                  DEBUG_PRINT("Actuator type unset\n"); \
                  return; \
              case OUTPUT_PWM: \
                  pwm.addr = paramGet_a ## XXX ## _addr(valid[0]); \
                  pwm.reverse = paramGet_a ## XXX ## _reverse(valid[1]); \
                  for (U32 j = 0; j < 2; j++) { \
                      if (Fw::PARAM_VALID != valid[j]) {  return;  } \
                  } \
                  \
                  pwm.port = paramGet_a ## XXX ## _port(valid[0]); \
                  if ((Fw::PARAM_VALID   != valid[0])  && \
                      (Fw::PARAM_DEFAULT != valid[0])) { \
                      return; \
                  } \
                  /* TODO(mereweth) - update when number of parm slots updates */ \
                  switch (parmSlot) { \
                      case 0: \
                          return; \
                      case 1: \
                          MAP_FROM_PARM_IDX(1); \
                          break; \
                      case 2: \
                          MAP_FROM_PARM_IDX(2); \
                          break; \
                      default: \
                          DEBUG_PRINT("Unhandled parm slot %u\n", parmSlot); \
                          return; \
                  } \
                  DEBUG_PRINT("Parm slot %u map ok\n", parmSlot); \
                  pwm.cmdOutputMap = cmdOutputMap; \
                  \
                  if (!setupPwm(i, pwm, inputActType, inputActIdx)) { \
                      DEBUG_PRINT("Setup pwm failed\n"); \
                      return; \
                  } \
                  break; \
              case OUTPUT_I2C: \
              case OUTPUT_I2C_SIMPLE: \
              case OUTPUT_I2C_SHORT: \
                  i2c.addr = paramGet_a ## XXX ## _addr(valid[0]); \
                  i2c.reverse = paramGet_a ## XXX ## _reverse(valid[1]); \
                  for (U32 j = 0; j < 2; j++) { \
                      if (Fw::PARAM_VALID != valid[j]) {  return;  } \
                  } \
                  \
                  i2c.port = paramGet_a ## XXX ## _port(valid[0]); \
                  if ((Fw::PARAM_VALID   != valid[0])  && \
                      (Fw::PARAM_DEFAULT != valid[0])) { \
                      return; \
                  } \
                  /* TODO(mereweth) - update when number of parm slots updates */ \
                  switch (parmSlot) { \
                      case 0: \
                          return; \
                      case 1: \
                          MAP_FROM_PARM_IDX(1); \
                          FB_FROM_PARM_IDX(1); \
                          break; \
                      case 2: \
                          MAP_FROM_PARM_IDX(2); \
                          FB_FROM_PARM_IDX(2); \
                          break; \
                      default: \
                          DEBUG_PRINT("Unhandled parm slot %u\n", parmSlot); \
                          return; \
                  } \
                  i2c.cmdOutputMap = cmdOutputMap; \
                  i2c.fbMeta = fb; \
                  \
                  I2CProtocol i2cProto; \
                  switch (outType) { \
                      case OUTPUT_I2C_SIMPLE: \
                          i2cProto = I2CProtoSimple; \
                          break; \
                      case OUTPUT_I2C_SHORT: \
                          i2cProto = I2CProtoShort; \
                          break; \
                      case OUTPUT_I2C: \
                          i2cProto = I2CProtoLong; \
                          break; \
                      default: \
                        return false; \
                  } \
                  if (!setupI2C(i, i2c, i2cProto, \
                                inputActType, inputActIdx)) { \
                      return; \
                  } \
                  break; \
              default: \
                  DEBUG_PRINT("Unknown act adap type %u\n", outType); \
                  FW_ASSERT(0, outType); \
                  return; \
          } \
      }
      // NOTE(mereweth) - end convenience defines

      OutputType outType = OUTPUT_UNSET;
      InputActuatorType inputActType = INPUTACT_UNSET;
      U32 inputActIdx = 0u;
      I2CMetadata i2c;
      PwmMetadata pwm;
      FeedbackMetadata fb;
      CmdOutputMapMetadata cmdOutputMap;
      for (U32 i = 0; i < this->numActuators; i++) {
          outType = OUTPUT_UNSET;
          inputActType = INPUTACT_UNSET;
          switch (i) {
              case 0:
                  METADATA_FROM_ACT_IDX(1);
                  break;
              case 1:
                  METADATA_FROM_ACT_IDX(2);
                  break;
              case 2:
                  METADATA_FROM_ACT_IDX(3);
                  break;
              case 3:
                  METADATA_FROM_ACT_IDX(4);
                  break;
              case 4:
                  METADATA_FROM_ACT_IDX(5);
                  break;
              case 5:
                  METADATA_FROM_ACT_IDX(6);
                  break;
              case 6:
                  METADATA_FROM_ACT_IDX(7);
                  break;
              case 7:
                  METADATA_FROM_ACT_IDX(8);
                  break;
              default:
                  FW_ASSERT(0, i);
          }
      }

      this->flySafeMaxElapsedCycles = paramGet_flySafeCycles(valid[0]);
      if (0 == this->flySafeMaxElapsedCycles) {
          this->flySafeCheckCycles = false;
      }
      else {
          this->flySafeCheckCycles = true;
      }
      const F64 flySafeFloatSecs = paramGet_flySafeTime(valid[1]);
      const Fw::Time forBaseContext = this->getTime();
      const TimeBase timeBase = forBaseContext.getTimeBase();
      const FwTimeContextStoreType timeContext = forBaseContext.getContext();
      if (flySafeFloatSecs > 0.0) {
          this->flySafeCheckTime = true;
          const U32 flySafeSecs = (U32) flySafeFloatSecs;
          this->flySafeLastTime = Fw::Time(timeBase, timeContext, 0, 0);
          this->flySafeMaxElapsedTime = Fw::Time(timeBase, timeContext,
                                                 flySafeSecs,
                                                 (U32) ((flySafeFloatSecs - flySafeSecs)
                                                 * 1000.0 * 1000.0));
      }
      else { // user gave negative value for timeout, indicating don't check time
          this->flySafeCheckTime = false;
      }

      for (unsigned int i = 0; i < 2; i++) {
          if ((Fw::PARAM_VALID   != valid[i])  &&
              (Fw::PARAM_DEFAULT != valid[i])) {
              return;
          }
      }

      this->paramsInited = true;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ActuatorAdapterComponentImpl ::
    flySafe_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::BoolStamped &BoolStamped
    )
  {
      Fw::Time msgStamp = BoolStamped.getheader().getstamp();
      // If msg time < time of last fly safe msg, it is out of order, so ignore it
      if (msgStamp < this->flySafeLastTime) {
          return;
      }
      // if msg time > current time, there has been an error in time translation, so ignore it
      if (msgStamp > this->getTime()) {
          return;
      }
      this->flySafeCycles = 0;
      this->flySafeLastTime = msgStamp;
      this->flySafe = BoolStamped.getdata().getdata();
  }

  void ActuatorAdapterComponentImpl ::
    motor_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::Actuators &Actuators
    )
  {
      bool hwEnabled = true;
      if (this->isConnected_outputEnable_OutputPort(0)) {
          this->outputEnable_out(0, hwEnabled);
      }

      if (ARMED != this->armedState) {
          return;
      }

      const bool didFlySafeCycleTimeout = (this->flySafeCheckCycles &&
                                           ((++this->flySafeCycles) > this->flySafeMaxElapsedCycles));
      const bool didFlySafeTimeTimeout = (this->flySafeCheckTime &&
                                          (this->getTime() > Fw::Time::add(this->flySafeMaxElapsedTime,
                                                                           this->flySafeLastTime)));
      if (!paramsInited || !hwEnabled ||
          didFlySafeCycleTimeout || didFlySafeTimeTimeout || !flySafe) {
          this->armedState = DISARMED;
          this->sendZeroCmdAll();
          if (!this->paramsInited) {
              this->log_WARNING_HI_ACTADAP_Error(ParamsUninit);
          }
          if (!hwEnabled) {
              this->log_WARNING_HI_ACTADAP_Error(HWEnableLow);
          }
          if (didFlySafeCycleTimeout) {
              this->log_WARNING_HI_ACTADAP_NotFlySafe(CycleTimeout);
          }
          if (didFlySafeTimeTimeout) {
              this->log_WARNING_HI_ACTADAP_NotFlySafe(TimeTimeout);
          }
          if (!this->flySafe) {
              this->log_WARNING_HI_ACTADAP_NotFlySafe(ValueFalse);
          }
          return;
      }

      U32 angleCount = Actuators.getangles_count();
      NATIVE_INT_TYPE angleSize = 0;
      const F64* angles = Actuators.getangles(angleSize);
      const U32 numAngles = FW_MIN(angleCount, angleSize);

      U32 angVelCount = Actuators.getangular_velocities_count();
      NATIVE_INT_TYPE angVelSize = 0;
      const F64* angVels = Actuators.getangular_velocities(angVelSize);
      const U32 numAngVels = FW_MIN(angVelCount, angVelSize);

      // send commands - lower latency
      for (U32 i = 0; i < FW_MIN(this->numActuators, ACTADAP_MAX_ACTUATORS); i++) {
          F64 inVal = 0.0;
          switch (this->outputInfo[i].inputActType) {
              case INPUTACT_UNSET:
              {
                  //TODO(mereweth) - issue diagnostic that this is unset
              }
                  break;
              case INPUTACT_ANGLE:
              {
                  if (this->outputInfo[i].inputActIdx >= numAngles) {
                      // TODO(mereweth) - EVR
                      break;
                  }
                  inVal = angles[this->outputInfo[i].inputActIdx];
              }
                  break;
              case INPUTACT_ANGULAR_VELOCITY:
              {
                  if (this->outputInfo[i].inputActIdx >= numAngVels) {
                      // TODO(mereweth) - EVR
                      break;
                  }
                  inVal = angVels[this->outputInfo[i].inputActIdx];
              }
                  break;
              default:
                  //TODO(mereweth) - DEBUG_PRINT
                  FW_ASSERT(0, this->outputInfo[i].inputActType);
          }
          // initialize to safe values
          FeedbackMetadata fbMeta;
          fbMeta.ctrlType = FEEDBACK_CTRL_NONE;
          CmdOutputMapMetadata cmdOutputMap;
          cmdOutputMap.type = CMD_OUTPUT_MAP_UNSET;
          switch (this->outputInfo[i].type) {
              case OUTPUT_UNSET:
              {
                  //TODO(mereweth) - issue diagnostic that this is unset
              }
                  break;
              case OUTPUT_PWM:
              {
                  cmdOutputMap = this->outputInfo[i].pwmMeta.cmdOutputMap;
              }
                  break;
              case OUTPUT_I2C:
              case OUTPUT_I2C_SIMPLE:
              case OUTPUT_I2C_SHORT:
              {
                  cmdOutputMap = this->outputInfo[i].i2cMeta.cmdOutputMap;
                  fbMeta = this->outputInfo[i].i2cMeta.fbMeta;
              }
                  break;
              default:
                  //TODO(mereweth) - DEBUG_PRINT
                  FW_ASSERT(0, this->outputInfo[i].type);
          }

          // NOTE(Mereweth) - DSPAL has no isnan
          if (!(inVal < cmdOutputMap.maxIn) &&
              !(inVal > cmdOutputMap.minIn)) {
              // TODO(mereweth) - EVR about disarming due to NaN
              inVal = cmdOutputMap.minIn;
              this->armedState = DISARMED;

              this->log_WARNING_HI_ACTADAP_Error(NaNCmd);
          }
          else if (inVal > cmdOutputMap.maxIn) {  inVal = cmdOutputMap.maxIn;  }
          else if (inVal < cmdOutputMap.minIn) {  inVal = cmdOutputMap.minIn;  }
          F64 out = 0.0;

          // TODO(mereweth) - share mapping among output types
          F64 Vnom_by_Vact = 1.0;
          if (cmdOutputMap.Vact > 1e-3) {
              Vnom_by_Vact = cmdOutputMap.Vnom / cmdOutputMap.Vact;
              if ((Vnom_by_Vact < 0.1) ||
                  (Vnom_by_Vact > 10.0)) {
                  // TODO(mereweth) - EVR!!
                  Vnom_by_Vact = 1.0;
              }
          }
          else {
              // TODO(mereweth) - EVR!!
          }

          const F64 inValAbs = fabs(inVal);
          F64 sign = 1.0;

          if ((CMD_OUTPUT_MAP_LIN_0BRK == cmdOutputMap.type) ||
              (CMD_OUTPUT_MAP_LIN_1BRK == cmdOutputMap.type) ||
              (CMD_OUTPUT_MAP_LIN_2BRK == cmdOutputMap.type)) {
              if (inVal < 0.0) {
                  sign = -1.0;
              }
          }
          switch (cmdOutputMap.type) {
              case CMD_OUTPUT_MAP_LIN_MINMAX:
                  out = (inVal - cmdOutputMap.minIn) /
                        (cmdOutputMap.maxIn - cmdOutputMap.minIn) * (cmdOutputMap.maxOut - cmdOutputMap.minOut)
                        + cmdOutputMap.minOut;
                  break;
              case CMD_OUTPUT_MAP_LIN_2BRK:
                  if (inValAbs < cmdOutputMap.x0) {
                      out = cmdOutputMap.k0 * Vnom_by_Vact * inValAbs + cmdOutputMap.b;
                  }
                  else if (inValAbs < cmdOutputMap.x1) {
                      out = cmdOutputMap.k0 * Vnom_by_Vact * cmdOutputMap.x0
                            + cmdOutputMap.b
                            + cmdOutputMap.k1 * Vnom_by_Vact * (inValAbs - cmdOutputMap.x0);
                  }
                  else {
                      out = cmdOutputMap.k0 * Vnom_by_Vact * cmdOutputMap.x0
                            + cmdOutputMap.b
                            + cmdOutputMap.k1  * Vnom_by_Vact
                            * (cmdOutputMap.x1 - cmdOutputMap.x0)
                            + cmdOutputMap.k2 * Vnom_by_Vact * (inValAbs - cmdOutputMap.x1);
                  }
                    break;
              case CMD_OUTPUT_MAP_LIN_1BRK:
                  if (inValAbs < cmdOutputMap.x0) {
                      out = cmdOutputMap.k0 * Vnom_by_Vact * inValAbs + cmdOutputMap.b;
                  }
                  else {
                      out = cmdOutputMap.k0 * Vnom_by_Vact * cmdOutputMap.x0
                            + cmdOutputMap.b
                            + cmdOutputMap.k1 * Vnom_by_Vact * (inValAbs - cmdOutputMap.x0);
                  }
                  break;
              case CMD_OUTPUT_MAP_LIN_0BRK:
                    out = cmdOutputMap.k0 * Vnom_by_Vact * inValAbs + cmdOutputMap.b;
                    break;
              case CMD_OUTPUT_MAP_UNSET:
              default:
                  DEBUG_PRINT("Unhandled command output map slot %u\n", cmdOutputMap.type);
                  FW_ASSERT(0, cmdOutputMap.type);
                  break;
          }

          F64 delta = 0.0;
          switch (fbMeta.ctrlType) {
              case FEEDBACK_CTRL_PROP:
                  // TODO(Mereweth) - what if not tracking near zero in bidirectional?
                  // TODO(mereweth) - check input type here - feedback might be an angle
                  delta = fbMeta.kp * (inValAbs - fabs(this->outputInfo[i].feedback.angVel));

                  // NOTE(Mereweth) - fallthrough so this code runs for all control types except NONE

                  // NOTE(Mereweth) - clamp the delta vs the open-loop mapping
                  if (delta > 0.0) {
                      delta = FW_MIN(delta, fbMeta.maxErr);
                  }
                  if (delta < 0.0) {
                      delta = -1.0 * FW_MIN(-1.0 * delta, fbMeta.maxErr);
                  }

                  out += delta;
                  break;
              case FEEDBACK_CTRL_NONE:
                  default:
                  break;
          }

          // if we took fabs of inVal for calculating the mapping, put it back the right way
          out *= sign;

          /* NOTE(mereweth) - prevent piecewise mappings from flipping the sign due to e.g.
          * negative y intercept, or controller flipping the sign
          */
          if (((inVal > 0.0) && (out < 0.0)) ||
              ((inVal < 0.0) && (out > 0.0))) {
              out = 0.0;
          }

          /* NOTE(mereweth) - in case inVal is exactly 0.0
          */
          if ((CMD_OUTPUT_MAP_LIN_0BRK == cmdOutputMap.type) ||
              (CMD_OUTPUT_MAP_LIN_1BRK == cmdOutputMap.type) ||
              (CMD_OUTPUT_MAP_LIN_2BRK == cmdOutputMap.type)) {
              if (inVal == 0.0) {
                  out = 0;
              }
          }

          DEBUG_PRINT("esc idx %u, in %f, out %f\n", i, inVal, out);

          switch (this->outputInfo[i].type) {
              case OUTPUT_UNSET:
              {
                  //TODO(mereweth) - issue diagnostic that this is unset
              }
                  break;
              case OUTPUT_PWM:
              {
                  PwmMetadata pwm = this->outputInfo[i].pwmMeta;
                  if (this->isConnected_pwmSetDuty_OutputPort(pwm.port)) {
                      Fw::Time cmdTime = this->getTime();
                      
                      if (out > pwm.cmdOutputMap.maxOut) {  out = pwm.cmdOutputMap.maxOut;  }
                      if (out < pwm.cmdOutputMap.minOut) {  out = pwm.cmdOutputMap.minOut;  }
                      
                      // TODO(mereweth) - param for default zero value
                      F32 duty[ACTADAP_MAX_ACTUATORS] = { 0.0 };
                      if (pwm.addr >= FW_NUM_ARRAY_ELEMENTS(duty)) {
                          //TODO(mereweth) - evr
                          break;
                      }

                      if (pwm.reverse) {  out = -out;  }
                      duty[pwm.addr] = out + cmdOutputMap.offset;
                      U32 bitmask = 1 << pwm.addr;
                      DEBUG_PRINT("pwm esc idx %u, offset %f, duty %f\n",
                                  i, cmdOutputMap.offset, duty[pwm.addr]);
                      Drv::PwmSetDutyCycle dutySer(duty, FW_NUM_ARRAY_ELEMENTS(duty), bitmask);
                      this->pwmSetDuty_out(pwm.port, dutySer);
                      this->outputInfo[i].feedback.cmdIn   = inVal;
                      this->outputInfo[i].feedback.cmd     = duty[pwm.addr];
                      this->outputInfo[i].feedback.cmdSec  = cmdTime.getSeconds();
                      this->outputInfo[i].feedback.cmdUsec = cmdTime.getUSeconds();
                  }
                  else {
                      //TODO(mereweth) - issue error
                  }
              }
                  break;
              case OUTPUT_I2C:
              case OUTPUT_I2C_SIMPLE:
              case OUTPUT_I2C_SHORT:
              {
                  I2CMetadata i2c = this->outputInfo[i].i2cMeta;
                  if (this->isConnected_escConfig_OutputPort(i2c.port) &&
                      this->isConnected_escReadWrite_OutputPort(i2c.port)) {
                      Fw::Time cmdTime = this->getTime();

                      // TODO(mereweth) - put the I2C clock speed in config header? separate config ports?
                      this->escConfig_out(i2c.port, 400, i2c.addr, 500);

                      I64 outInt = (I64) out;
                      if (outInt > i2c.cmdOutputMap.maxOut) {  outInt = i2c.cmdOutputMap.maxOut;  }
                      if (outInt < i2c.cmdOutputMap.minOut) {  outInt = i2c.cmdOutputMap.minOut;  }

                      Fw::Buffer readBufObj(0, 0, 0, 0); // no read
                      if ((OUTPUT_I2C       == this->outputInfo[i].type) ||
                          (OUTPUT_I2C_SHORT == this->outputInfo[i].type)) {
                          if (i2c.reverse) {  outInt = -outInt;  }
                          
                          I8 outI8[2] = { (I8) (outInt >> 8), (I8) outInt };
                          U8 outU8[2] = { 0 };
                          memcpy(outU8, outI8, 2);
                          // MSB is reverse bit

                          DEBUG_PRINT("reverse: %u, ihigh: %d, ilow: %d, high: %u, low: %u\n",
                          i2c.reverse, outI8[0], outI8[1],
                          outU8[0], outU8[1]);

                          U8 writeBuf[3] = { 0x00, outU8[0], outU8[1] };
                          Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                          this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
                      }
                      else { // simple protocol
                          if (outInt < 0) {  outInt = 0;  }
                          U8 writeBuf[2] = { (U8) (outInt / 8), (U8) (outInt % 8) };
                          Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                          this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
                      }

                      this->outputInfo[i].feedback.cmdIn   = inVal;
                      this->outputInfo[i].feedback.cmd     = out;
                      this->outputInfo[i].feedback.cmdSec  = cmdTime.getSeconds();
                      this->outputInfo[i].feedback.cmdUsec = cmdTime.getUSeconds();
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

      // get feedback - higher latency OK
      for (U32 i = 0; i < FW_MIN(this->numActuators, ACTADAP_MAX_ACTUATORS); i++) {
          switch (this->outputInfo[i].type) {
              case OUTPUT_UNSET:
              case OUTPUT_PWM:
                  //NOTE(mereweth) - no way to get feedback
                  break;
              case OUTPUT_I2C:
              case OUTPUT_I2C_SIMPLE:
              case OUTPUT_I2C_SHORT:
              {
                  I2CMetadata i2c = this->outputInfo[i].i2cMeta;
                  if (this->isConnected_escConfig_OutputPort(i2c.port) &&
                      this->isConnected_escReadWrite_OutputPort(i2c.port)) {
                      Fw::Time fbTime = this->getTime();
                      F64 fbTimeLast = (F64) this->outputInfo[i].feedback.fbSec +
                                       (F64) this->outputInfo[i].feedback.fbUsec * 0.001 * 0.001;
                      U32 countsLast = this->outputInfo[i].feedback.counts;

                      // TODO(mereweth) - put the I2C clock speed in config header? separate config ports?
                      this->escConfig_out(i2c.port, 400, i2c.addr, 500);

                      U8 readBuf[9] = { 0 };

                      if (OUTPUT_I2C == this->outputInfo[i].type) {
                          U8 writeBuf[1] = { 0x02 }; // start at rev_count_h
                          Fw::Buffer readBufObj(0, 0, (U64) readBuf, FW_NUM_ARRAY_ELEMENTS(readBuf));
                          Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                          this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
                      }
                      else if (OUTPUT_I2C_SHORT == this->outputInfo[i].type) {
                          U8 writeBuf[1] = { 0x02 }; // start at rev_count_h
                          Fw::Buffer readBufObj(0, 0, (U64) readBuf, 2); // only read rev counter
                          Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                          this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
                      }
                      else { // simple protocol
                          Fw::Buffer readBufObj(0, 0, (U64) readBuf, 2);
                          Fw::Buffer writeBufObj(0, 0, 0, 0); // no write in simple protocol
                          this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
                      }

                      if ((0xab == readBuf[8])                             ||
                          (OUTPUT_I2C_SHORT  == this->outputInfo[i].type)  ||
                          (OUTPUT_I2C_SIMPLE == this->outputInfo[i].type)) {
                          this->outputInfo[i].feedback.counts      = (readBuf[0] << 8) + readBuf[1];
                          // TODO(mereweth) - establish units and validity
                          this->outputInfo[i].feedback.voltage     = (F32) ((readBuf[2] << 8) + readBuf[3]) / 2016.0;
                          this->outputInfo[i].feedback.temperature = (F32) ((readBuf[4] << 8) + readBuf[5]);
                          this->outputInfo[i].feedback.current     = ((F32) ((readBuf[6] << 8) + readBuf[7]) - 32767.0) / 891.0;
                          this->outputInfo[i].feedback.fbSec       = fbTime.getSeconds();
                          this->outputInfo[i].feedback.fbUsec      = fbTime.getUSeconds();

                          F64 fbTimeFloat = (F64) this->outputInfo[i].feedback.fbSec +
                                            (F64) this->outputInfo[i].feedback.fbUsec * 0.001 * 0.001;

                          // guard against bad parameter setting and small time increment
                          if ((this->outputInfo[i].i2cMeta.fbMeta.countsPerRev > 0) &&
                              (fbTimeFloat - fbTimeLast > 1e-4)) {

                              if ((OUTPUT_I2C       == this->outputInfo[i].type)  ||
                                  (OUTPUT_I2C_SHORT == this->outputInfo[i].type)) {
                                  this->outputInfo[i].feedback.angVel = 2.0 * M_PI
                                      * this->outputInfo[i].feedback.counts
                                      / (fbTimeFloat - fbTimeLast) / this->outputInfo[i].i2cMeta.fbMeta.countsPerRev;

                                  DEBUG_PRINT("esc id: %u, counts: %u, angVel: %f\n",
                                              i, this->outputInfo[i].feedback.counts,
                                              this->outputInfo[i].feedback.angVel);
                              }
                              else {
                                  U32 countsDiff = 0u;
                                  //To account for the CSC rolling over at 0xFFFF = 2^16-1
                                  if (countsLast > this->outputInfo[i].feedback.counts) {
                                      countsDiff = 0xFFFF - countsLast + this->outputInfo[i].feedback.counts;
                                  }
                                  else {
                                      countsDiff = this->outputInfo[i].feedback.counts - countsLast;
                                  }

                                  this->outputInfo[i].feedback.angVel = 2.0 * M_PI * countsDiff
                                      / (fbTimeFloat - fbTimeLast) / this->outputInfo[i].i2cMeta.fbMeta.countsPerRev;

                                  DEBUG_PRINT("esc id: %u, counts: %u, angVel: %f\n",
                                              i, countsDiff,
                                              this->outputInfo[i].feedback.angVel);
                              }

                              // TODO(mereweth) - run rate estimator here
                          }
                      }

                      Fw::SerializeStatus status;
                      // sec, usec, motor id, command, response
                      U8 buff[sizeof(U8) + 2 * sizeof(U32) + 2 * sizeof(F64)
                              + 2 * sizeof(U32) + sizeof(F64)
                              + sizeof(readBuf) + sizeof(U32)];
                      Fw::SerialBuffer buffObj(buff, FW_NUM_ARRAY_ELEMENTS(buff));
                      status = buffObj.serialize((U8) i);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));

                      status = buffObj.serialize(this->outputInfo[i].feedback.cmdSec);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      status = buffObj.serialize(this->outputInfo[i].feedback.cmdUsec);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      status = buffObj.serialize(this->outputInfo[i].feedback.cmd);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      status = buffObj.serialize(this->outputInfo[i].feedback.cmdIn);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));

                      status = buffObj.serialize(this->outputInfo[i].feedback.fbSec);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      status = buffObj.serialize(this->outputInfo[i].feedback.fbUsec);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      status = buffObj.serialize(this->outputInfo[i].feedback.angVel);
                      FW_ASSERT(Fw::FW_SERIALIZE_OK == status,static_cast<AssertArg>(status));
                      NATIVE_INT_TYPE size = FW_NUM_ARRAY_ELEMENTS(readBuf);
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
      bool hwEnabled = true;
      if (this->isConnected_outputEnable_OutputPort(0)) {
          this->outputEnable_out(0, hwEnabled);
      }

      // NOTE(mereweth) - don't increment cycle count here; do that in motor_handler
      const bool didFlySafeCycleTimeout = (this->flySafeCheckCycles &&
                                           (this->flySafeCycles > this->flySafeMaxElapsedCycles));
      const bool didFlySafeTimeTimeout = (this->flySafeCheckTime &&
                                          (this->getTime() > Fw::Time::add(this->flySafeMaxElapsedTime,
                                                                           this->flySafeLastTime)));
      if ((DISARMED != this->armedState) &&
          (!paramsInited || !hwEnabled ||
           didFlySafeCycleTimeout || didFlySafeTimeTimeout || !flySafe)) {

          if (ARMING == this->armedState) {
              this->cmdResponse_out(this->opCode, this->cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          }
          this->sendZeroCmdAll();
          this->armedState = DISARMED;
          if (!this->paramsInited) {
              this->log_WARNING_HI_ACTADAP_Error(ParamsUninit);
          }
          if (!hwEnabled) {
              this->log_WARNING_HI_ACTADAP_Error(HWEnableLow);
          }
          if (didFlySafeCycleTimeout) {
              this->log_WARNING_HI_ACTADAP_NotFlySafe(CycleTimeout);
          }
          if (didFlySafeTimeTimeout) {
              this->log_WARNING_HI_ACTADAP_NotFlySafe(TimeTimeout);
          }
          if (!this->flySafe) {
              this->log_WARNING_HI_ACTADAP_NotFlySafe(ValueFalse);
          }
          return;
      }

      if (ACTADAP_SCHED_CONTEXT_TLM == context) {
          this->tlmWrite_ACTADAP_ArmState(static_cast<ArmStateTlm>(this->armedState));
          this->tlmWrite_ACTADAP_HwEnabled(hwEnabled);

          this->tlmWrite_ACTADAP_Rot0(this->outputInfo[0].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd0(this->outputInfo[0].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel0(this->outputInfo[0].feedback.cmdIn);

          this->tlmWrite_ACTADAP_Rot1(this->outputInfo[1].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd1(this->outputInfo[1].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel1(this->outputInfo[1].feedback.cmdIn);

          this->tlmWrite_ACTADAP_Rot2(this->outputInfo[2].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd2(this->outputInfo[2].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel2(this->outputInfo[2].feedback.cmdIn);

          this->tlmWrite_ACTADAP_Rot3(this->outputInfo[3].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd3(this->outputInfo[3].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel3(this->outputInfo[3].feedback.cmdIn);

          this->tlmWrite_ACTADAP_Rot4(this->outputInfo[4].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd4(this->outputInfo[4].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel4(this->outputInfo[4].feedback.cmdIn);

          this->tlmWrite_ACTADAP_Rot5(this->outputInfo[5].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd5(this->outputInfo[5].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel5(this->outputInfo[5].feedback.cmdIn);

          this->tlmWrite_ACTADAP_Rot6(this->outputInfo[6].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd6(this->outputInfo[6].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel6(this->outputInfo[6].feedback.cmdIn);

          this->tlmWrite_ACTADAP_Rot7(this->outputInfo[7].feedback.angVel);
          this->tlmWrite_ACTADAP_Cmd7(this->outputInfo[7].feedback.cmd);
          this->tlmWrite_ACTADAP_CmdVel7(this->outputInfo[7].feedback.cmdIn);
      }
      else if ((ACTADAP_SCHED_CONTEXT_ARM == context) &&
               (ARMING == this->armedState)) {
          if (++this->armCount > ACTADAP_ARM_COUNT) {
              this->armedState = ARMED;
              this->cmdResponse_out(this->opCode, this->cmdSeq, Fw::COMMAND_OK);
              return;
          }
          for (U32 i = 0; i < FW_MIN(this->numActuators, ACTADAP_MAX_ACTUATORS); i++) {
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
                  case OUTPUT_I2C_SHORT:
                  case OUTPUT_I2C_SIMPLE:
                  {
                      I2CMetadata i2c = this->outputInfo[i].i2cMeta;
                      if (this->isConnected_escConfig_OutputPort(i2c.port) &&
                          this->isConnected_escReadWrite_OutputPort(i2c.port)) {
                          // TODO(mereweth) - put the I2C clock speed in config header? separate config ports?
                          this->escConfig_out(i2c.port, 400, i2c.addr, 500);

                          Fw::Buffer readBufObj(0, 0, 0, 0); // no read
                          if ((OUTPUT_I2C       == this->outputInfo[i].type)  ||
                              (OUTPUT_I2C_SHORT == this->outputInfo[i].type)) {
                              // MSB is reverse bit
                              U8 writeBuf[3] = { 0 };
                              Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                              this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
                          }
                          else { // simple protocol
                              U8 writeBuf[1] = { 0 };
                              Fw::Buffer writeBufObj(0, 0, (U64) writeBuf, FW_NUM_ARRAY_ELEMENTS(writeBuf));
                              this->escReadWrite_out(i2c.port, writeBufObj, readBufObj);
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
      bool hwEnabled = true;
      if (this->isConnected_outputEnable_OutputPort(0)) {
          this->outputEnable_out(0, hwEnabled);
      }

      // can disarm immediately - change this if hardware changes
      if (!armState) {
          if (ARMED == this->armedState) {
              this->sendZeroCmdAll();
          }

          this->armedState = DISARMED;
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
          return;
      }
      // after this point, we are trying to arm

      if (!this->paramsInited || !hwEnabled) {
          if (ARMED == this->armedState) {
              this->sendZeroCmdAll();
          }
          this->armedState = DISARMED;
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          if (!this->paramsInited) {
              this->log_WARNING_HI_ACTADAP_Error(ParamsUninit);
          }
          if (!hwEnabled) {
              this->log_WARNING_HI_ACTADAP_Error(HWEnableLow);
          }
          return;
      }
      // can only arm if disarmed
      if (DISARMED != this->armedState) {
          this->log_WARNING_LO_ACTADAP_AlreadyArmed();
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }

      // if we get here, either we are disarmed now, or about to disarm
      this->armCount = 0u;

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

  void ActuatorAdapterComponentImpl ::
    ACTADAP_SetVoltAct_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        U8 actIdx,
        F64 voltage
    )
  {
      if ((!this->paramsInited) ||
          (actIdx > this->numActuators) ||
          (voltage < 0.0)) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }

      switch (this->outputInfo[actIdx].type) {
          case OUTPUT_UNSET:
          {
              //TODO(mereweth) - EVR that this is unset
          }
              break;
          case OUTPUT_PWM:
          {
              this->outputInfo[actIdx].pwmMeta.cmdOutputMap.Vact = voltage;
          }
              break;
          case OUTPUT_I2C:
          case OUTPUT_I2C_SHORT:
          case OUTPUT_I2C_SIMPLE:
          {
              this->outputInfo[actIdx].i2cMeta.cmdOutputMap.Vact = voltage;
          }
              break;
          default:
              //TODO(mereweth) - DEBUG_PRINT
              FW_ASSERT(0, this->outputInfo[actIdx].type);
      }

      this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }
} // end namespace Gnc
