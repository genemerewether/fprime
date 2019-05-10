// ====================================================================== 
// \title  SigGenImpl.cpp
// \author mereweth
// \brief  cpp file for SigGen component implementation class
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


#include <Gnc/Sysid/SigGen/SigGenComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Gnc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  SigGenComponentImpl ::
#if FW_OBJECT_NAMES == 1
    SigGenComponentImpl(
        const char *const compName
    ) :
      SigGenComponentBase(compName),
#else
      SigGenImpl(void),
#endif
      paramsInited(false),
      dt(0.0),
      sigType(IDLE),
      outputMode(ACTUATOR_ANGLE),
      offset(0.0),
      actuatorIdx(0u),
      seq(0u),
      opCode(0u),
      cmdSeq(0u),
      signalGen()
  {

  }

  void SigGenComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    SigGenComponentBase::init(instance);
  }

  SigGenComponentImpl ::
    ~SigGenComponentImpl(void)
  {

  }

  void SigGenComponentImpl ::
    parameterUpdated(FwPrmIdType id)
  {
#ifndef BUILD_TIR5
    printf("prm %d updated\n", id);
#endif
  }

  void SigGenComponentImpl ::
    parametersLoaded()
  {
      Fw::ParamValid valid[1];
      this->dt = paramGet_SIGGEN_dt(valid[0]);
      if (Fw::PARAM_VALID != valid[0]) {  return;  }
      // TODO(mereweth) - shared constant for small number
      if (this->dt < 1e-6) {  return;  }
      
      paramsInited = true;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void SigGenComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      if (SIGGEN_SCHED_CONTEXT_TLM == context) {
      }
      else if (SIGGEN_SCHED_CONTEXT_OP == context) {
          if (IDLE == this->sigType) {
              return;
          }

          if ((ACTUATOR_ANGLE == this->outputMode) ||
              (ACTUATOR_ANGVEL == this->outputMode)) {
              F64 angle[SIGGEN_MAX_ACTUATORS] = {0.0};
              F64 angVel[SIGGEN_MAX_ACTUATORS] = {0.0};
              NATIVE_INT_TYPE stat = 0;
              if (this->actuatorIdx >= SIGGEN_MAX_ACTUATORS) {
                  this->sigType = IDLE;
                  this->cmdResponse_out(this->opCode, this->cmdSeq,
                                        Fw::COMMAND_EXECUTION_ERROR);
                  // TODO(mereweth) - issue EVR
                  return;
              }
              FW_ASSERT(SIGGEN_MAX_ACTUATORS > this->actuatorIdx,
                        SIGGEN_MAX_ACTUATORS, this->actuatorIdx);
              F64 temp;
              F64 value;
              stat = this->signalGen.GetScalar(&value,
                                               &temp);
              value += offset;
              
              if (ACTUATOR_ANGLE == this->outputMode) {
                  angle[this->actuatorIdx] = value;
              }
              else if (ACTUATOR_ANGVEL == this->outputMode) {
                  angVel[this->actuatorIdx] = value;
              }
              
              if (stat) {
                  this->sigType = IDLE;
                  // TODO(mereweth) - issue EVR
                  // TODO(mereweth) - check for "done" return code only
                  // TODO(mereweth) - set actuator back to home setpoint (not zero)
                  this->cmdResponse_out(this->opCode, this->cmdSeq,
                                        Fw::COMMAND_OK);

                  angle[this->actuatorIdx] = 0.0;
                  angVel[this->actuatorIdx] = 0.0;
              }

              ROS::std_msgs::Header h(this->seq++,
                                      this->getTime(),
                                      0);

              ROS::mav_msgs::Actuators actuators__comm;
              actuators__comm.set(h,
                                  angle, SIGGEN_MAX_ACTUATORS, SIGGEN_MAX_ACTUATORS,
                                  // TODO(mereweth) - specify actuator count?
                                  angVel, SIGGEN_MAX_ACTUATORS, SIGGEN_MAX_ACTUATORS,
                                  NULL, 0, 0);
              if (this->isConnected_motor_OutputPort(0)) {
                  this->motor_out(0, actuators__comm);
              }
          }
      }
      else {
          //TODO(mereweth) - assert invalid context
      }
  }

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void SigGenComponentImpl ::
    SIGGEN_SetChirp_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F64 omega_i,
        F64 omega_f,
        F64 amplitude,
        F64 duration,
        F64 offset
    )
  {
      if (!this->paramsInited ||
          IDLE != this->sigType) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
      int stat = signalGen.SetChirp(omega_i, omega_f, amplitude,
                                    static_cast<U32>(duration / this->dt),
                                    this->dt);
      this->offset = offset;
      if (stat) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
      this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void SigGenComponentImpl ::
    SIGGEN_SetAxis_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F64 x,
        F64 y,
        F64 z
    )
  {
      if (!this->paramsInited ||
          IDLE != this->sigType) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
      int stat = signalGen.SetUnitAxis(Eigen::Vector3d(x, y, z).normalized());
      if (stat) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
      this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void SigGenComponentImpl ::
    SIGGEN_SetRamp_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        F64 amplitude,
        F64 halfDuration,
        F64 offset
    )
  {
      if (!this->paramsInited ||
          IDLE != this->sigType) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
      int stat = signalGen.SetRamp(amplitude,
                                   static_cast<U32>(2.0 * halfDuration / this->dt),
                                   this->dt);
      this->offset = offset;
      if (stat) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
      this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }
  
  void SigGenComponentImpl ::
    SIGGEN_Start_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq,
        OutputMode outputMode,
        SignalType signalType,
        U8 index
    )
  {
      if (!this->paramsInited ||
          IDLE != this->sigType) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_EXECUTION_ERROR);
          return;
      }
      
      this->opCode = opCode;
      this->cmdSeq = cmdSeq;

      this->sigType = signalType;
      this->actuatorIdx = index;
      this->outputMode = outputMode;

      if (IDLE == this->sigType) {
          this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
      }      
  }

  void SigGenComponentImpl ::
    SIGGEN_Cancel_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
      // NOTE(Mereweth) - notify that previous command did not finish
      if (IDLE != this->sigType) {
          this->cmdResponse_out(this->opCode, this->cmdSeq,
                                Fw::COMMAND_EXECUTION_ERROR);
      }
      this->sigType = IDLE;
      this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
  }

  void SigGenComponentImpl ::
    SIGGEN_InitParams_cmdHandler(
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
