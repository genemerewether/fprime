// ======================================================================
// \title  ActuatorAdapterImpl.hpp
// \author mereweth
// \brief  hpp file for ActuatorAdapter component implementation class
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

#ifndef ActuatorAdapter_HPP
#define ActuatorAdapter_HPP

#include "Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentAc.hpp"
#include "Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImplCfg.hpp"

namespace Gnc {

  class ActuatorAdapterComponentImpl :
    public ActuatorAdapterComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ActuatorAdapter
      //!
      ActuatorAdapterComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object ActuatorAdapter
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ActuatorAdapter
      //!
      ~ActuatorAdapterComponentImpl(void);

      // TODO(mereweth) - check for near-zero and zero in this setter
      struct PwmMetadata {
          F64 minIn;
          F64 maxIn;
          F32 minOut;
          F32 maxOut;
      };

      struct I2CMetadata {
          U32 addr;
          F64 minIn;
          F64 maxIn;
          U32 minOut;
          U32 maxOut;
          bool reverse;
          U32 countsPerRev;
      };

      bool setupI2C(U32 actuator, I2CMetadata meta);

      void parameterUpdated(FwPrmIdType id /*!< The parameter ID*/);

      void parametersLoaded();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for motor
      //!
      void motor_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::mav_msgs::Actuators &Actuators
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations
      // ----------------------------------------------------------------------

      //! Implementation for ACTADAP_Arm command handler
      //!
      void ACTADAP_Arm_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          bool armState
      );

      //! Implementation for ACTADAP_InitParams command handler
      //!
      void ACTADAP_InitParams_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      enum OutputType {
          OUTPUT_UNSET = 0,
          OUTPUT_VALID_MIN = 1,
          OUTPUT_PWM = OUTPUT_VALID_MIN,
          OUTPUT_I2C = 2,
          OUTPUT_VALID_MAX = OUTPUT_I2C
      };

      struct Feedback {
          U32 cmdSec;
          U32 cmdUsec;
          U32 cmd;
          U32 fbSec;
          U32 fbUsec;
          U32 counts;
          F64 angVel;
          F32 voltage; // voltage of supply
          F32 temperature; // celsius?
          F32 current; // amps?
      };

      struct OutputInfo {
          OutputType type;
          union {
              PwmMetadata pwmMeta;
              I2CMetadata i2cMeta;
          };
          Feedback feedback;
      } outputInfo[AA_MAX_ACTUATORS];

      enum ArmingState {
          DISARMED,
          ARMING,
          ARMED
      } armedState;

      FwOpcodeType opCode;

      U32 cmdSeq;

      U32 armCount;

      U32 numActuators;

      bool paramsInited;

    };

} // end namespace Gnc

#endif
