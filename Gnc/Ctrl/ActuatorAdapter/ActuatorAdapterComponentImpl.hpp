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
      };

      bool setupI2C(U32 actuator, I2CMetadata meta);
    
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

      enum OutputType {
          OUTPUT_UNSET,
          OUTPUT_PWM,
          OUTPUT_I2C
      };

      struct OutputInfo {
          OutputType type;
          union {
              PwmMetadata pwmMeta;
              I2CMetadata i2cMeta;
          };
      } outputInfo[AA_MAX_ACTUATORS];
    
    };

} // end namespace Gnc

#endif
