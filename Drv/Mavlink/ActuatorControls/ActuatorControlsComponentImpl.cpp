// ======================================================================
// \title  ActuatorControlsComponentImpl.cpp
// \author mereweth
// \brief  cpp file for ActuatorControls component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <Drv/Mavlink/ActuatorControls/ActuatorControlsComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <float.h>

#define MAVLINK_NO_CONVERSION_HELPERS

#ifdef BUILD_TIR5

#ifndef __GNUC__
#define __GNUC__
#include <Drv/Mavlink/c_library_v2/mavlink_types.h>
#undef __GNUC__
#endif //__GNUC__

#else //BUILD_TIR5

#include <Drv/Mavlink/c_library_v2/mavlink_types.h>

#endif //BUILD_TIR5

extern mavlink_system_t mavlink_system;

#include <Drv/Mavlink/c_library_v2/standard/mavlink.h>


#include <stdio.h>

#ifdef BUILD_DSPAL
#include <HAP_farf.h>
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#include <stdio.h>
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

#undef DEBUG_PRINT
#define DEBUG_PRINT(x,...)

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  ActuatorControlsComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ActuatorControlsComponentImpl(
        const char *const compName
    ) :
      ActuatorControlsComponentBase(compName),
#else
      ActuatorControlsComponentImpl(void),
#endif
      minThrust(0.0),
      maxThrust(FLT_MAX),
      paramsInited(false),
      m_outputBufObj(0, 0, (U64) m_outputBuf, sizeof(m_outputBuf))
  {

  }

  void ActuatorControlsComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    ActuatorControlsComponentBase::init(instance);
  }

  ActuatorControlsComponentImpl ::
    ~ActuatorControlsComponentImpl(void)
  {

  }

    void ActuatorControlsComponentImpl ::
      parameterUpdated(FwPrmIdType id)
    {
        DEBUG_PRINT("prm %d updated\n", id);
        Fw::ParamValid valid;

        switch (id) {
            // default params
          
            case PARAMID_ACTCTRL_MINTHRUST:
            {
                F64 temp = paramGet_ACTCTRL_minThrust(valid);
                if ((Fw::PARAM_VALID == valid) ||
                    (Fw::PARAM_DEFAULT == valid)) {
                    this->minThrust = temp;
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
                
            // non-default params
          
            case PARAMID_ACTCTRL_MAXTHRUST:
            {
                F64 temp = paramGet_ACTCTRL_maxThrust(valid);
                if (Fw::PARAM_VALID == valid) {
                    this->maxThrust = temp;
                }
                else {
                    // TODO(mereweth) - issue EVR
                }
            }
                break;
        }
    }
  
    void ActuatorControlsComponentImpl ::
      parametersLoaded()
    {
        this->paramsInited = false;
        for (U32 i = 0; i < __MAX_PARAMID; i++) {
            parameterUpdated(i);
        }
        if (this->maxThrust - this->minThrust < 1e-3) {
            return;
        }
        this->paramsInited = true;
    }
  
  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ActuatorControlsComponentImpl ::
    prmTrigger_handler(
        const NATIVE_INT_TYPE portNum,
        FwPrmIdType dummy
    )
  {
      this->loadParameters();
  }
 
  void ActuatorControlsComponentImpl ::
    rateThrust_handler(
        const NATIVE_INT_TYPE portNum,
        ROS::mav_msgs::RateThrust &RateThrust
    )
  {
      if (!this->paramsInited) {
          // TODO(mereweth) - EVR
          return;
      }
      if (this->maxThrust - this->minThrust < 1e-3) {
          // TODO(mereweth) - EVR - too small or negative
          return;
      }
      
      mavlink_set_attitude_target_t sp;
      sp.time_boot_ms = 0;
      sp.target_system = 0;
      sp.target_component = 0;
      sp.q[0] = 1.0;
      sp.q[1] = 0.0;
      sp.q[2] = 0.0;
      sp.q[3] = 0.0;

      ROS::geometry_msgs::Vector3 rates = RateThrust.getangular_rates();
      sp.type_mask = 128;
      sp.body_roll_rate = rates.getx();
      sp.body_pitch_rate = rates.gety();
      sp.body_yaw_rate = rates.getz();

      F64 thrust = RateThrust.getthrust().getz();
      if (thrust <= this->minThrust) {
          // TODO(mereweth) - EVR
          sp.thrust = 0;
      }
      else if (thrust >= this->maxThrust) {
          // TODO(mereweth) - EVR
          sp.thrust = 1;
      }
      else {
          sp.thrust = (thrust - this->minThrust) / (this->maxThrust - this->minThrust);
      }

      mavlink_message_t message;
      // TODO(mereweth) - system and component ID
      mavlink_msg_set_attitude_target_encode(0, 0, &message, &sp);
      
      unsigned len = mavlink_msg_to_send_buffer((uint8_t*)m_outputBuf, &message);
      this->m_outputBufObj.setsize(len);
      this->SerWritePort_out(0, this->m_outputBufObj);
  }

  void ActuatorControlsComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO
  }

  void ActuatorControlsComponentImpl ::
    SerReadPort_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &serBuffer,
        SerialReadStatus &status
    )
  {
    // TODO
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void ActuatorControlsComponentImpl ::
    ACTCTRL_InitParams_cmdHandler(
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

} // end namespace Drv
