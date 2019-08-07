// ======================================================================
// \title  WrenchMixerImpl.hpp
// \author mereweth
// \brief  hpp file for WrenchMixer component implementation class
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

#ifndef WrenchMixer_HPP
#define WrenchMixer_HPP

#include "Gnc/Ctrl/WrenchMixer/WrenchMixerComponentAc.hpp"
#include "quest_gnc/mixer/wrench_mixer.h"
#include "quest_gnc/mixer/wrench_mixer_cfg.h"

namespace Gnc {

  class WrenchMixerComponentImpl :
    public WrenchMixerComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object WrenchMixer
      //!
      WrenchMixerComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object WrenchMixer
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object WrenchMixer
      //!
      ~WrenchMixerComponentImpl(void);

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PRIVATE:
      void parameterUpdated(FwPrmIdType id /*!< The parameter ID*/);

      void parametersLoaded();

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for prmTrigger
      //!
      void prmTrigger_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType dummy 
      );

      //! Handler implementation for controls
      //!
      void controls_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::geometry_msgs::WrenchStamped &WrenchStamped 
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

      //! Implementation for WMIX_InitParams command handler
      //!
      void WMIX_InitParams_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      quest_gnc::multirotor::WrenchMixer wrenchMixer;

      bool paramsInited;

      U32 numRotors;

      F64 angVelTlm[quest_gnc::multirotor::kWrenchMixerMaxActuators];

    };

} // end namespace Gnc

#endif
