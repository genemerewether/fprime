// ====================================================================== 
// \title  SigGenImpl.hpp
// \author mereweth
// \brief  hpp file for SigGen component implementation class
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

#ifndef SigGen_HPP
#define SigGen_HPP

#include "Gnc/Sysid/SigGen/SigGenComponentAc.hpp"
#include "Gnc/Sysid/SigGen/SigGenComponentImplCfg.hpp"

#include "quest_gnc/sysid/signal_gen.h"

namespace Gnc {

  class SigGenComponentImpl :
    public SigGenComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object SigGen
      //!
      SigGenComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object SigGen
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object SigGen
      //!
      ~SigGenComponentImpl(void);

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PRIVATE:
      void parameterUpdated(FwPrmIdType id /*!< The parameter ID*/);

      void parametersLoaded();

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      // ----------------------------------------------------------------------
      // Command handler implementations 
      // ----------------------------------------------------------------------

      //! Implementation for SIGGEN_SetChirp command handler
      //! 
      void SIGGEN_SetChirp_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          F64 omega_i, 
          F64 omega_f, 
          F64 amplitude, 
          F64 duration,
          F64 offset
      );

      //! Implementation for SIGGEN_SetAxis command handler
      //! 
      void SIGGEN_SetAxis_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          F64 x, 
          F64 y, 
          F64 z 
      );

      //! Implementation for SIGGEN_DoChirp command handler
      //! 
      void SIGGEN_DoChirp_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          ChirpMode mode, 
          U8 index 
      );

      //! Implementation for SIGGEN_Cancel command handler
      //! 
      void SIGGEN_Cancel_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      bool paramsInited;

      F64 dt;

      enum SignalType {
          IDLE,
          CHIRP,
          STEP
      } sigType;

      // TODO(mereweth) - home setpoints for each actuator

      ChirpMode chirpMode;
      F64 offset;
      U32 actuatorIdx;
      U32 seq;

      FwOpcodeType opCode;

      U32 cmdSeq;

      quest_gnc::sysid::SignalGen signalGen;

    };

} // end namespace Gnc

#endif
