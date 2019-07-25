// ====================================================================== 
// \title  ActiveL2PrmDbImpl.hpp
// \author kubiak
// \brief  hpp file for ActiveL2PrmDb component implementation class
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

#ifndef ActiveL2PrmDb_HPP
#define ActiveL2PrmDb_HPP

#include "Svc/ActiveL2PrmDb/ActiveL2PrmDbComponentAc.hpp"
#include <Svc/ActiveL2PrmDb/ActiveL2PrmDbImplCfg.hpp>
#include <Svc/PrmDb/PrmDbImpl.hpp>

namespace Svc {

  class ActiveL2PrmDbComponentImpl :
    public ActiveL2PrmDbComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ActiveL2PrmDb
      //!
      ActiveL2PrmDbComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const NATIVE_INT_TYPE maxRecvSize
#else
          const NATIVE_INT_TYPE maxRecvSize
#endif
      );

      //! Initialize object ActiveL2PrmDb
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ActiveL2PrmDb
      //!
      ~ActiveL2PrmDbComponentImpl(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for sendPrmReady
      //!
      void sendPrmReady_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 maxSize,
          bool resend
      );

      //! Handler implementation for setPrm
      //!
      void setPrm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, /*!< Parameter ID*/
          Fw::ParamBuffer &val /*!< Buffer containing serialized parameter value*/
      );

      //! Handler implementation for recvPrm
      //!
      void recvPrm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool morePrms, 
          Fw::ParamList val
      );

      //! Handler implementation for pingIn
      //!
      void pingIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Handler implementation for getPrm
      //!
      Fw::ParamValid getPrm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, /*!< Parameter ID*/
          Fw::ParamBuffer &val /*!< Buffer containing serialized parameter value*/
      );

      //! Handler implementation for getPrm
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum,
          NATIVE_UINT_TYPE context
      );

      //! Handler for command L2_PRM_PUSH_UPDATES
      /* Push parameter updates to the L1 Prm Db */
      void L2_PRM_PUSH_UPDATES_cmdHandler(
          FwOpcodeType opCode, /*!< The opcode*/
          U32 cmdSeq, /*!< The command sequence number*/
          UpdateMethod updateMethod 
      );


      void queuePrm(FwPrmIdType id);

      NATIVE_INT_TYPE sendPrms(void);

      enum L2PrmUpdateMethod {
        ACTIVE_L2_PUSH_UPDATES,
        ACTIVE_L2_NO_UPDATES
      };

      const NATIVE_INT_TYPE m_maxRecvSize;

      PrmDbImpl m_prmDb;

      FwPrmIdType m_prmSendBuffer[ACTIVE_L2_PRMDB_SEND_BUFFER_ENTRIES];
      NATIVE_UINT_TYPE m_prmSendBufferIdx;

      NATIVE_UINT_TYPE m_prmSendReadySize;

      Fw::ParamList m_sendListBuffer;

      L2PrmUpdateMethod m_updateMethod;

      bool m_firstSched;

      bool m_prmTriggered;

    };

} // end namespace Svc

#endif
