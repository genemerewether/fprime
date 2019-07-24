// ====================================================================== 
// \title  PassiveL2PrmDbImpl.cpp
// \author kubiak
// \brief  cpp file for PassiveL2PrmDb component implementation class
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


#include <Svc/PassiveL2PrmDb/PassiveL2PrmDbImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  PassiveL2PrmDbComponentImpl ::
#if FW_OBJECT_NAMES == 1
    PassiveL2PrmDbComponentImpl(
        const char *const compName,
        const NATIVE_INT_TYPE maxRecvSize
    ) :
      PassiveL2PrmDbComponentBase(compName),
#else
      PassiveL2PrmDbImpl(const NATIVE_INT_TYPE maxRecvSize) :
#endif
      m_maxRecvSize(maxRecvSize),
      m_prmDb(),
      m_prmSendBuffer(),
      m_prmSendBufferIdx(),
      m_prmSendReadySize(0),
      m_sendListBuffer(),
      m_updateMethod(PASSIVE_L2_NO_UPDATES),
      m_firstSched(true)
  {

  }

  void PassiveL2PrmDbComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    PassiveL2PrmDbComponentBase::init(instance);
  }

  PassiveL2PrmDbComponentImpl ::
    ~PassiveL2PrmDbComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void PassiveL2PrmDbComponentImpl ::
    sendPrmReady_handler(
        const NATIVE_INT_TYPE portNum,
        U32 maxSize,
        bool resend
    )
  {
    this->m_prmSendReadySize = maxSize;

    FwPrmIdType dummy = 0;
    for (NATIVE_INT_TYPE port = 0; port < NUM_PRMTRIGGER_OUTPUT_PORTS; port++) {
        if (this->isConnected_prmTrigger_OutputPort(port)) {
            this->prmTrigger_out(port, dummy);
        }
    }
  }

  void PassiveL2PrmDbComponentImpl ::
    setPrm_handler(
        const NATIVE_INT_TYPE portNum,
        FwPrmIdType id,
        Fw::ParamBuffer &val
    )
  {
    PrmDbImpl::SetPrmStatus stat;

    stat = this->m_prmDb.setPrm(id, val);

    if (this->m_updateMethod == PASSIVE_L2_PUSH_UPDATES) {
        this->queuePrm(id);
    }

    if (stat == PrmDbImpl::SET_PRM_UPDATED) {
        this->log_ACTIVITY_HI_PrmIdUpdated(id);
    } else if (stat == PrmDbImpl::SET_PRM_ADDED) {
        this->log_ACTIVITY_HI_PrmIdAdded(id);
    } else if (stat == PrmDbImpl::SET_PRM_FULL) {
        this->log_WARNING_HI_PrmDbFull(id);
    } else {
        FW_ASSERT(false, stat);
    }

  }

  void PassiveL2PrmDbComponentImpl ::
    recvPrm_handler(
        const NATIVE_INT_TYPE portNum,
        bool morePrms,
        Fw::ParamList val
    )
  {
    PrmDbImpl::DeserializePrmStatus desPrmStat;
    PrmDbImpl::SetPrmStatus setStat;

    FwPrmIdType prmId;
    Fw::ParamBuffer prmBuff;

    if (!this->m_firstSched) {

        while (val.getBuffLeft() > 0) {

            desPrmStat = this->m_prmDb.deserializePrm(val, prmId, prmBuff);
            FW_ASSERT(PrmDbImpl::DESERIALIZE_PRM_OK == desPrmStat, desPrmStat);

            prmBuff.resetDeser();
            setStat = this->m_prmDb.setPrm(prmId, prmBuff);
            FW_ASSERT(PrmDbImpl::SET_PRM_ADDED == setStat, setStat);
        }
    }

    this->recvPrmReady_out(portNum, this->m_maxRecvSize, false);

  }

  Fw::ParamValid PassiveL2PrmDbComponentImpl ::
    getPrm_handler(
        const NATIVE_INT_TYPE portNum,
        FwPrmIdType id,
        Fw::ParamBuffer &val
    )
  {
    Fw::ParamValid valid;

    valid = this->m_prmDb.getPrm(id, val);
    
    if (valid == Fw::PARAM_INVALID) {
        this->log_WARNING_LO_PrmIdNotFound(id);
    }

    return valid;
  }

  void PassiveL2PrmDbComponentImpl ::
    sched_handler(
      const NATIVE_INT_TYPE portNum,
      NATIVE_UINT_TYPE context
    )
  {
    if (this->m_firstSched) {
        this->recvPrmReady_out(portNum, this->m_maxRecvSize, true);
        this->m_firstSched = false;
    } else {
        if (this->m_prmSendBufferIdx > 0) {
            U32 nSent = this->sendPrms();

            FW_ASSERT(nSent <= this->m_prmSendBufferIdx, nSent, this->m_prmSendBufferIdx);

            if (nSent != this->m_prmSendBufferIdx) {
                memmove(&this->m_prmSendBuffer[0],
                        &this->m_prmSendBuffer[nSent],
                        (this->m_prmSendBufferIdx - nSent) * sizeof(this->m_prmSendBuffer[0]));
            }

            this->m_prmSendBufferIdx -= nSent;
        }
    }
  }

  //! Handler for command L2_PRM_PUSH_UPDATES
  /* Push parameter updates to the L1 Prm Db */
  void PassiveL2PrmDbComponentImpl::
    L2_PRM_PUSH_UPDATES_cmdHandler(
      FwOpcodeType opCode, /*!< The opcode*/
      U32 cmdSeq, /*!< The command sequence number*/
      UpdateMethod updateMethod 
    )
  {
      switch (updateMethod) {

          case PUSH_UPDATES:
              this->m_updateMethod = PASSIVE_L2_PUSH_UPDATES;
              break;

          case NO_UPDATES:
              this->m_updateMethod = PASSIVE_L2_NO_UPDATES;
              break;

          default:
              FW_ASSERT(false, updateMethod);
      }
  }

  void PassiveL2PrmDbComponentImpl ::
    queuePrm(
      FwPrmIdType id
      )
  {
      for (U32 i = 0; i < this->m_prmSendBufferIdx; i++) {
          if (this->m_prmSendBuffer[i] == id) {
              // Already in list. No-op
              return;
          }
      }

      if (this->m_prmSendBufferIdx < PASSIVE_L2_PRMDB_SEND_BUFFER_ENTRIES) {
          this->m_prmSendBuffer[this->m_prmSendBufferIdx] = id;
          this->m_prmSendBufferIdx++; 
      } else {
          this->log_WARNING_HI_PrmSendBufferFull();
      }
  }

  NATIVE_INT_TYPE PassiveL2PrmDbComponentImpl ::
    sendPrms(void)
  {
    PrmDbImpl::SerializePrmStatus serStat;
    NATIVE_INT_TYPE serSize;

    U32 nSent = 0;

    if (this->m_prmSendReadySize > 0) {

        this->m_sendListBuffer.resetSer();

        while (nSent < this->m_prmSendBufferIdx) {
            FwPrmIdType prmId;

            prmId = this->m_prmSendBuffer[nSent];

            // TODO: serializePrmSize is a bad name since it doens't actually serialize anything...
            serStat = this->m_prmDb.serializePrmSize(prmId, serSize);
            FW_ASSERT(serStat == PrmDbImpl::SERIALIZE_PRM_OK);

            if (this->m_sendListBuffer.getBuffLength() + serSize > this->m_prmSendReadySize) {

                if (nSent == 0) {
                    // Don't break if we can't send the first parameter.
                    // Pretend like we sent it but EVR that the parameter was
                    // too large to sent
                    log_WARNING_HI_PrmSendTooLarge(prmId, serSize);
                } else {
                    break;
                }

            } else {
                serStat = this->m_prmDb.serializePrm(prmId, this->m_sendListBuffer);

                FW_ASSERT(serStat == PrmDbImpl::SERIALIZE_PRM_OK);
            }

            nSent++;
        }

        if (this->m_sendListBuffer.getBuffLength() > 0) {
            bool morePrms = nSent != this->m_prmSendBufferIdx;

            this->sendPrm_out(0, morePrms, this->m_sendListBuffer);
            this->m_prmSendReadySize = 0;
        }
    }

    return nSent;
  }

} // end namespace Svc
