// ====================================================================== 
// \title  ActiveL1PrmDbImpl.cpp
// \author kubiak
// \brief  cpp file for ActiveL1PrmDb component implementation class
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


#include <Svc/ActiveL1PrmDb/ActiveL1PrmDbImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Os/File.hpp>

#include <Svc/PrmDb/PrmDbImpl.hpp>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#define DEBUG_PRINT(x,...)

namespace Svc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  ActiveL1PrmDbComponentImpl ::
#if FW_OBJECT_NAMES == 1
    ActiveL1PrmDbComponentImpl(
        const char *const compName,
        const char *file,
        const NATIVE_INT_TYPE maxRecvSize
    ) :
      ActiveL1PrmDbComponentBase(compName),
#else
    ActiveL1PrmDbImpl(
        const char *file,
        const NATIVE_INT_TYPE maxRecvSize
    ) :
#endif
      m_maxRecvSize(maxRecvSize)
  {
    this->m_fileName = file;
  }

  void ActiveL1PrmDbComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    ) 
  {
    ActiveL1PrmDbComponentBase::init(queueDepth, instance);

    NATIVE_INT_TYPE idx;

    for (idx = 0; idx < NUM_SENDPRM_OUTPUT_PORTS; idx++) {
        this->m_level2PrmDbs[idx].iter = this->m_prmDb.end();
    }
  }

  ActiveL1PrmDbComponentImpl ::
    ~ActiveL1PrmDbComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void ActiveL1PrmDbComponentImpl ::
    sendPrmReady_handler(
        const NATIVE_INT_TYPE portNum,
        U32 maxSize,
        bool reload
    )
  {
    PrmDbImpl::SerializePrmStatus serStat;
    NATIVE_INT_TYPE serSize;

    // Confirm that we can respond to this command
    FW_ASSERT(this->isConnected_sendPrm_OutputPort(portNum), portNum);
    FW_ASSERT(portNum < NUM_SENDPRM_OUTPUT_PORTS, portNum);

    DEBUG_PRINT("L1PrmDb sendprmready handler\n");

    if (reload) {
        this->m_level2PrmDbs[portNum].iter = this->m_prmDb.begin();
    }

    bool morePrms = false;
    if (this->m_level2PrmDbs[portNum].iter.valid() &&
        this->m_level2PrmDbs[portNum].iter != this->m_prmDb.end()) {

        this->lock();

        PrmDbImpl::PrmDbIterator& iter = this->m_level2PrmDbs[portNum].iter;
        Fw::ParamList& buff = this->m_level2PrmDbs[portNum].m_sendBuffer;

        buff.resetSer();

        while (iter != this->m_prmDb.end()) {

            FwPrmIdType prmId = *iter;

            if (this->prmIdInRange(portNum, prmId)) {
                serStat = this->m_prmDb.serializePrmSize(prmId, serSize);
                FW_ASSERT(serStat == PrmDbImpl::SERIALIZE_PRM_OK);

                if (((serSize + buff.getBuffLength()) > maxSize) ||
                    ((serSize + buff.getBuffLength()) > buff.getBuffCapacity())) {

                    if (buff.getBuffLength() == 0) {
                        // Not enough size to send parameter. EVR
                        // Increase iterator to hobble along

                        this->log_WARNING_HI_PrmSendTooLarge(prmId,
                                                             serSize,
                                                             portNum);

                        iter++;
                        continue;
                    } else {
                        break;
                    }
                }

                serStat = this->m_prmDb.serializePrm(prmId, buff);
                FW_ASSERT(serStat == PrmDbImpl::SERIALIZE_PRM_OK);
            }

            iter++;
        }

        morePrms = iter != this->m_prmDb.end();
        
        this->unLock();

        this->sendPrm_out(portNum, morePrms, buff);
    }
            
    if (!morePrms) {
        DEBUG_PRINT("L1PrmDb ready to recv prms\n");
        // Signal that we are ready to receive parameter updates
        this->recvPrmReady_out(portNum, this->m_maxRecvSize, false);
    }
  }

  void ActiveL1PrmDbComponentImpl ::
    setPrm_handler(
        const NATIVE_INT_TYPE portNum,
        FwPrmIdType id,
        Fw::ParamBuffer &val
    )
  {
    PrmDbImpl::SetPrmStatus stat;

    this->lock();

    stat = this->m_prmDb.setPrm(id, val);

    this->unLock();

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

  void ActiveL1PrmDbComponentImpl ::
    recvPrm_handler(
        const NATIVE_INT_TYPE portNum,
        bool morePrms,
        Fw::ParamList val
    )
  {
    Fw::SerializeStatus desStat;
    PrmDbImpl::DeserializePrmStatus desPrmStat;
    PrmDbImpl::SetPrmStatus setStat;

    FwPrmIdType prmId;
    Fw::ParamBuffer prmBuff;

    DEBUG_PRINT("L1PrmDb recvprm handler\n");
    
    this->lock();

    while (val.getBuffLeft() > 0) {

        desPrmStat = this->m_prmDb.deserializePrm(val, prmId, prmBuff);
        FW_ASSERT(PrmDbImpl::DESERIALIZE_PRM_OK == desPrmStat, desPrmStat);

        prmBuff.resetDeser();
        setStat = this->m_prmDb.setPrm(prmId, prmBuff);
        if (setStat == PrmDbImpl::SET_PRM_FULL) {
            this->log_WARNING_HI_PrmDbFull(prmId);
        }
    }
    
    this->recvPrmReady_out(portNum, this->m_maxRecvSize, false);

    this->unLock();
    
    DEBUG_PRINT("L1PrmDb recvprm handler done\n");
  }

  void ActiveL1PrmDbComponentImpl ::
    pingIn_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    this->pingOut_out(portNum, key);
  }

  Fw::ParamValid ActiveL1PrmDbComponentImpl ::
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

  // ----------------------------------------------------------------------
  // Command handler implementations 
  // ----------------------------------------------------------------------

  void ActiveL1PrmDbComponentImpl ::
    PRM_SAVE_FILE_cmdHandler(
        const FwOpcodeType opCode,
        const U32 cmdSeq
    )
  {
    Os::File paramFile;
    PrmDbWorkingBuffer buff;
    PrmDbWorkingBuffer recordSizeBuff;
    NATIVE_INT_TYPE writeSize;
    const U8 delim = ACTIVE_PRMDB_ENTRY_DELIMETER;
    Fw::SerializeStatus fwSerStat;
    U32 recordSize;

    Os::File::Status stat = paramFile.open(this->m_fileName.toChar(),Os::File::OPEN_WRITE);
    if (stat != Os::File::OP_OK) {
        this->log_WARNING_HI_PrmFileWriteError(PRM_WRITE_OPEN,0,stat);
        this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
        return;
    }

    this->lock();

    U32 numRecords = 0;

    PrmDbImpl::SerializePrmStatus serStat;

    PrmDbImpl::PrmDbIterator idIter = this->m_prmDb.begin();

    // Traverse the parameter list, saving each entry
    while (idIter != this->m_prmDb.end()) {

        FwPrmIdType id = *idIter;

        buff.resetSer();

        serStat = this->m_prmDb.serializePrm(id, buff);
        FW_ASSERT(serStat == PrmDbImpl::SERIALIZE_PRM_OK, serStat);

        // write delimeter
        writeSize = sizeof(delim);
        stat = paramFile.write(&delim,writeSize,true);
        if (stat != Os::File::OP_OK) {
            this->log_WARNING_HI_PrmFileWriteError(PRM_WRITE_DELIMETER,numRecords,stat);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            break;
        }
        if (writeSize != sizeof(delim)) {
            this->log_WARNING_HI_PrmFileWriteError(PRM_WRITE_DELIMETER_SIZE,numRecords,writeSize);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            break;
        }

        recordSizeBuff.resetSer();
        recordSize = buff.getBuffLength();

        fwSerStat = recordSizeBuff.serialize(recordSize);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == fwSerStat, fwSerStat);

        writeSize = static_cast<NATIVE_INT_TYPE>(recordSizeBuff.getBuffLength());
        stat = paramFile.write(recordSizeBuff.getBuffAddr(),
                               writeSize,
                               true);
        if (stat != Os::File::OP_OK) {
            this->log_WARNING_HI_PrmFileWriteError(PRM_WRITE_RECORDSIZE_SIZE,
                    numRecords,
                    stat);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            break;
        }
        if (writeSize != sizeof(recordSize)) {
            this->log_WARNING_HI_PrmFileWriteError(PRM_WRITE_RECORDSIZE_SIZE,
                    numRecords,
                    writeSize);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            break;
        }


        // Ensure the the cast to int is valid
        FW_ASSERT(buff.getBuffLength() < static_cast<NATIVE_UINT_TYPE>(0x80000000), buff.getBuffLength());
        writeSize = static_cast<NATIVE_INT_TYPE>(buff.getBuffLength());
        stat = paramFile.write(buff.getBuffAddr(),
                writeSize,
                true);
        if (stat != Os::File::OP_OK) {
            this->log_WARNING_HI_PrmFileWriteError(PRM_WRITE_PARAMETER,
                    numRecords,
                    stat);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            break;
        }
        if (writeSize != buff.getBuffLength()) {
            this->log_WARNING_HI_PrmFileWriteError(PRM_WRITE_PARAMETER_SIZE,
                    numRecords,
                    writeSize);
            this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_EXECUTION_ERROR);
            break;
        }

        numRecords++;
        idIter++;
    } // end for each record

    this->unLock();

    if (idIter == this->m_prmDb.end()) {
        paramFile.close();
        this->log_ACTIVITY_HI_PrmFileSaveComplete(numRecords);
        this->cmdResponse_out(opCode,cmdSeq,Fw::COMMAND_OK);
    }
  }

  void ActiveL1PrmDbComponentImpl::readPrmFile(void) {
    // Load parameter file. TODO: Put more robust file checking, such as a CRC.
    //TODO: The serialization format is very inefficient.
    //Up to three copies of the entry length are stored per string...
    Os::File paramFile;
    U8 delimeter;
    PrmDbWorkingBuffer buff;
    Fw::ParamBuffer prmValBuff;
    U32 recordNum = 0;
    U32 recordSize;
    bool success = false;
    FwPrmIdType parameterId;
    PrmDbImpl::SetPrmStatus setStat;

    Os::File::Status stat = paramFile.open(this->m_fileName.toChar(),Os::File::OPEN_READ);
    if (stat != Os::File::OP_OK) {
        this->log_WARNING_HI_PrmFileReadError(PRM_READ_OPEN,0,stat);
        return;
    }

    this->lock();

    this->m_prmDb.clearDb();

    for (NATIVE_INT_TYPE entry = 0; entry < PRMDB_NUM_DB_ENTRIES; entry++)  {

        // Read Delimiter
        NATIVE_INT_TYPE readSize = sizeof(delimeter);

        Os::File::Status fStat = paramFile.read(&delimeter,readSize,true);

        // check for end of file (read size 0)
        if (0 == readSize) {
            success = true;
            break;
        }

        if (fStat != Os::File::OP_OK) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_DELIMETER,recordNum,fStat);
            break;
        }

        if (sizeof(delimeter) != readSize) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_DELIMETER_SIZE,recordNum,readSize);
            break;
        }

        if (ACTIVE_PRMDB_ENTRY_DELIMETER != delimeter) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_DELIMETER_VALUE,recordNum,delimeter);
            break;
        }

        // Read Record Size
        readSize = sizeof(recordSize);

        fStat = paramFile.read(buff.getBuffAddr(),readSize,true);
        if (fStat != Os::File::OP_OK) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_RECORD_SIZE,recordNum,fStat);
            break;
        }
        if (sizeof(recordSize) != readSize) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_RECORD_SIZE_SIZE,recordNum,readSize);
            break;
        }

        Fw::SerializeStatus desStat = buff.setBuffLen(readSize);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == desStat,(NATIVE_INT_TYPE)desStat);

        buff.resetDeser();
        desStat = buff.deserialize(recordSize);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == desStat);

        // Sanity check value. It can't be larger than the maximum parameter buffer size + id
        // or smaller than the record id
        if ((recordSize > FW_PARAM_BUFFER_MAX_SIZE + sizeof(U32)) or (recordSize < sizeof(U32))) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_RECORD_SIZE_VALUE,recordNum,recordSize);
            break;
        }

        // Read Parameter
        readSize = recordSize;
        fStat = paramFile.read(buff.getBuffAddr(),readSize,true);
        if (fStat != Os::File::OP_OK) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_PARAMETER,recordNum,fStat);
            break;
        }

        if (static_cast<NATIVE_INT_TYPE>(recordSize) != readSize) {
            this->log_WARNING_HI_PrmFileReadError(PRM_READ_PARAMETER_SIZE,recordNum,readSize);
            break;
        }

        desStat = buff.setBuffLen(recordSize);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == desStat,(NATIVE_INT_TYPE)desStat);

        buff.resetDeser();

        PrmDbImpl::DeserializePrmStatus desPrmStat =
            this->m_prmDb.deserializePrm(buff,
                                         parameterId,
                                         prmValBuff);
        FW_ASSERT(desPrmStat == PrmDbImpl::DESERIALIZE_PRM_OK);

        setStat = this->m_prmDb.setPrm(parameterId, prmValBuff);

        FW_ASSERT(setStat == PrmDbImpl::SET_PRM_ADDED);

        recordNum++;
    }

    this->unLock();

    if (success) {
        this->log_ACTIVITY_HI_PrmFileLoadComplete(recordNum);
    }
  }

  void ActiveL1PrmDbComponentImpl::setPrmDbRanges(const PrmDbRange* ranges,
                                                  const NATIVE_INT_TYPE numRanges) {
    FW_ASSERT(numRanges < ACTIVE_PRMDB_NUM_RANGES, numRanges);
    FW_ASSERT(ranges != nullptr, reinterpret_cast<U64>(ranges));

    for (NATIVE_INT_TYPE idx = 0; idx < numRanges; idx++) {
        this->m_prmDbRanges[idx] = ranges[idx];
    }
  }

  void ActiveL1PrmDbComponentImpl::setupPrmTransfer() {

      NATIVE_INT_TYPE idx;

      for (idx = 0; idx < NUM_SENDPRM_OUTPUT_PORTS; idx++) {
          this->m_level2PrmDbs[idx].iter = this->m_prmDb.begin();
      }
  }

  bool ActiveL1PrmDbComponentImpl::prmIdInRange(const NATIVE_INT_TYPE port,
                                                const FwPrmIdType id) {

    NATIVE_INT_TYPE idx;

    for (idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(this->m_prmDbRanges); idx++) {

        if (this->m_prmDbRanges[idx].port == port) {
            if (this->m_prmDbRanges[idx].valid &&
                this->m_prmDbRanges[idx].lowId <= id &&
                this->m_prmDbRanges[idx].highId >= id) {

                return true;
            }
        }
    }

    return false;
  }

} // end namespace Svc
