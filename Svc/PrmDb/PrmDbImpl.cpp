/*
 * PrmDbImpl.cpp
 *
 *  Created on: March 9, 2015
 *      Author: Timothy Canham
 */

#include <Svc/PrmDb/PrmDbImpl.hpp>
#include <Fw/Types/Assert.hpp>
#include <Fw/Types/EightyCharString.hpp>

#include <Os/File.hpp>

#include <cstring>
#include <stdio.h>

namespace Svc {


    PrmDbImpl::PrmDbImpl() {
        this->clearDb();
    }

    PrmDbImpl::~PrmDbImpl() {

    }

    void PrmDbImpl::clearDb(void) {
        for (I32 entry = 0; entry < PRMDB_NUM_DB_ENTRIES; entry++) {
            this->m_db[entry].used = false;
            this->m_db[entry].id = 0;
        }
    }

    Fw::ParamValid PrmDbImpl::getPrm(const FwPrmIdType id, Fw::ParamBuffer &val) {
        // search for entry
        Fw::ParamValid stat = Fw::PARAM_INVALID;

        for (I32 entry = 0; entry < PRMDB_NUM_DB_ENTRIES; entry++) {
            if (this->m_db[entry].used) {
                if (this->m_db[entry].id == id) {
                    val = this->m_db[entry].val;
                    stat = Fw::PARAM_VALID;
                    break;
                }
            }
        }

        return stat;
    }

    PrmDbImpl::SetPrmStatus PrmDbImpl::setPrm(const FwPrmIdType id, Fw::ParamBuffer &val) {

        // search for existing entry

        bool existingEntry = false;
        bool noSlots = true;

        for (NATIVE_INT_TYPE entry = 0; entry < PRMDB_NUM_DB_ENTRIES; entry++) {
            if ((this->m_db[entry].used) && (id == this->m_db[entry].id)) {
                this->m_db[entry].val = val;
                existingEntry = true;
                break;
            }
        }

        // if there is no existing entry, add one
        if (!existingEntry) {
            for (I32 entry = 0; entry < PRMDB_NUM_DB_ENTRIES; entry++) {
                if (!(this->m_db[entry].used)) {
                    this->m_db[entry].val = val;
                    this->m_db[entry].id = id;
                    this->m_db[entry].used = true;
                    noSlots = false;
                    break;
                }
            }
        }

        if (existingEntry) {
            return SET_PRM_UPDATED;
        } else if (noSlots) {
            return SET_PRM_FULL;
        } else {
            return SET_PRM_ADDED;
        }
    }

    PrmDbImpl::SerializePrmStatus PrmDbImpl::serializePrmSize(const FwPrmIdType id, NATIVE_INT_TYPE& size) {
        Fw::ParamValid valid;

        Fw::ParamBuffer prmBuffer;

        valid = this->getPrm(id, prmBuffer);

        if (valid == Fw::PARAM_INVALID) {
            return SERIALIZE_PRM_NOT_VALID;
        }

        size = sizeof(FwPrmIdType) + sizeof(FwBuffSizeType) + prmBuffer.getBuffLength();

        return SERIALIZE_PRM_OK;
    }

    PrmDbImpl::SerializePrmStatus PrmDbImpl::serializePrm(const FwPrmIdType id, Fw::SerializeBufferBase& buff) {
        Fw::SerializeStatus stat;
        Fw::ParamValid valid;

        Fw::ParamBuffer prmBuffer;

        valid = this->getPrm(id, prmBuffer);

        if (valid == Fw::PARAM_INVALID) {
            return SERIALIZE_PRM_NOT_VALID;
        }

        // Serialize Id
        stat = buff.serialize(id);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, (NATIVE_INT_TYPE)stat);

        // Serialize Parameter
        stat = buff.serialize(prmBuffer);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, (NATIVE_INT_TYPE)stat);

        return SERIALIZE_PRM_OK;
    }

    PrmDbImpl::DeserializePrmStatus PrmDbImpl::deserializePrm(Fw::SerializeBufferBase& buff, FwPrmIdType& id, Fw::ParamBuffer& prm) {
        Fw::SerializeStatus stat;

        // Deserialize Id
        stat = buff.deserialize(id);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, (NATIVE_INT_TYPE)stat);

        // Deserialize Parameter
        stat = buff.deserialize(prm);
        FW_ASSERT(Fw::FW_SERIALIZE_OK == stat, (NATIVE_INT_TYPE)stat);

        return DESERIALIZE_PRM_OK;
    }

    PrmDbImpl::PrmDbIterator::PrmDbIterator(const PrmDbImpl& prmDb, const NATIVE_INT_TYPE startEntry) :
        db(prmDb.m_db)
    {
        NATIVE_INT_TYPE idx;

        // Locate a valid entry. If no valid entries are found, use PRMDB_NUM_DB_ENTRIES

        if (startEntry < PRMDB_NUM_DB_ENTRIES) {
            // Look for the first entry
            for (idx = startEntry; idx < PRMDB_NUM_DB_ENTRIES; idx++) {
                if (this->db[idx].used) {
                    break;
                }
            }

            this->entry = idx;
        } else {
            this->entry = PRMDB_NUM_DB_ENTRIES;
        }
    }

    void PrmDbImpl::PrmDbIterator::advanceToNextEntry() {
        NATIVE_INT_TYPE idx;

        for (idx = (this->entry + 1); idx < PRMDB_NUM_DB_ENTRIES; idx++) {
            if (this->db[idx].used) {
                break;
            }
        }

        this->entry = idx;
    }
}
