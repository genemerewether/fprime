/*
 * PrmDbImplTester.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: tcanham
 */

#include <Svc/PrmDb/test/ut/PrmDbImplTester.hpp>
#include <Fw/Com/ComBuffer.hpp>
#include <Fw/Com/ComPacket.hpp>
#include <Os/Stubs/FileStubs.hpp>

#include <cstdio>
#include <cstring>
#include <gtest/gtest.h>

#include <Fw/Test/UnitTest.hpp>

namespace Svc {

    struct prmPair {

        prmPair(FwPrmIdType id, NATIVE_INT_TYPE prmInt) {
            this->id = id;
            this->prm.serialize(prmInt);
        }

        prmPair(FwPrmIdType id, const char* prmStr) {
            this->id = id;
            this->prm.serialize(reinterpret_cast<const U8*>(prmStr), strlen(prmStr));
        }

        FwPrmIdType id;
        Fw::ParamBuffer prm;
    }; 

    void PrmDbImplTester::runNominalPopulate(void) {

        PrmDbImpl::SetPrmStatus setStat;
        Fw::ParamValid valid;
        Fw::ParamBuffer prmBuff;

        struct prmPair prms[] = {
            {1, 10},
            {2, 50},
            {3, "MyString"},
            {5, "Skipped 4"},
            {6, -1000}
        };

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            setStat = this->m_impl.setPrm(prms[idx].id, prms[idx].prm);

            ASSERT_EQ(PrmDbImpl::SET_PRM_ADDED, setStat);
        }

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            valid = this->m_impl.getPrm(prms[idx].id, prmBuff);

            ASSERT_EQ(Fw::PARAM_VALID, valid);

            ASSERT_EQ(prms[idx].prm.getBuffLength(), prmBuff.getBuffLength());

            ASSERT_EQ(0, memcmp(prms[idx].prm.getBuffAddr(),
                                prmBuff.getBuffAddr(),
                                prmBuff.getBuffLength()));
        }
    }

    void PrmDbImplTester::runNominalUpdate(void) {

        PrmDbImpl::SetPrmStatus setStat;
        Fw::ParamValid valid;
        Fw::ParamBuffer prmBuff;

        struct prmPair prms[] = {
            {1, 10},
            {2, 50},
            {3, "MyString"},
            {5, "Skipped 4"},
            {6, -1000}
        };

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            setStat = this->m_impl.setPrm(prms[idx].id, prms[idx].prm);

            ASSERT_EQ(PrmDbImpl::SET_PRM_ADDED, setStat);
        }

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            valid = this->m_impl.getPrm(prms[idx].id, prmBuff);

            ASSERT_EQ(Fw::PARAM_VALID, valid);

            ASSERT_EQ(prms[idx].prm.getBuffLength(), prmBuff.getBuffLength());

            ASSERT_EQ(0, memcmp(prms[idx].prm.getBuffAddr(),
                                prmBuff.getBuffAddr(),
                                prmBuff.getBuffLength()));
        }

        // Update one of the integer parameters
        Fw::ParamBuffer intUpdateBuff;

        intUpdateBuff.serialize(static_cast<U32>(17));

        setStat = this->m_impl.setPrm(2, intUpdateBuff);

        ASSERT_EQ(PrmDbImpl::SET_PRM_UPDATED, setStat);

        // Read back updated parameter
        valid = this->m_impl.getPrm(2, prmBuff);

        ASSERT_EQ(Fw::PARAM_VALID, valid);

        U32 prmInt;

        prmBuff.deserialize(prmInt);

        ASSERT_EQ(17, prmInt);

        // Update one of the string parameters
        Fw::ParamBuffer strUpdateBuff;

        strUpdateBuff.serialize("Updated String");

        setStat = this->m_impl.setPrm(3, strUpdateBuff);

        ASSERT_EQ(PrmDbImpl::SET_PRM_UPDATED, setStat);

        // Read back updated parameter
        valid = this->m_impl.getPrm(3, prmBuff);

        ASSERT_EQ(Fw::PARAM_VALID, valid);

        ASSERT_EQ(strUpdateBuff.getBuffLength(), prmBuff.getBuffLength());

        ASSERT_EQ(0, memcmp(strUpdateBuff.getBuffAddr(),
                            prmBuff.getBuffAddr(),
                            prmBuff.getBuffLength()));

        ASSERT_NE(0, memcmp(prms[3].prm.getBuffAddr(),
                            prmBuff.getBuffAddr(),
                            std::min(prms[3].prm.getBuffLength(),
                                     prmBuff.getBuffLength())));


    }

    void PrmDbImplTester::runNominalSerDes(void) {

        PrmDbImpl::SetPrmStatus setStat;
        Fw::ParamBuffer prmBuff;
        PrmDbImpl::SerializePrmStatus serStat;
        PrmDbImpl::DeserializePrmStatus desStat;
        Fw::SerializeStatus stat;

        U8 serMem[256];

        U8 expectedSerInt[] = {0, 0, 0, 1, 0, 4, 0, 0, 0, 10};
        U8 expectedSerStr[] = {0, 0, 0, 3, 0, 10, 0, 8, 'M', 'y', 'S', 't', 'r', 'i', 'n', 'g'};

        struct prmPair prms[] = {
            {1, 10},
            {2, 50},
            {3, "MyString"},
            {5, "Skipped 4"},
            {6, -1000}
        };

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            setStat = this->m_impl.setPrm(prms[idx].id, prms[idx].prm);

            ASSERT_EQ(PrmDbImpl::SET_PRM_ADDED, setStat);
        }

        // Serialize int parameter
        Fw::ExternalSerializeBuffer serdesBuff(serMem, sizeof(serMem));

        serStat = this->m_impl.serializePrm(1, serdesBuff);

        ASSERT_EQ(PrmDbImpl::SERIALIZE_PRM_OK, serStat);

        ASSERT_EQ(0, memcmp(serMem,
                            expectedSerInt,
                            sizeof(expectedSerInt)));

        // Deserialize int parameter
        FwPrmIdType id;
        U32 intPrm;

        desStat = this->m_impl.deserializePrm(serdesBuff, id, prmBuff);

        ASSERT_EQ(PrmDbImpl::DESERIALIZE_PRM_OK, desStat);

        ASSERT_EQ(1, id);

        stat = prmBuff.deserialize(intPrm);
        ASSERT_EQ(Fw::FW_SERIALIZE_OK, stat);

        ASSERT_EQ(10, intPrm);

        // Serialize string parameter
        serdesBuff.resetSer();
        serdesBuff.resetDeser();

        serStat = this->m_impl.serializePrm(3, serdesBuff);

        ASSERT_EQ(PrmDbImpl::SERIALIZE_PRM_OK, serStat);

        ASSERT_EQ(0, memcmp(serMem,
                            expectedSerStr,
                            sizeof(expectedSerStr)));


        // Deserialize string parameter
        U8 strPrm[256];
        NATIVE_UINT_TYPE strLen;

        memset(strPrm, 0, sizeof(strPrm));
        strLen = sizeof(strPrm);

        desStat = this->m_impl.deserializePrm(serdesBuff, id, prmBuff);

        ASSERT_EQ(PrmDbImpl::DESERIALIZE_PRM_OK, desStat);

        ASSERT_EQ(3, id);

        stat = prmBuff.deserialize(strPrm, strLen);
        ASSERT_EQ(Fw::FW_SERIALIZE_OK, stat);

        ASSERT_STREQ("MyString", reinterpret_cast<char*>(strPrm));
    }

    void PrmDbImplTester::runNominalClear(void) {

        PrmDbImpl::SetPrmStatus setStat;
        Fw::ParamValid valid;
        Fw::ParamBuffer prmBuff;

        struct prmPair prms[] = {
            {1, 10},
            {2, 50},
            {3, "MyString"},
            {5, "Skipped 4"},
            {6, -1000}
        };

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            setStat = this->m_impl.setPrm(prms[idx].id, prms[idx].prm);

            ASSERT_EQ(PrmDbImpl::SET_PRM_ADDED, setStat);
        }

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            valid = this->m_impl.getPrm(prms[idx].id, prmBuff);

            ASSERT_EQ(Fw::PARAM_VALID, valid);

            ASSERT_EQ(prms[idx].prm.getBuffLength(), prmBuff.getBuffLength());

            ASSERT_EQ(0, memcmp(prms[idx].prm.getBuffAddr(),
                                prmBuff.getBuffAddr(),
                                prmBuff.getBuffLength()));
        }

        this->m_impl.clearDb();

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            valid = this->m_impl.getPrm(prms[idx].id, prmBuff);

            ASSERT_EQ(Fw::PARAM_INVALID, valid);
        }
    }

    void PrmDbImplTester::runNominalIterator(void) {

        PrmDbImpl::SetPrmStatus setStat;
        Fw::ParamBuffer prmBuff;

        struct prmPair prms[] = {
            {1, 10},
            {2, 50},
            {3, "MyString"},
            {5, "Skipped 4"},
            {6, -1000}
        };

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            setStat = this->m_impl.setPrm(prms[idx].id, prms[idx].prm);

            ASSERT_EQ(PrmDbImpl::SET_PRM_ADDED, setStat);
        }

        const FwPrmIdType expIds[] = {1,2,3,5,6};
        NATIVE_INT_TYPE expIdsIdx = 0;

        for (const FwPrmIdType id : this->m_impl) {
            ASSERT_LT(expIdsIdx, FW_NUM_ARRAY_ELEMENTS(expIds));
            ASSERT_EQ(expIds[expIdsIdx], id);
            expIdsIdx++;
        }

        ASSERT_EQ(expIdsIdx, FW_NUM_ARRAY_ELEMENTS(expIds));

        this->m_impl.clearDb();

        for (const FwPrmIdType id : this->m_impl) {
            (void)id;
            // Should not be called
            ASSERT_TRUE(false);
        }
    }


    void PrmDbImplTester::runFullPopulate(void) {

        PrmDbImpl::SetPrmStatus setStat;
        Fw::ParamBuffer prmBuff;

        struct prmPair prms[] = {
            {1, 10},
            {2, 50},
            {3, "MyString"},
            {5, "Skipped 4"},
            {6, -1000}
        };

        for (int idx = 0; idx < PRMDB_NUM_DB_ENTRIES; idx++) {

            prmBuff.resetSer();
            prmBuff.serialize(idx*4);

            setStat = this->m_impl.setPrm(idx + 3, prmBuff);

            ASSERT_EQ(PrmDbImpl::SET_PRM_ADDED, setStat);
        }

        prmBuff.resetSer();
        prmBuff.serialize(-1);

        setStat = this->m_impl.setPrm(1, prmBuff);

        ASSERT_EQ(PrmDbImpl::SET_PRM_FULL, setStat);
    }

    void PrmDbImplTester::runInvalidSerialize(void) {

        PrmDbImpl::SetPrmStatus setStat;
        Fw::ParamBuffer prmBuff;
        PrmDbImpl::SerializePrmStatus serStat;

        U8 serMem[256];

        struct prmPair prms[] = {
            {1, 10},
            {2, 50},
            {3, "MyString"},
            {5, "Skipped 4"},
            {6, -1000}
        };

        for (int idx = 0; idx < FW_NUM_ARRAY_ELEMENTS(prms); idx++) {

            setStat = this->m_impl.setPrm(prms[idx].id, prms[idx].prm);

            ASSERT_EQ(PrmDbImpl::SET_PRM_ADDED, setStat);
        }

        // Serialize int parameter
        Fw::ExternalSerializeBuffer serdesBuff(serMem, sizeof(serMem));

        serStat = this->m_impl.serializePrm(10, serdesBuff);

        ASSERT_EQ(PrmDbImpl::SERIALIZE_PRM_NOT_VALID, serStat);
    }


    PrmDbImplTester::PrmDbImplTester(Svc::PrmDbImpl& inst) :
            m_impl(inst) {
    }

    PrmDbImplTester::~PrmDbImplTester() {
    }
    void PrmDbImplTester ::
      from_pingOut_handler(
          const NATIVE_INT_TYPE portNum,
          U32 key
      )
    {
      this->pushFromPortEntry_pingOut(key);
    }


} /* namespace SvcTest */
