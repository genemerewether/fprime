/*
 * TlmStoreImplTester.hpp
 *
 *  Created on: Mar 18, 2015
 *      Author: tcanham
 */

#ifndef TLMCHAN_TEST_UT_TLMCHANIMPLTESTER_HPP_
#define TLMCHAN_TEST_UT_TLMCHANIMPLTESTER_HPP_

#include <LLProc/LLTlmChan/test/ut/GTestBase.hpp>
#include <LLProc/LLTlmChan/LLTlmChanImpl.hpp>

namespace LLProc {

    class TlmChanImplTester: public LLTlmChanGTestBase {
        public:
            TlmChanImplTester(LLProc::LLTlmChanImpl& inst);
            virtual ~TlmChanImplTester();

            void init(NATIVE_INT_TYPE instance = 0);

            void runNominalChannel(void);
            void runMultiChannel(void);
            void runOffNominal(void);
            void runTooManyChannels(void);

        private:
            LLProc::LLTlmChanImpl& m_impl;

            void from_PktSend_handler(NATIVE_INT_TYPE portNum, Fw::ComBuffer &data, U32 context);

            // helper
            void sendBuff(FwChanIdType id, U32 val, NATIVE_INT_TYPE instance);
            bool doRun(bool check);
            void checkBuff(FwChanIdType id, U32 val, NATIVE_INT_TYPE instance);

            // Keep a history
            NATIVE_UINT_TYPE m_numBuffs;
            Fw::ComBuffer m_rcvdBuffer[TLMCHAN_HASH_BUCKETS];
            bool m_bufferRecv;
            void clearBuffs(void);

            // dump functions
            void dumpHash(void);
            static void dumpTlmEntry(LLTlmChanImpl::TlmEntry* entry);


    };

} /* namespace Svc */

#endif /* TLMCHAN_TEST_UT_TLMCHANIMPLTESTER_HPP_ */
