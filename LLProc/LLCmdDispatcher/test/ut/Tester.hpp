/*
 * LLCmdDispatcherImplTester.hpp
 *
 *  Created on: Mar 18, 2015
 *      Author: tcanham
 */

#ifndef CMDDISP_TEST_UT_TLMCHANIMPLTESTER_HPP_
#define CMDDISP_TEST_UT_TLMCHANIMPLTESTER_HPP_

#include <LLProc/LLCmdDispatcher/test/ut/GTestBase.hpp>
#include <LLProc/LLCmdDispatcher/LLCmdDispatcherComponentImpl.hpp>

namespace LLProc {

    class LLCmdDispatcherImplTester: public LLCmdDispatcherGTestBase {
        public:
            LLCmdDispatcherImplTester(LLProc::LLCmdDispatcherImpl& inst);
            virtual ~LLCmdDispatcherImplTester();

            void init(NATIVE_INT_TYPE instance = 0);

            void runNominalDispatch(void);
            void runInvalidOpcodeDispatch(void);
            void runFailedCommand(void);
            void runInvalidCommand(void);
            void runOverflowCommands(void);
            void runNopCommands(void);
            void runClearCommandTracking();

        private:
            LLProc::LLCmdDispatcherImpl& m_impl;

            void from_compCmdSend_handler(NATIVE_INT_TYPE portNum, FwOpcodeType opCode, U32 cmdSeq, Fw::CmdArgBuffer &args);

            bool m_cmdSendRcvd;
            FwOpcodeType m_cmdSendOpCode;
            U32 m_cmdSendCmdSeq;
            Fw::CmdArgBuffer m_cmdSendArgs;

            void from_seqCmdStatus_handler(NATIVE_INT_TYPE portNum, FwOpcodeType opCode, U32 cmdSeq, Fw::CommandResponse response);

            bool m_seqStatusRcvd;
            FwOpcodeType m_seqStatusOpCode;
            U32 m_seqStatusCmdSeq;
            Fw::CommandResponse m_seqStatusCmdResponse;

    };

} /* namespace LLProc */

#endif /* CMDDISP_TEST_UT_TLMCHANIMPLTESTER_HPP_ */
