/*
 * ActiveFileLoggerImpl.hpp
 *
 *  Created on: Nov 2, 2015
 *      Author: dpadams
 */

#ifndef ACTIVEFILELOGGERIMPL_HPP_
#define ACTIVEFILELOGGERIMPL_HPP_

#include <Svc/ActiveFileLogger/ActiveFileLoggerPacket.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerComponentAc.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerStreams.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerImplCfg.hpp>
#include <Os/LocklessQueue.hpp>
#include <Fw/Log/LogPacket.hpp>
#include <Fw/Types/Assert.hpp>

#include <cstdio>

namespace Svc {

    class ActiveFileLoggerImpl: public ActiveFileLoggerComponentBase {

        public:
#if FW_OBJECT_NAMES == 1
            ActiveFileLoggerImpl(const char* compName);
#else
            ActiveFileLoggerImpl();
#endif
            virtual ~ActiveFileLoggerImpl();
            void init(NATIVE_INT_TYPE queueDepth);
            void initLog(const char* baseDir); // open log files in specified base directory

        protected:
        private:
            void LogQueue_handler(NATIVE_INT_TYPE portNum,
                    Svc::ActiveFileLoggerPacket &data);
            void SchedIn_handler(NATIVE_INT_TYPE portNum, //!< The port number
                    NATIVE_UINT_TYPE context //!< The call order
                    );

            void FLOG_RESET_LOG_cmdHandler(FwOpcodeType opCode, U32 cmdSeq);

            //! Implementation for FLOG_ACTIVATE command handler
            //! Activate logging
            void FLOG_ACTIVATE_cmdHandler(
                const FwOpcodeType opCode, /*!< The opcode*/
                const U32 cmdSeq, /*!< The command sequence number*/
                bool activate /*!< Activate/deactivate the logger*/
            );

            //! Handler for command FLOG_RESET_LOG_DIR
            /* Restart the file logs with a new time/date stamp in the specified base directory */
            void FLOG_RESET_LOG_DIR_cmdHandler(
                FwOpcodeType opCode, /*!< The opcode*/
                U32 cmdSeq, /*!< The command sequence number*/
                const Fw::CmdStringArg& dir_name /*!< Base directory to start putting time stamped directories in for file logs.*/
            );

            FILE * m_fd[NUM_AFL_STREAMS];

            Svc::ActiveFileLoggerPacket * m_record;
            NATIVE_INT_TYPE * m_stream;
            char m_dirname[80];

            bool m_notify_drop;
            NATIVE_INT_TYPE m_record_size;
            NATIVE_INT_TYPE m_wr_pos;
            NATIVE_INT_TYPE m_rd_pos;

            Os::LocklessQueue * m_queue;

            bool m_active; //!< Logging is occurring

            const char* enumToString(active_file_logger_stream_t stream);
            void reset_log(FwOpcodeType opCode, U32 cmdSeq, const char* baseDir);

            char m_baseDir[ACTIVE_FILE_LOGGER_MAX_BASE_DIR];

    };

}
#endif /* ACTIVEFILELOGGERIMPL_HPP_ */
