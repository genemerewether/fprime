/*
 * ActiveFileLoggerImpl.cpp
 *
 *  Created on: Oct 06, 2015
 *      Author: dpadams
 */

#include <Svc/ActiveFileLogger/ActiveFileLoggerImpl.hpp>
#include <Os/LocklessQueue.hpp>
#include <Fw/Types/Assert.hpp>
#include <Os/FileSystem.hpp>

#include <time.h>
#include <sys/stat.h>

#ifdef __XENO__
#define FPRINTF __real_fprintf
#else
#define FPRINTF fprintf
#endif

#ifdef __XENO__
#define FWRITE __real_fwrite
#else
#define FWRITE fwrite
#endif

namespace Svc {

#if FW_OBJECT_NAMES == 1
    ActiveFileLoggerImpl::ActiveFileLoggerImpl(const char* name) :
            ActiveFileLoggerComponentBase(name)
#else
                    ActiveFileLoggerImpl::ActiveFileLoggerImpl() :
                    ActiveFileLoggerComponentBase()
#endif
                    , m_record(0), m_stream(0), m_notify_drop(true), m_record_size(
                    20000), m_wr_pos(0), m_rd_pos(0), m_active(false) {
        m_queue = new Os::LocklessQueue(m_record_size,
                ActiveFileLoggerPacket::ACTIVE_FILE_LOGGER_BUFFER_SIZE);

        for (int i = 0; i < NUM_AFL_STREAMS; i++) {
            m_fd[i] = NULL;
        }

    }

    ActiveFileLoggerImpl::~ActiveFileLoggerImpl() {
        for (int i = 0; i < NUM_AFL_STREAMS; i++) {
            if (m_fd[i]) {
                fclose(m_fd[i]);
                m_fd[i] = NULL;
            }
        }

        delete m_queue;
    }

    void ActiveFileLoggerImpl::init(NATIVE_INT_TYPE queueDepth) {
        ActiveFileLoggerComponentBase::init(queueDepth);
    }

    void ActiveFileLoggerImpl::initLog(const char* baseDir) {

        FW_ASSERT(baseDir);
        time_t t = time(NULL);
        struct tm tm = *localtime(&t);

        // store base directory
        strncpy(this->m_baseDir, baseDir,
                strnlen(baseDir, ACTIVE_FILE_LOGGER_MAX_BASE_DIR));
        // null terminate
        this->m_baseDir[ACTIVE_FILE_LOGGER_MAX_BASE_DIR - 1] = 0;

        snprintf(this->m_dirname, 80, "%s/%04d-%02d-%02dT%02d-%02d-%02d",
                this->m_baseDir, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                tm.tm_hour, tm.tm_min, tm.tm_sec);
        mkdir(this->m_dirname, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    }

    void ActiveFileLoggerImpl::LogQueue_handler(NATIVE_INT_TYPE portNum,
            Svc::ActiveFileLoggerPacket & data) {

        // only log if active
        if (not this->m_active) {
            return;
        }

        Os::Queue::QueueStatus stat;
        stat = m_queue->Send(data.getBuffAddr(), data.getBuffLength());
        FW_ASSERT(stat == Os::Queue::QUEUE_OK, stat);

    }

    void ActiveFileLoggerImpl::SchedIn_handler(NATIVE_INT_TYPE portNum, //!< The port number
            NATIVE_UINT_TYPE context //!< The call order
            ) {

        Os::Queue::QueueStatus stat;
        const NATIVE_INT_TYPE bufsize =
                ActiveFileLoggerPacket::ACTIVE_FILE_LOGGER_BUFFER_SIZE;
        U8 buf[bufsize];

        NATIVE_INT_TYPE size;
        NATIVE_INT_TYPE pnum;

        bool exit_loop = false;
        while (!exit_loop) {
            stat = m_queue->Receive(buf, bufsize, size);

            switch (stat) {
                case Os::Queue::QUEUE_OK: {
                    pnum = buf[0];
                    FW_ASSERT(pnum < NUM_AFL_STREAMS, pnum);

                    if (m_fd[pnum] == NULL) {
                        char filename[256];
                        memset(filename, 0, sizeof(filename));
                        snprintf(filename, 256, "%s/logstream_%s.bin",
                                this->m_dirname,
                                enumToString(
                                        (active_file_logger_stream_t) pnum));
                        m_fd[pnum] = fopen(filename, "wb");
                    }

                    if (m_fd[pnum] != NULL) {
                        // Print the entry to a file
                        FWRITE(buf + 1, sizeof(U8), size - 1, m_fd[pnum]);
                    }
                    continue;
                }
                case Os::Queue::QUEUE_NO_MORE_MSGS: {
                    exit_loop = true;
                    break;
                }
                default: {
                    FW_ASSERT(0, stat);
                    return;
                }
            }
        }

        // Only flush the file at the end of the processing to save on I/O
        for (int i = 0; i < NUM_AFL_STREAMS; i++) {
            if (m_fd[i] != NULL) {
                fflush(m_fd[i]);
            }
        }
    }

    const char* ActiveFileLoggerImpl::enumToString(
            active_file_logger_stream_t stream) {
        switch (stream) {
            case AFL_SERIAL_DATA:
                return "Ser";
            case AFL_HLROSIFACE_IMUNOCOV:
                return "ImuNoCov";
            case AFL_FILTIFACE_ODOMNOCOV:
                return "OdomNoCov";
            case AFL_MRCTRLIFACE_ACCEL_CMD:
                return "AccelCmd";
            case AFL_ACTADAP_ESC:
                return "ActAdapEsc";
            case AFL_MVCAM_CALLBACK:
                return "MVCamCallback";
            case AFL_SCAM_CALLBACK:
                return "StereoCamCallback";
  	    case AFL_ATINETBOX_WRENCH:
	        return "ATINetboxWrench";
  	    case AFL_ATINETBOX_WRENCH_AA:
	        return "ATINetboxWrenchAA";
            default:
                return "Unknown";
        }
    }

    void ActiveFileLoggerImpl::reset_log(FwOpcodeType opCode, U32 cmdSeq,
            const char* baseDir) {

        // if basedDir does not exist, then create it:
        U64 size;
        if (Os::FileSystem::getFileSize(baseDir, size)
                != Os::FileSystem::OP_OK) {
            (void) Os::FileSystem::createDirectory(baseDir);
        }

        // pick new directory name
        this->initLog(baseDir);
        // close any open logs
        for (U32 stream = AFL_SERIAL_DATA; stream < NUM_AFL_STREAMS; stream++) {
            if (this->m_fd[stream] != NULL) {
                ::fclose(this->m_fd[stream]);
                this->m_fd[stream] = NULL;
            }
        }

        Fw::LogStringArg dir(this->m_dirname);
        this->log_ACTIVITY_HI_AFL_ResetLogCmd(dir);

        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void ActiveFileLoggerImpl::FLOG_RESET_LOG_cmdHandler(FwOpcodeType opCode,
            U32 cmdSeq) {

        this->reset_log(opCode, cmdSeq, this->m_baseDir);
    }

    void ActiveFileLoggerImpl::FLOG_ACTIVATE_cmdHandler(
            const FwOpcodeType opCode, const U32 cmdSeq, bool activate) {

        this->m_active = activate;
        this->log_ACTIVITY_HI_AFL_ActivateCmd(activate);
        this->cmdResponse_out(opCode, cmdSeq, Fw::COMMAND_OK);
    }

    void ActiveFileLoggerImpl::FLOG_RESET_LOG_DIR_cmdHandler(
    FwOpcodeType opCode, /*!< The opcode*/
    U32 cmdSeq, /*!< The command sequence number*/
    const Fw::CmdStringArg& dir_name /*!< Base directory to start putting time stamped directories in for file logs.*/
    ) {
        this->reset_log(opCode, cmdSeq, dir_name.toChar());
    }

} // namespace Svc
