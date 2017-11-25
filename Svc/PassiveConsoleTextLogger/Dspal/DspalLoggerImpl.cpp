#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>
#include <HAP_farf.h>

namespace Svc {

    void ConsoleTextLoggerImpl::init(void) {
        PassiveTextLoggerComponentBase::init();
    }
    
    void ConsoleTextLoggerImpl::TextLogger_handler(NATIVE_INT_TYPE portNum, FwEventIdType id, Fw::Time &timeTag, Fw::TextLogSeverity severity, Fw::TextLogString &text) {

      NATIVE_INT_TYPE log = 0;
        const char *severityString = "UNK";
        switch (severity) {
            case Fw::TEXT_LOG_FATAL:
                severityString = "FAT";
                log = 1;
                break;
            case Fw::TEXT_LOG_WARNING_HI:
                severityString = "WRNG_HI";
                log = 1;
                break;
            case Fw::TEXT_LOG_WARNING_LO:
                severityString = "WRNG_LO";
                log = 1;
                break;
            case Fw::TEXT_LOG_COMMAND:
                severityString = "CMD";
                log = 1;
                break;
            case Fw::TEXT_LOG_ACTIVITY_HI:
                severityString = "ACT_HI";
                log = 1;
                break;
            case Fw::TEXT_LOG_ACTIVITY_LO:
                severityString = "ACT_LO";
                log = 1;
                break;
            case Fw::TEXT_LOG_DIAGNOSTIC:
                severityString = "DIAG";
                log = 1;
                break;
            default:
                severityString = "SVRTY ERR";
                log = 1;
                break;
        }

        (void) FARF(log, "EV: (%d) (%d,%d) %s: %s\n",
            id,timeTag.getSeconds(),timeTag.getUSeconds(),severityString,text.toChar());
    }

}
