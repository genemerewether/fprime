#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>
#include <HAP_farf.h>

namespace Svc {

    void ConsoleTextLoggerImpl::init(void) {
        PassiveTextLoggerComponentBase::init();
    }
    
    void ConsoleTextLoggerImpl::TextLogger_handler(NATIVE_INT_TYPE portNum, FwEventIdType id, Fw::Time &timeTag, Fw::TextLogSeverity severity, Fw::TextLogString &text) {
        const char *severityString = "UNK";
        switch (severity) {
            case Fw::TEXT_LOG_FATAL:
                severityString = "FAT";
                break;
            case Fw::TEXT_LOG_WARNING_HI:
                severityString = "WRNG_HI";
                break;
            case Fw::TEXT_LOG_WARNING_LO:
                severityString = "WRNG_LO";
                break;
            case Fw::TEXT_LOG_COMMAND:
                severityString = "CMD";
                break;
            case Fw::TEXT_LOG_ACTIVITY_HI:
                severityString = "ACT_HI";
                break;
            case Fw::TEXT_LOG_ACTIVITY_LO:
                severityString = "ACT_LO";
                break;
            case Fw::TEXT_LOG_DIAGNOSTIC:
                severityString = "DIAG";
                break;
            default:
                severityString = "SVRTY ERR";
                break;
        }

        FARF(ALWAYS, "EV: (%d) (%d,%d) %s: %s\n",
            id,timeTag.getSeconds(),timeTag.getUSeconds(),severityString,text.toChar());
    }

}
