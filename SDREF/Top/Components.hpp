#ifndef __SDREF_COMPONENTS_HEADER__
#define __SDREF_COMPONENTS_HEADER__
void constructSDREFArchitecture(void);
void exitTasks(void);

#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/SerialTextConverter/SerialTextConverterImpl.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerImpl.hpp>
#include <Svc/CmdDispatcher/CommandDispatcherImpl.hpp>
#include <Svc/CmdSequencer/CmdSequencerImpl.hpp>
#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Svc/ActiveLogger/ActiveLoggerImpl.hpp>
#include <Svc/LinuxTime/LinuxTimeImpl.hpp>
#include <Svc/TlmChan/TlmChanImpl.hpp>
#include <Svc/PrmDb/PrmDbImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <Svc/FileUplink/FileUplink.hpp>
#include <Svc/FileDownlink/FileDownlink.hpp>
#include <Svc/BufferManager/BufferManager.hpp>
#include <Svc/Health/HealthComponentImpl.hpp>

#include <Svc/SocketGndIf/SvcSocketGndIfImpl.hpp>

#include <SnapdragonFlight/HexRouter/HexRouterComponentImpl.hpp>
#include <HLProc/HLRosIface/HLRosIfaceComponentImpl.hpp>
#include <HLProc/EventExpander/EventExpanderComponentImpl.hpp>
#include <HLProc/LLRouter/LLRouterComponentImpl.hpp>

#include <SDREF/Top/TargetInit.hpp>
#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>

#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>

#include <Drv/LinuxSerialDriver/LinuxSerialDriverComponentImpl.hpp>

extern Drv::LinuxSerialDriverComponentImpl* serialDriverLL_ptr;
extern Drv::LinuxSerialDriverComponentImpl* serialDriverDebug_ptr;
extern Svc::SerialTextConverterComponentImpl* serialTextConv_ptr;
extern HLProc::LLRouterComponentImpl* llRouter_ptr;
extern HLProc::EventExpanderComponentImpl* eventExp_ptr;

extern Svc::RateGroupDriverImpl* rgDrv_ptr;
extern Svc::ActiveRateGroupImpl* rgTlm_ptr;
extern Svc::ActiveRateGroupImpl* rgXfer_ptr;
extern Svc::CmdSequencerComponentImpl* cmdSeq_ptr;
extern Svc::CmdSequencerComponentImpl* cmdSeqLL_ptr;
extern Svc::SocketGndIfImpl* sockGndIf_ptr;
extern Svc::SocketGndIfImpl* sockGndIfLL_ptr;
extern Svc::ConsoleTextLoggerImpl* textLogger_ptr;
extern Svc::ActiveLoggerImpl* eventLogger_ptr;
extern Svc::ActiveLoggerImpl* eventLoggerLL_ptr;
extern Svc::ActiveFileLoggerImpl* fileLogger_ptr;
extern Svc::LinuxTimeImpl* linuxTime_ptr;
extern Svc::TlmChanImpl* chanTlm_ptr;
extern Svc::CommandDispatcherImpl* cmdDisp_ptr;
extern Svc::PrmDbImpl* prmDb_ptr;
extern Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr;
extern Svc::FatalHandlerComponentImpl* fatalHandler_ptr;
extern SnapdragonFlight::HexRouterComponentImpl* hexRouter_ptr;
extern HLProc::HLRosIfaceComponentImpl* sdRosIface_ptr;
extern Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr;

#endif
