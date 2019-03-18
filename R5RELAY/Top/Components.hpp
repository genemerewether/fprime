#ifndef __R5RELAY_COMPONENTS_HEADER__
#define __R5RELAY_COMPONENTS_HEADER__
void constructR5RELAYArchitecture(void);
void exitTasks(void);

#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerImpl.hpp>
#include <Svc/CmdDispatcher/CommandDispatcherImpl.hpp>
#include <Svc/CmdSequencer/CmdSequencerImpl.hpp>
#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Svc/ActiveLogger/ActiveLoggerImpl.hpp>
#include <Svc/LinuxTime/LinuxTimeImpl.hpp>
#include <Svc/TlmChan/TlmChanImpl.hpp>
#include <Svc/PrmDb/PrmDbImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <Svc/SocketGndIf/SvcSocketGndIfImpl.hpp>
#include <Svc/UdpReceiver/UdpReceiverComponentImpl.hpp>

#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>

#include <Drv/LinuxSerialDriver/LinuxSerialDriverComponentImpl.hpp>
#include <HLProc/LLRouter/LLRouterComponentImpl.hpp>
#include <HLProc/EventExpander/EventExpanderComponentImpl.hpp>

extern Drv::LinuxSerialDriverComponentImpl serialDrv;
extern Svc::PassiveRateGroupImpl rg;
extern HLProc::LLRouterComponentImpl llRouter;
extern Svc::LinuxTimeImpl linuxTime;
extern HLProc::EventExpanderComponentImpl eventExpander;
extern Svc::SocketGndIfImpl sockGndIf;
extern Svc::ActiveLoggerImpl activeLogger;

#endif
