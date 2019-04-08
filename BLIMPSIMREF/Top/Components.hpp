#ifndef __BLIMPSIMREF_COMPONENTS_HEADER__
#define __BLIMPSIMREF_COMPONENTS_HEADER__
void constructBLIMPSIMREFArchitecture(void);
void exitTasks(void);

#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerImpl.hpp>
#include <Svc/CmdDispatcher/CommandDispatcherImpl.hpp>
#include <Svc/CmdSequencer/CmdSequencerImpl.hpp>
#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Svc/ActiveLogger/ActiveLoggerImpl.hpp>
#include <ROS/RosTime/RosTimeImpl.hpp>
#include <Svc/TlmChan/TlmChanImpl.hpp>
#include <Svc/PrmDb/PrmDbImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <Svc/SocketGndIf/SvcSocketGndIfImpl.hpp>
#include <Svc/UdpReceiver/UdpReceiverComponentImpl.hpp>

#include <BLIMPSIMREF/Top/TargetInit.hpp>
#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>

#include <ROS/RosCycle/RosCycleComponentImpl.hpp>
#include <ROS/RosSeq/RosSeqComponentImpl.hpp>
#include <SIMREF/RotorSDrv/RotorSDrvComponentImpl.hpp>
#include <SIMREF/GazeboManipIf/GazeboManipIfComponentImpl.hpp>
#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include <Gnc/Ctrl/BasicMixer/BasicMixerComponentImpl.hpp>
#include <Gnc/Sysid/SigGen/SigGenComponentImpl.hpp>
#include <Gnc/Est/AttFilter/AttFilterComponentImpl.hpp>

extern Svc::RateGroupDecouplerComponentImpl rgDecouple;
extern Svc::RateGroupDriverImpl rgGncDrv;
extern Svc::ActiveRateGroupImpl rg;
extern Svc::PassiveRateGroupImpl rgAtt;
extern Svc::PassiveRateGroupImpl rgPos;
extern Svc::CmdSequencerComponentImpl cmdSeq;
extern Svc::SocketGndIfImpl sockGndIf;
extern Svc::ConsoleTextLoggerImpl textLogger;
extern Svc::ActiveLoggerImpl eventLogger;
extern Svc::ActiveFileLoggerImpl fileLogger;
extern ROS::RosTimeImpl rosTime;
extern Svc::TlmChanImpl chanTlm;
extern Svc::CommandDispatcherImpl cmdDisp;
extern Svc::PrmDbImpl prmDb;
extern Svc::UdpReceiverComponentImpl udpReceiver;
extern Svc::AssertFatalAdapterComponentImpl fatalAdapter;
extern Svc::FatalHandlerComponentImpl fatalHandler;
extern ROS::RosCycleComponentImpl rosCycle;
extern ROS::RosSeqComponentImpl rosSeq;
extern SIMREF::RotorSDrvComponentImpl rotorSDrv;
extern SIMREF::GazeboManipIfComponentImpl gzManipIf;
extern Gnc::LeeCtrlComponentImpl leeCtrl;
extern Gnc::BasicMixerComponentImpl mixer;
extern Gnc::SigGenComponentImpl sigGen;
extern Gnc::AttFilterComponentImpl attFilter;

#endif
