#ifndef __SDREF_COMPONENTS_HEADER__
#define __SDREF_COMPONENTS_HEADER__
void constructSDREFArchitecture(void);
void exitTasks(void);

#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/SerialTextConverter/SerialTextConverterImpl.hpp>
#include <Svc/ActiveFileLogger/ActiveFileLoggerImpl.hpp>
#include <Svc/SerLogger/SerLoggerComponentImpl.hpp>
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

#include <SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentImpl.hpp>

#include <SnapdragonFlight/MVCam/MVCamComponentImpl.hpp>
#include <SnapdragonFlight/MVVislam/MVVislamComponentImpl.hpp>
#include <SnapdragonFlight/HiresCam/HiresCamComponentImpl.hpp>

#include <SnapdragonFlight/HexRouter/HexRouterComponentImpl.hpp>
#include <Gnc/Ctrl/MultirotorCtrlIface/MultirotorCtrlIfaceComponentImpl.hpp>
#include <Gnc/Est/FilterIface/FilterIfaceComponentImpl.hpp>
#include <HLProc/HLRosIface/HLRosIfaceComponentImpl.hpp>
#include <ROS/RosSeq/RosSeqComponentImpl.hpp>
#include <HLProc/EventExpander/EventExpanderComponentImpl.hpp>
#include <HLProc/LLRouter/LLRouterComponentImpl.hpp>

#include <Svc/UdpReceiver/UdpReceiverComponentImpl.hpp>

#include <Svc/ImgTlm/ImgTlmComponentImpl.hpp>

#include <Svc/IPCRelay/IPCRelayComponentImpl.hpp>
#include <SDREF/Top/TargetInit.hpp>
#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>

#include <Drv/LinuxSerialDriver/LinuxSerialDriverComponentImpl.hpp>
#include <Drv/ForceTorque/ATINetbox/ATINetboxComponentImpl.hpp>

extern Drv::ATINetboxComponentImpl* atiNetbox_ptr;
extern Drv::LinuxSerialDriverComponentImpl* serialDriverLL_ptr;
extern Drv::LinuxSerialDriverComponentImpl* serialDriverDebug_ptr;
extern Svc::SerialTextConverterComponentImpl* serialTextConv_ptr;
extern HLProc::LLRouterComponentImpl* llRouter_ptr;
extern HLProc::EventExpanderComponentImpl* eventExp_ptr;

extern Svc::RateGroupDriverImpl* rgDrv_ptr;
extern Svc::ActiveRateGroupImpl* rgTlm_ptr;
extern Svc::ActiveRateGroupImpl* rgXfer_ptr;
extern Svc::CmdSequencerComponentImpl* cmdSeq_ptr;
extern Svc::CmdSequencerComponentImpl* cmdSeq2_ptr;
extern Svc::SocketGndIfImpl* sockGndIf_ptr;
extern Svc::SocketGndIfImpl* sockGndIfLL_ptr;
extern Svc::ConsoleTextLoggerImpl* textLogger_ptr;
extern Svc::ActiveLoggerImpl* eventLogger_ptr;
extern Svc::ActiveLoggerImpl* eventLoggerLL_ptr;
extern Svc::ActiveFileLoggerImpl* fileLogger_ptr;
extern Svc::SerLoggerComponentImpl* serLogger_ptr;
extern Svc::LinuxTimeImpl* linuxTime_ptr;
extern Svc::TlmChanImpl* chanTlm_ptr;
extern Svc::CommandDispatcherImpl* cmdDisp_ptr;
extern Svc::PrmDbImpl* prmDb_ptr;
extern Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr;
extern Svc::FatalHandlerComponentImpl* fatalHandler_ptr;
extern SnapdragonFlight::MVCamComponentImpl* mvCam_ptr;
extern SnapdragonFlight::MVVislamComponentImpl* mvVislam_ptr;
extern SnapdragonFlight::HiresCamComponentImpl* hiresCam_ptr;
extern SnapdragonFlight::HexRouterComponentImpl* hexRouter_ptr;
extern SnapdragonFlight::SnapdragonHealthComponentImpl* snapHealth_ptr;
extern HLProc::HLRosIfaceComponentImpl* sdRosIface_ptr;
extern Gnc::MultirotorCtrlIfaceComponentImpl* mrCtrlIface_ptr;
extern Gnc::FilterIfaceComponentImpl* filterIface_ptr;
extern ROS::RosSeqComponentImpl* rosSeq_ptr;
extern Svc::UdpReceiverComponentImpl* udpReceiver_ptr;
extern Svc::IPCRelayComponentImpl* ipcRelay_ptr;

extern Svc::ImgTlmComponentImpl* imgTlm_ptr;

#endif
