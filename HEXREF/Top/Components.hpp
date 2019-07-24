#ifndef __HEXREF_COMPONENTS_HEADER__
#define __HEXREF_COMPONENTS_HEADER__
void constructHEXREFArchitecture(void);
void exitTasks(void);

#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>
#include <Svc/PassiveL2PrmDb/PassiveL2PrmDbImpl.hpp>
#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Svc/LinuxTime/LinuxTimeImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>
#include <Svc/ActiveDecoupler/ActiveDecouplerComponentImpl.hpp>
#include <Svc/QueuedDecoupler/QueuedDecouplerComponentImpl.hpp>

#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>

#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.hpp>
#include <Drv/LinuxI2CDriver/LinuxI2CDriverComponentImpl.hpp>
#include <Drv/LinuxGpioDriver/LinuxGpioDriverComponentImpl.hpp>
#include <Drv/LinuxPwmDriver/LinuxPwmDriverComponentImpl.hpp>
#include <Gnc/Utils/FrameTransform/FrameTransformComponentImpl.hpp>
#include <Gnc/Utils/ImuProc/ImuProcComponentImpl.hpp>
#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include <Gnc/Ctrl/BasicMixer/BasicMixerComponentImpl.hpp>
#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>
#include <Gnc/Sysid/SigGen/SigGenComponentImpl.hpp>
#include <Gnc/Est/AttFilter/AttFilterComponentImpl.hpp>

#include <LLProc/ShortLogQueue/ShortLogQueueComponentImpl.hpp>
#include <LLProc/LLCmdDispatcher/LLCmdDispatcherComponentImpl.hpp>
#include <LLProc/LLTlmChan/LLTlmChanImpl.hpp>

extern Svc::RateGroupDecouplerComponentImpl* rgDecouple_ptr;
extern Svc::QueuedDecouplerComponentImpl* imuDataPasser_ptr;
extern Svc::ActiveDecouplerComponentImpl* imuDecouple_ptr;
extern Svc::ActiveDecouplerComponentImpl* actDecouple_ptr;
extern Svc::RateGroupDriverImpl* rgDcplDrv_ptr;
extern Svc::RateGroupDriverImpl* rgGncDrv_ptr;
extern Svc::PassiveRateGroupImpl* rgAtt_ptr;
extern Svc::PassiveRateGroupImpl* rgPos_ptr;
extern Svc::PassiveRateGroupImpl* rgTlm_ptr;
extern Svc::PassiveRateGroupImpl* rgDev_ptr;
extern Svc::ConsoleTextLoggerImpl* textLogger_ptr;
extern LLProc::ShortLogQueueComponentImpl* logQueue_ptr;
extern LLProc::LLCmdDispatcherImpl* cmdDisp_ptr;
extern LLProc::LLTlmChanImpl* tlmChan_ptr;
extern Svc::PassiveL2PrmDbComponentImpl* prmDb_ptr;
extern Svc::LinuxTimeImpl* linuxTime_ptr;
extern Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr;
extern Svc::FatalHandlerComponentImpl* fatalHandler_ptr;
extern SnapdragonFlight::KraitRouterComponentImpl* kraitRouter_ptr;
extern Gnc::FrameTransformComponentImpl* ctrlXest_ptr;
extern Gnc::ImuProcComponentImpl* imuProc_ptr;
extern Gnc::LeeCtrlComponentImpl* leeCtrl_ptr;
extern Gnc::BasicMixerComponentImpl* mixer_ptr;
extern Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr;
extern Gnc::SigGenComponentImpl* sigGen_ptr;
extern Gnc::AttFilterComponentImpl* attFilter_ptr;
extern Drv::MPU9250ComponentImpl* mpu9250_ptr;
extern Drv::LinuxSpiDriverComponentImpl* spiDrv_ptr;
extern Drv::LinuxI2CDriverComponentImpl* i2cDrv_ptr;
extern Drv::LinuxGpioDriverComponentImpl* imuDRInt_ptr;
extern Drv::LinuxGpioDriverComponentImpl* hwEnablePin_ptr;
extern Drv::LinuxPwmDriverComponentImpl* escPwm_ptr;

#endif
