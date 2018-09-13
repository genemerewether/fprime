#ifndef __LITS_COMPONENTS_HEADER__
#define __LITS_COMPONENTS_HEADER__
void constructHEXREFArchitecture(void);
void exitTasks(void);

#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>

#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Svc/ActiveLogger/ActiveLoggerImpl.hpp>
#include <Svc/LinuxTime/LinuxTimeImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <HEXREF/Top/TargetInit.hpp>
#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>

#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>

#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.hpp>
#include <Drv/LinuxGpioDriver/LinuxGpioDriverComponentImpl.hpp>
#include <Drv/LinuxPwmDriver/LinuxPwmDriverComponentImpl.hpp>
#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include <Gnc/Est/ImuInteg/ImuIntegComponentImpl.hpp>

extern Svc::RateGroupDecouplerComponentImpl rgDecouple;
extern Svc::RateGroupDriverImpl rgGncDrv;
extern Svc::ActiveRateGroupImpl rg;
extern Svc::PassiveRateGroupImpl rgAtt;
extern Svc::PassiveRateGroupImpl rgPos;
extern Svc::ConsoleTextLoggerImpl textLogger;
extern Svc::ActiveLoggerImpl eventLogger;
extern Svc::LinuxTimeImpl linuxTime;
extern Svc::AssertFatalAdapterComponentImpl fatalAdapter;
extern Svc::FatalHandlerComponentImpl fatalHandler;
extern SnapdragonFlight::KraitRouterComponentImpl kraitRouter;
extern Gnc::LeeCtrlComponentImpl leeCtrl;
extern Gnc::ImuIntegComponentImpl imuInteg;
extern Drv::MPU9250ComponentImpl mpu9250;
extern Drv::LinuxSpiDriverComponentImpl spiDrv;
extern Drv::LinuxGpioDriverComponentImpl imuDRInt;
extern Drv::LinuxPwmDriverComponentImpl escPwm;

#endif
