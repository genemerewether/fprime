#ifndef __R5REF_COMPONENTS_HEADER__
#define __R5REF_COMPONENTS_HEADER__

void constructR5REFArchitecture();
void exitTasks();

#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>

#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Drv/IMU/STIM300/STIM300Impl.hpp>
#include <Drv/Altimeter/LIDARLiteV3/LIDARLiteV3Impl.hpp>
#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include <Gnc/Ctrl/BasicMixer/BasicMixerComponentImpl.hpp>
#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>
#include <Gnc/Est/ImuInteg/ImuIntegComponentImpl.hpp>
#include <Gnc/Sysid/SigGen/SigGenComponentImpl.hpp>

#include <R5/A2DDrv/R5A2DDriverComponentImpl.hpp>
#include <R5/GpioDrv/R5GpioDriverComponentImpl.hpp>
#include <R5/SpiMasterDrv/R5SpiMasterDriverComponentImpl.hpp>
#include <R5/UartDrv/R5UartDriverComponentImpl.hpp>
#include <R5/I2CDrv/R5I2CDriverImpl.hpp>
#include <R5/R5Time/R5TimeComponentImpl.hpp>
#include <R5/R5Prm/R5PrmComponentImpl.hpp>
#include <R5/R5EventCapture/R5EventCaptureImpl.hpp>
#include <R5/R5TimeForward/R5TimeForwardComponentImpl.hpp>
#include <R5/R5Rti/R5RtiComponentImpl.hpp>

#include <R5/GpioAdapter/R5GpioAdapterComponentImpl.hpp>

#include <LLProc/ShortLogQueue/ShortLogQueueComponentImpl.hpp>
#include <LLProc/LLDebug/LLDebugComponentImpl.hpp>
#include <LLProc/LLCycle/LLCycleComponentImpl.hpp>
#include <LLProc/HLRouter/HLRouterComponentImpl.hpp>
#include <LLProc/LLCmdDispatcher/LLCmdDispatcherComponentImpl.hpp>
#include <LLProc/LLTlmChan/LLTlmChanImpl.hpp>

extern Svc::RateGroupDriverImpl rgGncDrv;
extern Svc::PassiveRateGroupImpl rgAtt;
extern Svc::PassiveRateGroupImpl rgPos;
extern Svc::PassiveRateGroupImpl rgTlm;

extern Gnc::LeeCtrlComponentImpl leeCtrl;
extern Gnc::BasicMixerComponentImpl mixer;
extern Gnc::ActuatorAdapterComponentImpl actuatorAdapter;
extern Gnc::SigGenComponentImpl sigGen;
extern Gnc::ImuIntegComponentImpl imuInteg;
extern Drv::MPU9250ComponentImpl mpu9250;
extern Drv::STIM300ComponentImpl stim300;
extern Drv::LIDARLiteV3ComponentImpl lidarLiteV3;

extern R5::R5GpioDriverComponentImpl gpio;
extern R5::R5SpiMasterDriverComponentImpl spiMaster;
extern R5::R5UartDriverComponentImpl hlUart;
extern R5::R5UartDriverComponentImpl debugUart;
extern R5::R5UartDriverComponentImpl stimUart;
extern R5::R5TimeComponentImpl r5Time;
extern R5::R5A2DDriverComponentImpl a2dDrv;
extern R5::R5PrmComponentImpl prm;
extern R5::R5I2CDriverComponentImpl i2c1Drv;
extern R5::R5EventCaptureComponentImpl eventCapture;
extern R5::R5RtiComponentImpl rtiWait;
extern R5::R5TimeForwardComponentImpl tsForward;

extern R5::R5GpioAdapterComponentImpl rtiGpio;
extern R5::R5GpioAdapterComponentImpl faultGpio;

extern LLProc::ShortLogQueueComponentImpl logQueue;
extern LLProc::LLDebugComponentImpl llDebug;
extern LLProc::LLCycleComponentImpl llCycle;
extern LLProc::HLRouterComponentImpl hlRouter;
extern LLProc::LLCmdDispatcherImpl cmdDisp;
extern LLProc::LLTlmChanImpl tlmChan;

#endif
