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

#include <R5/GpioAdapter/R5GpioAdapterComponentImpl.hpp>

#include <LLProc/ShortLogQueue/ShortLogQueueComponentImpl.hpp>
#include <LLProc/LLDebug/LLDebugComponentImpl.hpp>
#include <LLProc/LLCycle/LLCycleComponentImpl.hpp>
#include <LLProc/HLRouter/HLRouterComponentImpl.hpp>
#include <LLProc/LLCmdDispatcher/LLCmdDispatcherComponentImpl.hpp>
#include <LLProc/LLTlmChan/LLTlmChanImpl.hpp>

extern Svc::RateGroupDriverImpl* rgGncDrv_ptr;
extern Svc::PassiveRateGroupImpl* rgAtt_ptr;
extern Svc::PassiveRateGroupImpl* rgPos_ptr;
extern Svc::PassiveRateGroupImpl* rgTlm_ptr;

extern Gnc::LeeCtrlComponentImpl* leeCtrl_ptr;
extern Gnc::BasicMixerComponentImpl* mixer_ptr;
extern Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr;
extern Gnc::SigGenComponentImpl* sigGen_ptr;
extern Gnc::ImuIntegComponentImpl* imuInteg_ptr;
extern Drv::MPU9250ComponentImpl* mpu9250_ptr;
extern Drv::STIM300ComponentImpl* stim300_ptr;
extern Drv::LIDARLiteV3ComponentImpl* lidarLiteV3_ptr;

extern R5::R5GpioDriverComponentImpl* gpio_ptr;
extern R5::R5SpiMasterDriverComponentImpl* spiMaster_ptr;
extern R5::R5UartDriverComponentImpl* hlUart_ptr;
extern R5::R5UartDriverComponentImpl* debugUart_ptr;
extern R5::R5UartDriverComponentImpl* stimUart_ptr;
extern R5::R5TimeComponentImpl* r5Time_ptr;
extern R5::R5A2DDriverComponentImpl* a2dDrv_ptr;
extern R5::R5PrmComponentImpl* prm_ptr;
extern R5::R5I2CDriverComponentImpl* i2c1Drv_ptr;
extern R5::R5EventCaptureComponentImpl* eventCapture_ptr;

extern R5::R5GpioAdapterComponentImpl* rtiGpio_ptr;
extern R5::R5GpioAdapterComponentImpl* faultGpio_ptr;

extern LLProc::ShortLogQueueComponentImpl* logQueue_ptr;
extern LLProc::LLDebugComponentImpl* llDebug_ptr;
extern LLProc::LLCycleComponentImpl* llCycle_ptr;
extern LLProc::HLRouterComponentImpl* hlRouter_ptr;
extern LLProc::LLCmdDispatcherImpl* cmdDisp_ptr;
extern LLProc::LLTlmChanImpl* tlmChan_ptr;

#endif
