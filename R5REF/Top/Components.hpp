#ifndef __R5REF_COMPONENTS_HEADER__
#define __R5REF_COMPONENTS_HEADER__

void constructR5REFArchitecture();
void exitTasks();

#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>

#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include <Gnc/Ctrl/BasicMixer/BasicMixerComponentImpl.hpp>
#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>
#include <Gnc/Est/ImuInteg/ImuIntegComponentImpl.hpp>

#include <R5/A2DDrv/R5A2DDriverComponentImpl.hpp>
#include <R5/GpioDrv/R5GpioDriverComponentImpl.hpp>
#include <R5/SpiMasterDrv/R5SpiMasterDriverComponentImpl.hpp>
#include <R5/UartDrv/R5UartDriverComponentImpl.hpp>
#include <R5/R5Time/R5TimeComponentImpl.hpp>
#include <R5/R5Prm/R5PrmComponentImpl.hpp>

extern Svc::RateGroupDriverImpl* rgGncDrv_ptr;
extern Svc::PassiveRateGroupImpl* rgAtt_ptr;
extern Svc::PassiveRateGroupImpl* rgPos_ptr;

extern Gnc::LeeCtrlComponentImpl* leeCtrl_ptr;
extern Gnc::BasicMixerComponentImpl* mixer_ptr;
extern Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr;
extern Gnc::ImuIntegComponentImpl* imuInteg_ptr;
extern Drv::MPU9250ComponentImpl* mpu9250_ptr;

extern R5::R5GpioDriverComponentImpl* gpio_ptr;
extern R5::R5SpiMasterDriverComponentImpl* spiMaster_ptr;
extern R5::R5UartDriverComponentImpl* hlUart_ptr;
extern R5::R5UartDriverComponentImpl* debugUart_ptr;
extern R5::R5TimeComponentImpl* r5Time_ptr;
extern R5::R5A2DDriverComponentImpl* a2dDrv_ptr;
extern R5::R5PrmComponentImpl* prm_ptr;

#endif
