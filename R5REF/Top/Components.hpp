#ifndef __R5REF_COMPONENTS_HEADER__
#define __R5REF_COMPONENTS_HEADER__
//void constructR5REFArchitecture(void);
void exitTasks(void);

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

extern Svc::RateGroupDriverImpl rgGncDrv;
extern Svc::PassiveRateGroupImpl rgAtt;
extern Svc::PassiveRateGroupImpl rgPos;
extern Gnc::LeeCtrlComponentImpl leeCtrl;
extern Gnc::BasicMixerComponentImpl mixer;
extern Gnc::ActuatorAdapterComponentImpl actuatorAdapter;
extern Gnc::ImuIntegComponentImpl imuInteg;
extern Drv::MPU9250ComponentImpl mpu9250;

extern R5::R5GpioDriverComponentImpl gpio;
extern R5::R5SpiMasterDriverComponentImpl spiMaster;
extern R5::R5UartDriverComponentImpl hlUart;
extern R5::R5UartDriverComponentImpl debugUart;
extern R5::R5TimeComponentImpl r5Time;
extern R5::R5A2DDriverComponentImpl a2dDrv;
extern R5::R5PrmComponentImpl prm;

#endif
