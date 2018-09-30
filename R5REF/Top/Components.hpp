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

extern Svc::RateGroupDriverImpl rgGncDrv;
extern Svc::PassiveRateGroupImpl rgAtt;
extern Svc::PassiveRateGroupImpl rgPos;
extern Gnc::LeeCtrlComponentImpl leeCtrl;
extern Gnc::BasicMixerComponentImpl mixer;
extern Gnc::ActuatorAdapterComponentImpl actuatorAdapter;
extern Gnc::ImuIntegComponentImpl imuInteg;
extern Drv::MPU9250ComponentImpl mpu9250;

#endif
