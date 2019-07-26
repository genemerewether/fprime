#ifndef __BLIMPREF_COMPONENTS_HEADER__
#define __BLIMPREF_COMPONENTS_HEADER__
void constructBLIMPREFArchitecture(void);
void exitTasks(void);

#include <Svc/CmdDispatcher/CommandDispatcherImpl.hpp>
#include <Svc/CmdSequencer/CmdSequencerImpl.hpp>
#include <Svc/ActiveTextLogger/ActiveTextLoggerImpl.hpp>
#include <Svc/ActiveLogger/ActiveLoggerImpl.hpp>
#include <Svc/TlmChan/TlmChanImpl.hpp>
#include <Svc/ActiveL1PrmDb/ActiveL1PrmDbImpl.hpp>
#include <Svc/SocketGndIf/SvcSocketGndIfImpl.hpp>
#include <SnapdragonFlight/SnapdragonHealth/SnapdragonHealthComponentImpl.hpp>
#include <Gnc/Ctrl/MultirotorCtrlIface/MultirotorCtrlIfaceComponentImpl.hpp>
#include <Gnc/Est/FilterIface/FilterIfaceComponentImpl.hpp>
#include <Gnc/Est/GroundTruthIface/GroundTruthIfaceComponentImpl.hpp>
#include <HLProc/HLRosIface/HLRosIfaceComponentImpl.hpp>
#include <ROS/RosSeq/RosSeqComponentImpl.hpp>
#include <Svc/TimeConvert/TimeConvertComponentImpl.hpp>
#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>
#include <Svc/LinuxTime/LinuxTimeImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>
#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>
#include <Svc/ActiveDecoupler/ActiveDecouplerComponentImpl.hpp>
#include <Svc/QueuedDecoupler/QueuedDecouplerComponentImpl.hpp>
#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <SnapdragonFlight/BlspSpiDriver/BlspSpiDriverComponentImpl.hpp>
#include <SnapdragonFlight/BlspI2CDriver/BlspI2CDriverComponentImpl.hpp>
#include <SnapdragonFlight/BlspPwmDriver/BlspPwmDriverComponentImpl.hpp>
#include <SnapdragonFlight/BlspGpioDriver/BlspGpioDriverComponentImpl.hpp>
#include <Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.hpp>
#include <Drv/LinuxI2CDriver/LinuxI2CDriverComponentImpl.hpp>
#include <Drv/LinuxGpioDriver/LinuxGpioDriverComponentImpl.hpp>
#include <Drv/LinuxPwmDriver/LinuxPwmDriverComponentImpl.hpp>
#include <Gnc/Utils/FrameTransform/FrameTransformComponentImpl.hpp>
#include <Gnc/Utils/ImuProc/ImuProcComponentImpl.hpp>
#include <Gnc/Utils/FixedAxisSe3Adapter/FixedAxisSe3AdapterComponentImpl.hpp>
#include <Gnc/Ctrl/Se3Ctrl/Se3CtrlComponentImpl.hpp>
#include <Gnc/Ctrl/WrenchMixer/WrenchMixerComponentImpl.hpp>
#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>
#include <Gnc/Sysid/SigGen/SigGenComponentImpl.hpp>
#include <Gnc/Est/AttFilter/AttFilterComponentImpl.hpp>

extern Svc::RateGroupDecouplerComponentImpl* rgDecouple_ptr;
extern Svc::QueuedDecouplerComponentImpl* passiveDataPasser_ptr;
extern Svc::ActiveDecouplerComponentImpl* imuDecouple_ptr;
extern Svc::ActiveDecouplerComponentImpl* actDecouple_ptr;
extern Svc::RateGroupDriverImpl* rgDcplDrv_ptr;
extern Svc::RateGroupDriverImpl* rgGncDrv_ptr;
extern Svc::PassiveRateGroupImpl* rgOp_ptr;
extern Svc::PassiveRateGroupImpl* rgTlm_ptr;
extern Svc::PassiveRateGroupImpl* rgDev_ptr;
extern Svc::LinuxTimeImpl* linuxTime_ptr;
extern Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr;
extern Svc::FatalHandlerComponentImpl* fatalHandler_ptr;
extern Gnc::FrameTransformComponentImpl* ctrlXest_ptr;
extern Gnc::ImuProcComponentImpl* imuProc_ptr;
extern Gnc::FixedAxisSe3AdapterComponentImpl* axSe3Adap_ptr;
extern Gnc::Se3CtrlComponentImpl* se3Ctrl_ptr;
extern Gnc::WrenchMixerComponentImpl* mixer_ptr;
extern Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr;
extern Gnc::SigGenComponentImpl* sigGen_ptr;
extern Gnc::AttFilterComponentImpl* attFilter_ptr;
extern Drv::MPU9250ComponentImpl* mpu9250_ptr;

extern SnapdragonFlight::BlspSpiDriverComponentImpl* spiDrvSnap_ptr;
extern SnapdragonFlight::BlspI2CDriverComponentImpl* i2cDrvSnap_ptr;
extern SnapdragonFlight::BlspI2CDriverComponentImpl* i2cDrvSnap2_ptr;
extern SnapdragonFlight::BlspGpioDriverComponentImpl* imuDRIntSnap_ptr;
extern SnapdragonFlight::BlspGpioDriverComponentImpl* hwEnablePinSnap_ptr;
extern SnapdragonFlight::BlspPwmDriverComponentImpl* escPwmSnap_ptr;

extern Drv::LinuxSpiDriverComponentImpl* spiDrv_ptr;
extern Drv::LinuxI2CDriverComponentImpl* i2cDrv_ptr;
extern Drv::LinuxI2CDriverComponentImpl* i2cDrv2_ptr;
extern Drv::LinuxGpioDriverComponentImpl* imuDRInt_ptr;
extern Drv::LinuxGpioDriverComponentImpl* hwEnablePin_ptr;
extern Drv::LinuxPwmDriverComponentImpl* escPwm_ptr;

extern Svc::CommandDispatcherImpl* cmdDisp_ptr;
extern Svc::CmdSequencerComponentImpl* cmdSeq_ptr;
extern Svc::ActiveTextLoggerComponentImpl* textLogger_ptr;
extern Svc::ActiveLoggerImpl* eventLogger_ptr;
extern Svc::TlmChanImpl* chanTlm_ptr;
extern Svc::ActiveL1PrmDbImpl* prmDb_ptr;
extern Svc::SocketGndIfImpl* sockGndIf_ptr;
extern SnapdragonFlight::SnapdragonHealthComponentImpl* snapHealth_ptr;
extern Svc::TimeConvertComponentImpl* timeConvert_ptr;

extern HLProc::HLRosIfaceComponentImpl* hlRosIface_ptr;
extern Gnc::MultirotorCtrlIfaceComponentImpl* mrCtrlIface_ptr;
extern Gnc::FilterIfaceComponentImpl* filterIface_ptr;
extern Gnc::GroundTruthIfaceComponentImpl* gtIface_ptr;
extern ROS::RosSeqComponentImpl* rosSeq_ptr;

#endif
