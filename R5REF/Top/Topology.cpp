#include <Components.hpp>

#include <Fw/Types/Assert.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

// Component instance pointers

static NATIVE_INT_TYPE rgGncDivs[] = {10, 1, 1000};
Svc::RateGroupDriverImpl rgGncDrv(
#if FW_OBJECT_NAMES == 1
                    "RGDRV",
#endif
                    rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

static NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    Drv::MPU9250_SCHED_CONTEXT_OPERATE,
    Gnc::IMUINTEG_SCHED_CONTEXT_ATT, // imuInteg
    Gnc::LCTRL_SCHED_CONTEXT_ATT, // leeCtrl
};
Svc::PassiveRateGroupImpl rgAtt(
#if FW_OBJECT_NAMES == 1
                    "RGATT",
#endif
                    rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

static NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    0, //TODO(mereweth) - IMU?
    Gnc::IMUINTEG_SCHED_CONTEXT_POS, // imuInteg
    Gnc::LCTRL_SCHED_CONTEXT_POS, // leeCtrl
};
Svc::PassiveRateGroupImpl rgPos(
#if FW_OBJECT_NAMES == 1
                    "RGPOS",
#endif
                    rgPosContext,FW_NUM_ARRAY_ELEMENTS(rgPosContext));
;

Gnc::LeeCtrlComponentImpl leeCtrl
#if FW_OBJECT_NAMES == 1
                    ("LEECTRL")
#endif
;

Gnc::BasicMixerComponentImpl mixer
#if FW_OBJECT_NAMES == 1
                    ("MIXER")
#endif
;

Gnc::ActuatorAdapterComponentImpl actuatorAdapter
#if FW_OBJECT_NAMES == 1
                    ("ACTADAP")
#endif
;

Gnc::ImuIntegComponentImpl imuInteg
#if FW_OBJECT_NAMES == 1
                    ("IMUINTEG")
#endif
;

Drv::MPU9250ComponentImpl mpu9250(
#if FW_OBJECT_NAMES == 1
                    "MPU9250",
#endif
                     false) // don't use magnetometer for now
;

void manualConstruct(void) {
    // Manual connections
    // TODO(mereweth) - multiple DSPAL components with commands?
    //kraitRouter.set_KraitPortsOut_OutputPort(0, .get_CmdDisp_InputPort(0));
    //.set_CmdStatus_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(0);

    //kraitRouter.set_KraitPortsOut_OutputPort(0, .get_CmdDisp_InputPort(0));
    //.set_CmdStatus_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(0);

    //mpu9250.set_Imu_OutputPort(1, kraitRouter.get_HexPortsIn_InputPort(1));
    //mpu9250.set_FIFORaw_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(2));
    //imuInteg.set_odomNoCov_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(2));

    //kraitRouter.set_KraitPortsOut_OutputPort(1, imuInteg.get_ImuStateUpdate_InputPort(0));
    //kraitRouter.set_KraitPortsOut_OutputPort(2, escPwm.get_pwmSetDuty_InputPort(1));
}

void constructApp() {

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgGncDrv.init();

    // Initialize the rate groups
    rgAtt.init(1);
    rgPos.init(0);

    // Initialize the GNC components
    leeCtrl.init(0);
    mixer.init(0);
    actuatorAdapter.init(0);
    imuInteg.init(0);
    mpu9250.init(0);

    // Connect rate groups to rate group driver
    //constructR5REFArchitecture();

    manualConstruct();
}

void cycleForever(void) {
  while (1) {
    //cycler.runCycles(1);
  }
}
