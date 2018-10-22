#include <Components.hpp>

#include <Fw/Types/Assert.hpp>
#include <R5/R5Mem/R5DmaAllocator.hpp>
#include <R5/DmaDrv/DmaDrv.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

enum UartInstances {
  HL_UART_INSTANCE = 2,
  DEBUG_UART_INSTANCE = 0,
};

// Component instance pointers

Svc::RateGroupDriverImpl* rgGncDrv_ptr = 0;
Svc::PassiveRateGroupImpl* rgAtt_ptr = 0;
Svc::PassiveRateGroupImpl* rgPos_ptr = 0;

Gnc::LeeCtrlComponentImpl* leeCtrl_ptr = 0;
Gnc::BasicMixerComponentImpl* mixer_ptr = 0;
Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr = 0;
Gnc::ImuIntegComponentImpl* imuInteg_ptr = 0;
Drv::MPU9250ComponentImpl* mpu9250_ptr = 0;

R5::R5GpioDriverComponentImpl* gpio_ptr = 0;
R5::R5SpiMasterDriverComponentImpl* spiMaster_ptr = 0;
R5::R5UartDriverComponentImpl* hlUart_ptr = 0;
R5::R5UartDriverComponentImpl* debugUart_ptr = 0;
R5::R5TimeComponentImpl* r5Time_ptr = 0;
R5::R5A2DDriverComponentImpl* a2dDrv_ptr = 0;
R5::R5PrmComponentImpl* prm_ptr = 0;

static R5DmaAllocator alloc;

void allocComps() {
    NATIVE_INT_TYPE rgGncDivs[] = {10, 1, 1000};
    rgGncDrv_ptr = new Svc::RateGroupDriverImpl(
    #if FW_OBJECT_NAMES == 1
                        "RGDRV",
    #endif
                        rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

    NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Drv::MPU9250_SCHED_CONTEXT_OPERATE,
        Gnc::IMUINTEG_SCHED_CONTEXT_ATT, // imuInteg
        Gnc::LCTRL_SCHED_CONTEXT_ATT, // leeCtrl
    };
    rgAtt_ptr = new Svc::PassiveRateGroupImpl(
    #if FW_OBJECT_NAMES == 1
                        "RGATT",
    #endif
                        rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
    ;

    NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        0, //TODO(mereweth) - IMU?
        Gnc::IMUINTEG_SCHED_CONTEXT_POS, // imuInteg
        Gnc::LCTRL_SCHED_CONTEXT_POS, // leeCtrl
    };
    rgPos_ptr = new Svc::PassiveRateGroupImpl(
    #if FW_OBJECT_NAMES == 1
                        "RGPOS",
    #endif
                        rgPosContext,FW_NUM_ARRAY_ELEMENTS(rgPosContext));
    ;

    leeCtrl_ptr = new Gnc::LeeCtrlComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("LEECTRL")
    #endif
    ;

    mixer_ptr = new Gnc::BasicMixerComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("MIXER")
    #endif
    ;

    actuatorAdapter_ptr = new Gnc::ActuatorAdapterComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("ACTADAP")
    #endif
    ;

   imuInteg_ptr = new Gnc::ImuIntegComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("IMUINTEG")
    #endif
    ;

    mpu9250_ptr = new Drv::MPU9250ComponentImpl(
    #if FW_OBJECT_NAMES == 1
                        "MPU9250",
    #endif
                         false) // don't use magnetometer for now
    ;

    gpio_ptr = new R5::R5GpioDriverComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("gpio")
    #endif
    ;

    spiMaster_ptr = new R5::R5SpiMasterDriverComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("spiMaster")
    #endif
    ;

    hlUart_ptr = new R5::R5UartDriverComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("hlUart")
    #endif
    ;

    debugUart_ptr = new R5::R5UartDriverComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("debugUart")
    #endif
    ;

    r5Time_ptr = new R5::R5TimeComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("time")
    #endif
    ;

    a2dDrv_ptr = new R5::R5A2DDriverComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("a2d")
    #endif
    ;

    prm_ptr = new R5::R5PrmComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("prm")
    #endif
    ;
}

void manualConstruct() {
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
    allocComps();

    // Initialize rate group driver
    rgGncDrv_ptr->init();
    // Initialize the rate groups
    rgAtt_ptr->init(1);
    rgPos_ptr->init(0);

    // Initialize the GNC components
    leeCtrl_ptr->init(0);
    mixer_ptr->init(0);
    actuatorAdapter_ptr->init(0);
    imuInteg_ptr->init(0);
    mpu9250_ptr->init(0);

    // initialize GPIO
    gpio_ptr->init(0);

    // inialize SPI drivers
    spiMaster_ptr->init(0);
    spiMaster_ptr->initDriver(0,
                              0, (256 * 2),
                              0, 256,
                              alloc);

    hlUart_ptr->init(HL_UART_INSTANCE);
    hlUart_ptr->initDriver(0,200, // bytes - max we could send in one cycle
                           0,400, // bytes - two cycles worth
                           alloc);

    debugUart_ptr->init(DEBUG_UART_INSTANCE);
    debugUart_ptr->initDriver(0,200, // bytes - max we could send in one cycle
                              0,0, // bytes - two cycles worth
                              alloc);

    DmaDrvInit();

    r5Time_ptr->init();

    prm_ptr->init(0);

    // Connect rate groups to rate group driver
    //constructR5REFArchitecture();

    manualConstruct();

    // load parameters from flash
    prm_ptr->load();
}

void cycleForever(void) {
  while (1) {
    //cycler.runCycles(1);
  }
}
