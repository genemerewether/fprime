#include <Components.hpp>

#include <Fw/Types/Assert.hpp>
#include <R5/R5Mem/R5DmaAllocator.hpp>
#include <R5/DmaDrv/DmaDrv.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#ifdef BUILD_TIR5
#include <R5/TiHal/include/HL_rti.h>
#endif

enum UartInstances {
  HL_UART_INSTANCE = 2,
  DEBUG_UART_INSTANCE = 0,
};

// Component instance pointers

Svc::RateGroupDriverImpl* rgGncDrv_ptr = 0;
Svc::PassiveRateGroupImpl* rgAtt_ptr = 0;
Svc::PassiveRateGroupImpl* rgPos_ptr = 0;
Svc::PassiveRateGroupImpl* rgTlm_ptr = 0;

Gnc::LeeCtrlComponentImpl* leeCtrl_ptr = 0;
Gnc::BasicMixerComponentImpl* mixer_ptr = 0;
Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr = 0;
Gnc::SigGenComponentImpl* sigGen_ptr = 0;
Gnc::ImuIntegComponentImpl* imuInteg_ptr = 0;
Drv::MPU9250ComponentImpl* mpu9250_ptr = 0;
Drv::LIDARLiteV3ComponentImpl* lidarLiteV3_ptr = 0;

R5::R5GpioDriverComponentImpl* gpio_ptr = 0;
R5::R5SpiMasterDriverComponentImpl* spiMaster_ptr = 0;
R5::R5UartDriverComponentImpl* hlUart_ptr = 0;
R5::R5UartDriverComponentImpl* debugUart_ptr = 0;
R5::R5TimeComponentImpl* r5Time_ptr = 0;
R5::R5A2DDriverComponentImpl* a2dDrv_ptr = 0;
R5::R5PrmComponentImpl* prm_ptr = 0;
R5::R5I2CDriverComponentImpl* i2c1Drv_ptr = 0;

R5::R5GpioAdapterComponentImpl* rtiGpio_ptr = 0;
R5::R5GpioAdapterComponentImpl* faultGpio_ptr = 0;

LLProc::ShortLogQueueComponentImpl* logQueue_ptr = 0;
LLProc::LLDebugComponentImpl* llDebug_ptr = 0;
LLProc::LLCycleComponentImpl* llCycle_ptr = 0;
LLProc::HLRouterComponentImpl* hlRouter_ptr = 0;
LLProc::LLCmdDispatcherImpl* cmdDisp_ptr = 0;
LLProc::LLTlmChanImpl* tlmChan_ptr = 0;


static R5DmaAllocator alloc;

void allocComps() {
    NATIVE_UINT_TYPE rgTlmContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Drv::MPU9250_SCHED_CONTEXT_TLM, // mpu9250
        Gnc::IMUINTEG_SCHED_CONTEXT_TLM, // imuInteg
        Gnc::LCTRL_SCHED_CONTEXT_TLM, // leeCtrl
        0, // mixer
        0, // logQueue
        0, // chanTlm
        LLProc::HLRTR_SCHED_UART_SEND,
        Drv::LLV3_RG_MEASURE
    };

    rgTlm_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGTLM",
#endif
                            rgTlmContext,FW_NUM_ARRAY_ELEMENTS(rgTlmContext));
;

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
        0, //mixer
        0, //logQueue
        LLProc::HLRTR_SCHED_UART_SEND,
        LLProc::HLRTR_SCHED_UART_RECEIVE,
        0,
        Drv::LLV3_RG_FAST
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
        0, //mixer
        0, //logQueue
        LLProc::HLRTR_SCHED_UART_SEND,
        LLProc::HLRTR_SCHED_UART_RECEIVE,
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
 
    sigGen_ptr = new Gnc::SigGenComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SIGGEN")
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

    rtiGpio_ptr = new R5::R5GpioAdapterComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("rtigpio")
    #endif
    ;

    faultGpio_ptr = new R5::R5GpioAdapterComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("faultgpio")
    #endif
    ;

    logQueue_ptr = new LLProc::ShortLogQueueComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("slog")
    #endif
    ;

    llDebug_ptr = new LLProc::LLDebugComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("dbg")
    #endif
    ;

    llCycle_ptr = new LLProc::LLCycleComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("cycle")
    #endif
    ;

    hlRouter_ptr = new LLProc::HLRouterComponentImpl
    #if FW_OBJECT_NAMES == 1
                        ("hlRouter")
    #endif
    ;

    cmdDisp_ptr = new LLProc::LLCmdDispatcherImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDDISP")
#endif
;

    tlmChan_ptr = new LLProc::LLTlmChanImpl
#if FW_OBJECT_NAMES == 1
                        ("TLMCHAN")
#endif
;

    lidarLiteV3_ptr = new Drv::LIDARLiteV3ComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("LLV3")
#endif
;

    i2c1Drv_ptr = new R5::R5I2CDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("I2C1")
#endif
;

}

void manualConstruct() {
    // Manual connections
    hlRouter_ptr->set_HLPortsOut_OutputPort(0, cmdDisp_ptr->get_seqCmdBuff_InputPort(0));
    cmdDisp_ptr->set_seqCmdStatus_OutputPort(0, hlRouter_ptr->get_LLPortsIn_InputPort(0));

    mpu9250_ptr->set_Imu_OutputPort(1, hlRouter_ptr->get_LLPortsIn_InputPort(1));
    //imuInteg_ptr->set_odomNoCov_OutputPort(0, hlRouter_ptr->get_LLPortsIn_InputPort(2));
    leeCtrl_ptr->set_accelCommand_OutputPort(0, hlRouter_ptr->get_LLPortsIn_InputPort(3));
    logQueue_ptr->set_LogSend_OutputPort(0,hlRouter_ptr->get_LLPortsIn_InputPort(4));
    tlmChan_ptr->set_PktSend_OutputPort(0,hlRouter_ptr->get_LLPortsIn_InputPort(5));
    actuatorAdapter_ptr->set_serialDat_OutputPort(0, hlRouter_ptr->get_LLPortsIn_InputPort(6));

    hlRouter_ptr->set_HLPortsOut_OutputPort(1, imuInteg_ptr->get_ImuStateUpdate_InputPort(0));
    hlRouter_ptr->set_HLPortsOut_OutputPort(2, actuatorAdapter_ptr->get_motor_InputPort(1));
    // aux actuator command
    hlRouter_ptr->set_HLPortsOut_OutputPort(4, cmdDisp_ptr->get_seqCmdBuff_InputPort(2));
    hlRouter_ptr->set_HLPortsOut_OutputPort(5, leeCtrl_ptr->get_flatOutput_InputPort(0));
    hlRouter_ptr->set_HLPortsOut_OutputPort(6, leeCtrl_ptr->get_attRateThrust_InputPort(0));
    hlRouter_ptr->set_HLPortsOut_OutputPort(7, leeCtrl_ptr->get_attRateThrust_InputPort(0));

    llDebug_ptr->set_SerWritePort_OutputPort(0, debugUart_ptr->get_serialSend_InputPort(0));

    hlRouter_ptr->set_HLPortsOut_OutputPort(8, cmdDisp_ptr->get_seqCmdBuff_InputPort(1));
    cmdDisp_ptr->set_seqCmdStatus_OutputPort(1, hlRouter_ptr->get_LLPortsIn_InputPort(8));

    hlRouter_ptr->set_HLPortsOut_OutputPort(9, actuatorAdapter_ptr->get_flySafe_InputPort(0));
}

void constructApp() {
    allocComps();

    // Initialize rate group driver
    rgGncDrv_ptr->init();
    // Initialize the rate groups
    rgAtt_ptr->init(1);
    rgPos_ptr->init(0);
    rgTlm_ptr->init(2);

    // Initialize the GNC components
    leeCtrl_ptr->init(0);
    mixer_ptr->init(0);
    actuatorAdapter_ptr->init(0);
    sigGen_ptr->init(0);
    imuInteg_ptr->init(0);
    mpu9250_ptr->init(0);

    hlRouter_ptr->init(0);
    cmdDisp_ptr->init(0);
    tlmChan_ptr->init(0);

    a2dDrv_ptr->init(0);

    // initialize GPIO
    gpio_ptr->init(0);

    rtiGpio_ptr->init(0);
    faultGpio_ptr->init(0);

    // inialize SPI drivers
    spiMaster_ptr->init(0);
    spiMaster_ptr->initDriver(0,
                              0, 64,
                              0, 512,
                              alloc);

    hlUart_ptr->init(HL_UART_INSTANCE);
    hlUart_ptr->initDriver(0,1000, // bytes - max we could send in one cycle
                           0,1600, // bytes - two cycles worth
                           alloc);

    debugUart_ptr->init(DEBUG_UART_INSTANCE);
    debugUart_ptr->initDriver(0,200, // bytes - max we could send in one cycle
                              0,0, // bytes - two cycles worth
                              alloc);

    DmaDrvInit();

    r5Time_ptr->init();

    prm_ptr->init(0);

    logQueue_ptr->init(0);

    llDebug_ptr->init(0);
    llCycle_ptr->init(0);

    lidarLiteV3_ptr->init(0);

    i2c1Drv_ptr->init(0);
    i2c1Drv_ptr->initDriver(128, 128, alloc);

    // Connect rate groups to rate group driver
    constructR5REFArchitecture();

    manualConstruct();

    /* Register commands */
    cmdDisp_ptr->regCommands();

    leeCtrl_ptr->regCommands();
    imuInteg_ptr->regCommands();
    mixer_ptr->regCommands();
    actuatorAdapter_ptr->regCommands();
    sigGen_ptr->regCommands();

    rtiGpio_ptr->waitMapping(R5::GPIO_WAIT_BANK_A, 2);
    faultGpio_ptr->setMapping(R5::GPIO_SET_BANK_A, 0);

    // load parameters from flash
    prm_ptr->load();
}

void cycleForever(void) {
    while (!mpu9250_ptr->isReady()) {
        Svc::InputCyclePort* port = rgGncDrv_ptr->get_CycleIn_InputPort(0);
        Svc::TimerVal cycleStart;
        cycleStart.take();
        port->invoke(cycleStart);

#ifdef BUILD_TIR5
        U32 start = rtiGetMedResTimestamp();
        bool keepGoing = true;
        while (keepGoing) {
            U32 stampNow = rtiGetMedResTimestamp();
            // NOTE(mereweth) - rollover occurred
            if (stampNow < start) {
                start = 0; // restarts this timer, but that is OK
            }

            // one millisecond elapsed
            if (stampNow > start + 1000) {
                keepGoing = false;
            }
        }
#endif
    }

    while (1) {
        llCycle_ptr->runCycles(1);
    }
}
