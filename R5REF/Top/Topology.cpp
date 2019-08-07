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
  STIM_UART_INSTANCE = 1,
};

enum EventCaptuerInstances {
  STIM_TOV_INSTANCE = 0,
  TIME_SYNC_INSTANCE = 1,
};

static R5DmaAllocator alloc;

static NATIVE_UINT_TYPE rgTlmContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    Drv::MPU9250_SCHED_CONTEXT_TLM, // mpu9250
    Gnc::IMUINTEG_SCHED_CONTEXT_TLM, // imuInteg
    Gnc::LCTRL_SCHED_CONTEXT_TLM, // leeCtrl
    0,
    0, // logQueue
    0 // chanTlm
};

Svc::PassiveRateGroupImpl rgTlm(
#if FW_OBJECT_NAMES == 1
                                "RGTLM",
#endif
                                rgTlmContext,FW_NUM_ARRAY_ELEMENTS(rgTlmContext));
;

static NATIVE_INT_TYPE rgGncDivs[] = {10, 1, 500};
Svc::RateGroupDriverImpl rgGncDrv(
#if FW_OBJECT_NAMES == 1
                                  "RGDRV",
#endif
                                  rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

static NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    0, //STIM300
    Drv::MPU9250_SCHED_CONTEXT_OPERATE,
    Gnc::IMUINTEG_SCHED_CONTEXT_FILT, // imuInteg
    Gnc::LCTRL_SCHED_CONTEXT_ATT, // leeCtrl
    0, //mixer
    0, //logQueue
    LLProc::HLRTR_SCHED_UART_SEND,
    LLProc::HLRTR_SCHED_UART_RECEIVE,
    0,
    Drv::LLV3_RG_FAST,
    0, // Time Forward
};

Svc::PassiveRateGroupImpl rgAtt(
#if FW_OBJECT_NAMES == 1
                                "RGATT",
#endif
                                rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

static NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    0, //TODO(mereweth) - actuatoradapter arm
    Gnc::LCTRL_SCHED_CONTEXT_POS, // leeCtrl
    Drv::LLV3_RG_MEASURE,
    0,
    0, //logQueue
};

Svc::PassiveRateGroupImpl rgPos(
#if FW_OBJECT_NAMES == 1
                                "RGPOS",
#endif
                                rgPosContext,FW_NUM_ARRAY_ELEMENTS(rgPosContext));
;

Svc::PassiveL2PrmDbComponentImpl prmDb
#if FW_OBJECT_NAMES == 1
                        ("PRMDB", 1024) //1024 max receive
#endif
;

Gnc::FrameTransformComponentImpl ctrlXest
#if FW_OBJECT_NAMES == 1
                        ("CTRLXEST")
#endif
;

Gnc::ImuProcComponentImpl imuProc
#if FW_OBJECT_NAMES == 1
                        ("IMUPROC")
#endif
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
 
Gnc::SigGenComponentImpl sigGen
#if FW_OBJECT_NAMES == 1
                        ("SIGGEN")
#endif
;

Gnc::ImuIntegComponentImpl imuInteg
#if FW_OBJECT_NAMES == 1
                        ("IMUINTEG")
#endif
;

Drv::MPU9250ComponentImpl mpu9250 (
#if FW_OBJECT_NAMES == 1
                                   "MPU9250",
#endif
                                   false) // don't use magnetometer for now
;

R5::R5GpioDriverComponentImpl gpio
#if FW_OBJECT_NAMES == 1
                        ("gpio")
#endif
;

R5::R5SpiMasterDriverComponentImpl spiMaster
#if FW_OBJECT_NAMES == 1
                        ("spiMaster")
#endif
;

R5::R5UartDriverComponentImpl hlUart
#if FW_OBJECT_NAMES == 1
                        ("hlUart")
#endif
;

R5::R5UartDriverComponentImpl debugUart
#if FW_OBJECT_NAMES == 1
                        ("debugUart")
#endif
;

R5::R5TimeComponentImpl r5Time
#if FW_OBJECT_NAMES == 1
                        ("time")
#endif
;

R5::R5A2DDriverComponentImpl a2dDrv
#if FW_OBJECT_NAMES == 1
                        ("a2d")
#endif
;

R5::R5GpioAdapterComponentImpl faultGpio
#if FW_OBJECT_NAMES == 1
                        ("faultgpio")
#endif
;

LLProc::ShortLogQueueComponentImpl logQueue
#if FW_OBJECT_NAMES == 1
                        ("slog")
#endif
;

LLProc::LLDebugComponentImpl llDebug
#if FW_OBJECT_NAMES == 1
                        ("dbg")
#endif
;

LLProc::LLCycleComponentImpl llCycle
#if FW_OBJECT_NAMES == 1
                        ("cycle")
#endif
;

LLProc::HLRouterComponentImpl hlRouter
#if FW_OBJECT_NAMES == 1
                        ("hlRouter")
#endif
;

LLProc::LLCmdDispatcherImpl cmdDisp
#if FW_OBJECT_NAMES == 1
                        ("CMDDISP")
#endif
;

LLProc::LLTlmChanImpl tlmChan
#if FW_OBJECT_NAMES == 1
                        ("TLMCHAN")
#endif
;

Drv::LIDARLiteV3ComponentImpl lidarLiteV3
#if FW_OBJECT_NAMES == 1
                        ("LLV3")
#endif
;

R5::R5I2CDriverComponentImpl i2c1Drv
#if FW_OBJECT_NAMES == 1
                        ("I2C1")
#endif
;

R5::R5UartDriverComponentImpl stimUart
#if FW_OBJECT_NAMES == 1
                        ("stimUart")
#endif
;

Drv::STIM300ComponentImpl stim300
#if FW_OBJECT_NAMES == 1
                        ("stim300")
#endif
;

R5::R5EventCaptureComponentImpl eventCapture
#if FW_OBJECT_NAMES == 1
                        ("eventCapture")
#endif
;

R5::R5RtiComponentImpl rtiWait
#if FW_OBJECT_NAMES == 1
                        ("rti")
#endif
;

R5::R5TimeForwardComponentImpl tsForward
#if FW_OBJECT_NAMES == 1
                        ("TSForward")
#endif
;

void manualConstruct() {
    // Manual connections
    hlRouter.set_HLPortsOut_OutputPort(0, cmdDisp.get_seqCmdBuff_InputPort(0));
    cmdDisp.set_seqCmdStatus_OutputPort(0, hlRouter.get_LLPortsIn_InputPort(0));

    // from groundRouter on SDREF
    hlRouter.set_HLPortsOut_OutputPort(12, cmdDisp.get_seqCmdBuff_InputPort(3));

    // L2 <-> L1 PrmDb
    prmDb.set_sendPrm_OutputPort(0, hlRouter.get_LLPortsIn_InputPort(10));
    hlRouter.set_HLPortsOut_OutputPort(10, prmDb.get_recvPrm_InputPort(0));
    prmDb.set_recvPrmReady_OutputPort(0, hlRouter.get_LLPortsIn_InputPort(11));
    hlRouter.set_HLPortsOut_OutputPort(11, prmDb.get_sendPrmReady_InputPort(0));

    stim300.set_IMU_OutputPort(1, hlRouter.get_LLPortsIn_InputPort(1));
    //imuInteg.set_odomNoCov_OutputPort(0, hlRouter.get_LLPortsIn_InputPort(2));
    //leeCtrl.set_accelCommand_OutputPort(0, hlRouter.get_LLPortsIn_InputPort(3));
    logQueue.set_LogSend_OutputPort(0,hlRouter.get_LLPortsIn_InputPort(4));
    tlmChan.set_PktSend_OutputPort(0,hlRouter.get_LLPortsIn_InputPort(5));
    //actuatorAdapter.set_serialDat_OutputPort(0, hlRouter.get_LLPortsIn_InputPort(6));
    tsForward.set_SendEventTime_OutputPort(0,hlRouter.get_LLPortsIn_InputPort(7));
    lidarLiteV3.set_AltimeterSend_OutputPort(0,hlRouter.get_LLPortsIn_InputPort(9));

    hlRouter.set_HLPortsOut_OutputPort(1, imuInteg.get_ImuStateUpdate_InputPort(0));
    hlRouter.set_HLPortsOut_OutputPort(2, actuatorAdapter.get_motor_InputPort(1));
    hlRouter.set_HLPortsOut_OutputPort(3, leeCtrl.get_flatOutput_InputPort(0));
    hlRouter.set_HLPortsOut_OutputPort(4, cmdDisp.get_seqCmdBuff_InputPort(2));
    hlRouter.set_HLPortsOut_OutputPort(5, leeCtrl.get_flatOutput_InputPort(0));
    hlRouter.set_HLPortsOut_OutputPort(6, leeCtrl.get_attRateThrust_InputPort(0));
    hlRouter.set_HLPortsOut_OutputPort(7, leeCtrl.get_attRateThrust_InputPort(0));

    hlRouter.set_HLPortsOut_OutputPort(8, cmdDisp.get_seqCmdBuff_InputPort(1));
    cmdDisp.set_seqCmdStatus_OutputPort(1, hlRouter.get_LLPortsIn_InputPort(8));

    hlRouter.set_HLPortsOut_OutputPort(9, actuatorAdapter.get_flySafe_InputPort(0));

    hlRouter.set_HLPortsOut_OutputPort(13, leeCtrl.get_attRateThrust_InputPort(0));
    hlRouter.set_HLPortsOut_OutputPort(14, leeCtrl.get_flatOutput_InputPort(0));
    hlRouter.set_HLPortsOut_OutputPort(15, actuatorAdapter.get_flySafe_InputPort(0));
    hlRouter.set_HLPortsOut_OutputPort(16, actuatorAdapter.get_flySafe_InputPort(0));

    // TODO(mereweth) - switch STIM vs MPU9520
    stim300.set_IMU_OutputPort(0, imuProc.get_HighRateImu_InputPort(0));
}

void constructApp() {
    // Initialize rate group driver
    rgGncDrv.init();
    // Initialize the rate groups
    rgAtt.init(1);
    rgPos.init(0);
    rgTlm.init(2);

    prmDb.init(0);

    // Initialize the GNC components
    leeCtrl.init(0);
    ctrlXest.init(0);
    imuProc.init(0);
    mixer.init(0);
    actuatorAdapter.init(0);
    sigGen.init(0);
    imuInteg.init(0);
    mpu9250.init(0);

    hlRouter.init(0);
    cmdDisp.init(0);
    tlmChan.init(0);

    a2dDrv.init(0);

    // initialize GPIO
    gpio.init(0);

    faultGpio.init(0);

    // inialize SPI drivers
    spiMaster.init(0);
    spiMaster.initDriver(0,
                         0, 64,
                         0, 512,
                         alloc);
    
    hlUart.init(HL_UART_INSTANCE);
    hlUart.initDriver(0,5000, // bytes - max we could send in one cycle
                      0,2000, // bytes - two cycles worth
                      alloc);

    debugUart.init(DEBUG_UART_INSTANCE);
    debugUart.initDriver(0,200, // bytes - max we could send in one cycle
                         0,0, // bytes - two cycles worth
                         alloc);

    stimUart.init(STIM_UART_INSTANCE);
    stimUart.initDriver(0,0,
                        0,2000,
                        alloc);

    DmaDrvInit();

    r5Time.init();

    logQueue.init(0);

    llDebug.init(0);
    llCycle.init(0);

    lidarLiteV3.init(0);

    i2c1Drv.init(0);
    i2c1Drv.initDriver(128, 128, alloc);

    eventCapture.init(0);
    eventCapture.initDriver(STIM_TOV_INSTANCE, 64, alloc);
    eventCapture.initDriver(TIME_SYNC_INSTANCE, 64, alloc);

    stim300.init(0);

    rtiWait.init(0);

    tsForward.init(0);

    // Connect rate groups to rate group driver
    constructR5REFArchitecture();

    manualConstruct();

    /* Register commands */
    cmdDisp.regCommands();
    llDebug.regCommands();
    prmDb.regCommands();

    ctrlXest.regCommands();
    imuProc.regCommands();
    leeCtrl.regCommands();
    imuInteg.regCommands();
    mixer.regCommands();
    actuatorAdapter.regCommands();
    sigGen.regCommands();

    faultGpio.setMapping(R5::GPIO_SET_BANK_A, 0);
}

void cycleForever(void) {
    while (!mpu9250.isReady() && 0) {
        Svc::InputCyclePort* port = rgGncDrv.get_CycleIn_InputPort(0);
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
        llCycle.runCycles(1);
    }
}
