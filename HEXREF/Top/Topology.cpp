#include <Components.hpp>


#include <Fw/Types/Assert.hpp>
#include <Os/Task.hpp>
#include <Os/Log.hpp>
#include <Fw/Types/MallocAllocator.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#ifdef BUILD_DSPAL
#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>

//TODO(mereweth) - move to HexPower component
#include <dspal_platform.h>
#endif

#include <unistd.h>

#ifdef BUILD_DSPAL
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

//#undef DEBUG_PRINT
//#define DEBUG_PRINT(x,...)

#define DECOUPLE_RG
#define DECOUPLE_ACTUATORS

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers

Svc::RateGroupDecouplerComponentImpl* rgDecouple_ptr = 0;
Svc::QueuedDecouplerComponentImpl* imuDataPasser_ptr = 0;
Svc::ActiveDecouplerComponentImpl* imuDecouple_ptr = 0;
Svc::ActiveDecouplerComponentImpl* actDecouple_ptr = 0;
Svc::RateGroupDriverImpl* rgDcplDrv_ptr = 0;
Svc::RateGroupDriverImpl* rgGncDrv_ptr = 0;
Svc::PassiveRateGroupImpl* rgAtt_ptr = 0;
Svc::PassiveRateGroupImpl* rgPos_ptr = 0;
Svc::PassiveRateGroupImpl* rgTlm_ptr = 0;
Svc::PassiveRateGroupImpl* rgDev_ptr = 0;
Svc::ConsoleTextLoggerImpl* textLogger_ptr = 0;
LLProc::ShortLogQueueComponentImpl* logQueue_ptr = 0;
Svc::LinuxTimeImpl* linuxTime_ptr = 0;
SnapdragonFlight::KraitRouterComponentImpl* kraitRouter_ptr = 0;
Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr = 0;
Svc::FatalHandlerComponentImpl* fatalHandler_ptr = 0;
LLProc::LLCmdDispatcherImpl* cmdDisp_ptr = 0;
LLProc::LLTlmChanImpl* tlmChan_ptr = 0;
Svc::ActiveL2PrmDbComponentImpl* prmDb_ptr;
Gnc::FrameTransformComponentImpl* ctrlXest_ptr = 0;
Gnc::ImuProcComponentImpl* imuProc_ptr = 0;
Gnc::LeeCtrlComponentImpl* leeCtrl_ptr = 0;
Gnc::BasicMixerComponentImpl* mixer_ptr = 0;
Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr = 0;
Gnc::SigGenComponentImpl* sigGen_ptr = 0;
Gnc::AttFilterComponentImpl* attFilter_ptr = 0;
Drv::MPU9250ComponentImpl* mpu9250_ptr = 0;
Drv::LinuxSpiDriverComponentImpl* spiDrv_ptr = 0;
Drv::LinuxI2CDriverComponentImpl* i2cDrv_ptr = 0;
Drv::LinuxGpioDriverComponentImpl* imuDRInt_ptr = 0;
Drv::LinuxGpioDriverComponentImpl* hwEnablePin_ptr = 0;
Drv::LinuxPwmDriverComponentImpl* escPwm_ptr = 0;

void allocComps() {
    rgDecouple_ptr = new Svc::RateGroupDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDECOUPLE",
#endif
                        5) // multiply by sleep duration in run1backupCycle
;

    imuDataPasser_ptr = new Svc::QueuedDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "IMUDATPASSER"
#endif
                                                            )
;
	
    imuDecouple_ptr = new Svc::ActiveDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "IMUDECOUPLE"
#endif
                                                            )
;
    
    actDecouple_ptr = new Svc::ActiveDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "ACTDECOUPLE"
#endif
                                                            )
;

    NATIVE_UINT_TYPE rgDevContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Drv::MPU9250_SCHED_CONTEXT_OPERATE, // mpu9250
    };

    rgDev_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGDEV",
#endif
                            rgDevContext,FW_NUM_ARRAY_ELEMENTS(rgDevContext));
;
 
    NATIVE_UINT_TYPE rgTlmContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Drv::MPU9250_SCHED_CONTEXT_TLM, // mpu9250
        Gnc::ATTFILTER_SCHED_CONTEXT_TLM, // attFilter
        Gnc::SIGGEN_SCHED_CONTEXT_TLM, // sigGen
        Gnc::LCTRL_SCHED_CONTEXT_TLM, // leeCtrl
        0, // mixer
        Gnc::ACTADAP_SCHED_CONTEXT_TLM, // adapter
        0, // logQueue
        0, // chanTlm
    };

    rgTlm_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGTLM",
#endif
                            rgTlmContext,FW_NUM_ARRAY_ELEMENTS(rgTlmContext));
;

    NATIVE_INT_TYPE rgDcplDivs[] = {1, 2};

    rgDcplDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDCPLDRV",
#endif
                        rgDcplDivs,FW_NUM_ARRAY_ELEMENTS(rgDcplDivs));
 
    NATIVE_INT_TYPE rgGncDivs[] = {10, 1, 500};

    rgGncDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDRV",
#endif
                        rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

    NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        0, // imuDataPasser
        Gnc::ATTFILTER_SCHED_CONTEXT_FILT, // attFilter
        Gnc::SIGGEN_SCHED_CONTEXT_OP, // sigGen
        Gnc::LCTRL_SCHED_CONTEXT_ATT, // leeCtrl
        0,
        0,
        0, // logQueue
        0, // kraitRouter
    };

    rgAtt_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGATT",
#endif
                            rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

    NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Gnc::ACTADAP_SCHED_CONTEXT_ARM, // adapter - for arming
        Gnc::LCTRL_SCHED_CONTEXT_POS, // leeCtrl
        0,
        0,
        0,
        0,
        0, // logQueue
        0, // kraitRouter
    };

    rgPos_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                    "RGPOS",
#endif
                    rgPosContext,FW_NUM_ARRAY_ELEMENTS(rgPosContext));
;

    textLogger_ptr = new Svc::ConsoleTextLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("TLOG")
#endif
;

    logQueue_ptr = new LLProc::ShortLogQueueComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SLOG")
#endif
;

    linuxTime_ptr = new Svc::LinuxTimeImpl
#if FW_OBJECT_NAMES == 1
                        ("LTIME")
#endif
;

    kraitRouter_ptr = new SnapdragonFlight::KraitRouterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("KRAITRTR")
#endif
;

    fatalAdapter_ptr = new Svc::AssertFatalAdapterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("fatalAdapter")
#endif
;

    fatalHandler_ptr = new Svc::FatalHandlerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("fatalHandler")
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

    prmDb_ptr = new Svc::ActiveL2PrmDbComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("PRMDB", 1024) //1024 max receive
#endif
;

    ctrlXest_ptr = new Gnc::FrameTransformComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("CTRLXEST")
#endif
;
 
    imuProc_ptr = new Gnc::ImuProcComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("IMUPROC")
#endif
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

    attFilter_ptr = new Gnc::AttFilterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ATTFILTER")
#endif
;

    mpu9250_ptr = new Drv::MPU9250ComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "MPU9250",
#endif
                         false) // don't use magnetometer for now
;

    spiDrv_ptr = new Drv::LinuxSpiDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SPIDRV")
#endif
;

    i2cDrv_ptr = new Drv::LinuxI2CDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                    ("I2CDRV")
#endif
;

    imuDRInt_ptr = new Drv::LinuxGpioDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("IMUDRINT")
#endif
;

    hwEnablePin_ptr = new Drv::LinuxGpioDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("HWENPIN")
#endif
;

    escPwm_ptr = new Drv::LinuxPwmDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ESCPWM")
#endif
;

}

#if FW_OBJECT_REGISTRATION == 1

void dumparch(void) {
    simpleReg.dump();
}

#if FW_OBJECT_NAMES == 1
void dumpobj(const char* objName) {
    simpleReg.dump(objName);
}
#endif

#endif

void manualConstruct(void) {
#ifdef SOC_8074
    actuatorAdapter_ptr->set_outputEnable_OutputPort(0, hwEnablePin_ptr->get_gpioRead_InputPort(0));
#else // SOC_8074
    // TODO(mereweth) - connect when we add enable pin
#endif
  
#ifdef DECOUPLE_RG
    rgDcplDrv_ptr->set_CycleOut_OutputPort(1, rgDecouple_ptr->get_CycleIn_InputPort(0));
    rgDecouple_ptr->set_CycleOut_OutputPort(0, rgGncDrv_ptr->get_CycleIn_InputPort(0));
#else
    rgDcplDrv_ptr->set_CycleOut_OutputPort(1, rgGncDrv_ptr->get_CycleIn_InputPort(0));
#endif
  
    // Manual connections
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(0, cmdDisp_ptr->get_seqCmdBuff_InputPort(0));
    cmdDisp_ptr->set_seqCmdStatus_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(0));

    // L2 <-> L1 PrmDb
    prmDb_ptr->set_sendPrm_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(10));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(10, prmDb_ptr->get_recvPrm_InputPort(0));
    prmDb_ptr->set_recvPrmReady_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(11));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(11, prmDb_ptr->get_sendPrmReady_InputPort(0));

    imuProc_ptr->set_BatchImu_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(1));
    attFilter_ptr->set_odomNoCov_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(2));
    leeCtrl_ptr->set_accelCommand_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(3));
    logQueue_ptr->set_LogSend_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(4));
    tlmChan_ptr->set_PktSend_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(5));
    // TODO(mereweth) - too much data?
    //actuatorAdapter_ptr->set_serialDat_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(6));

    // NOTE(mereweth) - port 7 used for timesync with LL proc, so skip that HexPortsIn index

    kraitRouter_ptr->set_KraitPortsOut_OutputPort(1, attFilter_ptr->get_ImuStateUpdate_InputPort(0));
#ifdef DECOUPLE_ACTUATORS
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(9, actDecouple_ptr->get_DataIn_InputPort(2));
    actDecouple_ptr->set_DataOut_OutputPort(2, actuatorAdapter_ptr->get_flySafe_InputPort(0));
    
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(2, actDecouple_ptr->get_DataIn_InputPort(3));
    actDecouple_ptr->set_DataOut_OutputPort(3, actuatorAdapter_ptr->get_motor_InputPort(1));
#else
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(9, actuatorAdapter_ptr->get_flySafe_InputPort(0));    
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(2, actuatorAdapter_ptr->get_motor_InputPort(1));
#endif
    // aux actuator command
    //kraitRouter_ptr->set_KraitPortsOut_OutputPort(3, );
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(4, cmdDisp_ptr->get_seqCmdBuff_InputPort(2));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(5, leeCtrl_ptr->get_flatOutput_InputPort(0));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(6, leeCtrl_ptr->get_attRateThrust_InputPort(0));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(7, leeCtrl_ptr->get_attRateThrust_InputPort(0));

    kraitRouter_ptr->set_KraitPortsOut_OutputPort(8, cmdDisp_ptr->get_seqCmdBuff_InputPort(1));
    cmdDisp_ptr->set_seqCmdStatus_OutputPort(1, kraitRouter_ptr->get_HexPortsIn_InputPort(8));

    // other serial ports
    rgDcplDrv_ptr->set_CycleOut_OutputPort(0, imuDecouple_ptr->get_DataIn_InputPort(0));
    imuDecouple_ptr->set_DataOut_OutputPort(0, rgDev_ptr->get_CycleIn_InputPort(0));

    imuProc_ptr->set_DownsampledImu_OutputPort(0, imuDataPasser_ptr->get_DataIn_InputPort(0));
    imuDataPasser_ptr->set_DataOut_OutputPort(0, attFilter_ptr->get_Imu_InputPort(0));
    
#ifdef DECOUPLE_ACTUATORS
    mixer_ptr->set_motor_OutputPort(0, actDecouple_ptr->get_DataIn_InputPort(0));
    actDecouple_ptr->set_DataOut_OutputPort(0, actuatorAdapter_ptr->get_motor_InputPort(0));
        
    sigGen_ptr->set_motor_OutputPort(0, actDecouple_ptr->get_DataIn_InputPort(1));
    actDecouple_ptr->set_DataOut_OutputPort(1, actuatorAdapter_ptr->get_motor_InputPort(0));

    rgPos_ptr->set_RateGroupMemberOut_OutputPort(0, actDecouple_ptr->get_DataIn_InputPort(4));
    actDecouple_ptr->set_DataOut_OutputPort(4, actuatorAdapter_ptr->get_sched_InputPort(0));
#else
    mixer_ptr->set_motor_OutputPort(0, actuatorAdapter_ptr->get_motor_InputPort(0));
        
    sigGen_ptr->set_motor_OutputPort(0, actuatorAdapter_ptr->get_motor_InputPort(0));

    rgPos_ptr->set_RateGroupMemberOut_OutputPort(0, actuatorAdapter_ptr->get_sched_InputPort(0));
#endif
}

void constructApp() {
    allocComps();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgGncDrv_ptr->init();
    rgDcplDrv_ptr->init();

    // Initialize the rate groups
    rgDecouple_ptr->init(10, 0); // designed to drop if full
    imuDataPasser_ptr->init(100, 400); // big entries for IMU
    imuDecouple_ptr->init(100, 20); // just need to serialize cycle port
    actDecouple_ptr->init(100, 500); // big message queue entry, few entries
    rgAtt_ptr->init(1);
    rgPos_ptr->init(0);
    rgTlm_ptr->init(2);
    rgDev_ptr->init(3);

    // Initialize the GNC components
    ctrlXest_ptr->init(0);
    imuProc_ptr->init(0);
    leeCtrl_ptr->init(0);
    mixer_ptr->init(0);
    actuatorAdapter_ptr->init(0);
    sigGen_ptr->init(0);
    attFilter_ptr->init(0);
    mpu9250_ptr->init(0);

    //mpu9250_ptr->setOutputMode(Drv::MPU9250ComponentImpl::OUTPUT_ACCEL_4KHZ_GYRO_8KHZ_DLPF_GYRO_3600KHZ);

    mpu9250_ptr->setOutputMode(Drv::MPU9250ComponentImpl::OUTPUT_1KHZ_DLPF_ACCEL_184HZ_GYRO_184HZ);
    
    spiDrv_ptr->init(0);
    i2cDrv_ptr->init(0);
    hwEnablePin_ptr->init(1);
    imuDRInt_ptr->init(0);
    escPwm_ptr->init(0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger_ptr->init();
#endif

    logQueue_ptr->init(0);

    linuxTime_ptr->init(0);

    fatalAdapter_ptr->init(0);
    fatalHandler_ptr->init(0);

    cmdDisp_ptr->init(0);
    tlmChan_ptr->init(0);
    
    prmDb_ptr->init(100, 0);

    kraitRouter_ptr->init(200, 1600);

    // Connect rate groups to rate group driver
    constructHEXREFArchitecture();

    manualConstruct();

    /* Register commands */
    cmdDisp_ptr->regCommands();
    fatalHandler_ptr->regCommands();
    prmDb_ptr->regCommands();

    ctrlXest_ptr->regCommands();
    imuProc_ptr->regCommands();
    leeCtrl_ptr->regCommands();
    attFilter_ptr->regCommands();
    mixer_ptr->regCommands();
    actuatorAdapter_ptr->regCommands();
    sigGen_ptr->regCommands();

    // Open devices

#ifdef BUILD_DSPAL
#ifdef SOC_8074
    // /dev/spi-1 on QuRT; connected to MPU9250
    spiDrv_ptr->open(1, 0, Drv::SPI_FREQUENCY_1MHZ);
    imuDRInt_ptr->open(65, Drv::LinuxGpioDriverComponentImpl::GPIO_INT);
    
    // J13-3, 5V level
    hwEnablePin_ptr->open(28, Drv::LinuxGpioDriverComponentImpl::GPIO_IN);

    // J15, BLSP9
    i2cDrv_ptr->open(9, Drv::I2C_FREQUENCY_400KHZ);

    // J15, BLSP9
    // TODO(mereweth) - Spektrum UART and binding GPIO

    // J13 is already at 5V, so use for 4 of the ESCs
    //NATIVE_UINT_TYPE pwmPins[4] = {27, 28, 29, 30};
    // /dev/pwm-1 on QuRT
    //escPwm_ptr->open(1, pwmPins, 4, 20 * 1000);
#else 
    // /dev/spi-10 on 820; connected to MPU9250
    spiDrv_ptr->open(10, 0, Drv::SPI_FREQUENCY_1MHZ);
    imuDRInt_ptr->open(78, Drv::LinuxGpioDriverComponentImpl::GPIO_INT);
    
    // TODO(mereweth) - hardware enable pin

    // J10, SSC I2C 2, sonar pinout
    i2cDrv_ptr->open(2, Drv::I2C_FREQUENCY_400KHZ);

    // TODO(mereweth) - Spektrum UART and binding GPIO
    // TODO(mereweth) - PWM pins
#endif // SOC
#endif // BUILD_DSPAL

    // Active component startup
    imuDecouple_ptr->start(0, 91, 20*1024);
#ifdef DECOUPLE_RG
    // NOTE(mereweth) - GNC att & pos loops run in this thread on 801:
    rgDecouple_ptr->start(0, 90, 20*1024);
#endif
#ifdef DECOUPLE_ACTUATORS
    // NOTE(mereweth) - ESC I2C calls happen in this thread:
    actDecouple_ptr->start(0, 89, 20*1024);
#endif

    prmDb_ptr->start(0, 50, 20*1024);

#ifdef BUILD_DSPAL
    imuDRInt_ptr->startIntTask(99); // NOTE(mereweth) - priority unused on DSPAL
#endif
    
#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter_ptr->buffRead(port, buff, buffLen, bytes);
}

int hexref_rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
#ifndef BUILD_DSPAL
    DEBUG_PRINT("hexref_rpc_relay_port_read\n");
#endif
    return kraitRouter_ptr->portRead(buff, buffLen, bytes);
}

int hexref_rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return kraitRouter_ptr->buffWrite(port, buff, buffLen);
}

int hexref_rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    return kraitRouter_ptr->portWrite(buff, buffLen);
}

void run1backupCycle(void) {
#ifdef DECOUPLE_RG
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rgDecouple_ptr->get_BackupCycleIn_InputPort(0);
    Svc::TimerVal cycleStart;
    cycleStart.take();
    port->invoke(cycleStart);
#endif
    Os::Task::delay(100);
}

void exitTasks(void) {
    kraitRouter_ptr->quit();
#ifdef DECOUPLE_RG
    rgDecouple_ptr->exit();
    rgDecouple_ptr->join(NULL);
#endif
    imuDecouple_ptr->exit();
    imuDecouple_ptr->join(NULL);
#ifdef DECOUPLE_ACTUATORS
    actDecouple_ptr->exit();
    actDecouple_ptr->join(NULL);
#endif
#ifdef BUILD_DSPAL
    imuDRInt_ptr->exitThread();
#endif

    prmDb_ptr->exit();
}

volatile bool terminate = false;
volatile bool preinit = true;

#include <Fw/Cmd/CmdPacket.hpp>

int hexref_arm() {
    DEBUG_PRINT("hexref_arm\n");
    terminate = true;

    return 0;
}

int hexref_init(void) {
    DEBUG_PRINT("Before constructing app\n");
    constructApp();
    DEBUG_PRINT("After constructing app\n");

    //TODO(mereweth) - move to HexPower component
#ifdef BUILD_DSPAL
    HAP_power_request(100, 100, 1);
#endif // BUILD_DSPAL

    //dumparch();

    preinit = false;

    return 0;
}

void start_mpu9250() {
    int imuCycle = 0;
    while (!mpu9250_ptr->isReady()) {
        DEBUG_PRINT("starting imu cycle %d\n", imuCycle++);
        Svc::TimerVal cycleStart;
        cycleStart.take();
#if 0
        Svc::InputCyclePort* port = rgDev_ptr->get_CycleIn_InputPort(0);
        port->invoke(cycleStart);
#else
        Fw::InputSerializePort* serPort = imuDecouple_ptr->get_DataIn_InputPort(0);
        Fw::ExternalSerializeBuffer bufObj;
        char buf[18];
        bufObj.setExtBuffer((U8*) buf, sizeof(buf));
        bufObj.serialize(cycleStart);
        serPort->invokeSerial(bufObj);
#endif
        
        Os::Task::delay(10);
    }
}

int hexref_run(void) {
    DEBUG_PRINT("hexref_run\n");
    if (preinit) {
        DEBUG_PRINT("hexref_run preinit - returning\n");
        return -1;
    }
    
    start_mpu9250();
#ifdef DECOUPLE_RG
    rgDecouple_ptr->setEnabled(true);
#endif
    
    int backupCycle = 0;

    while (!terminate) {
        DEBUG_PRINT("running cycle %d\n", backupCycle++);
        run1backupCycle();
    }

    // stop tasks
#ifdef DECOUPLE_RG
    rgDecouple_ptr->setEnabled(false);
#endif
    exitTasks();
    // Give time for threads to exit
    DEBUG_PRINT("Waiting for threads...\n");
    Os::Task::delay(1000);

    DEBUG_PRINT("Exiting...\n");

    return 0;
}

int hexref_cycle(unsigned int backupCycles) {
    DEBUG_PRINT("hexref_cycle\n");
    if (preinit) {
        DEBUG_PRINT("hexref_cycle preinit - returning\n");
        return -1;
    }

    start_mpu9250();
#ifdef DECOUPLE_RG
    rgDecouple_ptr->setEnabled(true);
#endif
    for (unsigned int i = 0; i < backupCycles; i++) {
        if (terminate) break;
        run1backupCycle();
    }
#ifdef DECOUPLE_RG
    rgDecouple_ptr->setEnabled(false);
#endif
    DEBUG_PRINT("hexref_cycle returning\n");

    return 0;
}

int hexref_wait() {
    DEBUG_PRINT("hexref_wait\n");
    while (!terminate) {
        DEBUG_PRINT("hexref_wait loop; terminate: %d\n", terminate);
        Os::Task::delay(1000);
    }
    return 0;
}

int hexref_fini(void) {
    DEBUG_PRINT("hexref_fini called...\n");
    terminate = true;
    DEBUG_PRINT("hexref_fini done...\n");
    return 0;
}

#ifndef BUILD_DSPAL

#include <signal.h>
#include <stdio.h>

extern "C" {
    int main(int argc, char* argv[]);
};

static void sighandler(int signum) {
    terminate = 1;
}

int main(int argc, char* argv[]) {
    hexref_init();

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);
    signal(SIGHUP,sighandler);

    preinit=false;

    for (int i = 0; i < 1000; i++) {
        // call interrupt to emulate a clock
        Svc::InputCyclePort* port = rgDecouple_ptr->get_CycleIn_InputPort(0);
        Svc::TimerVal cycleStart;
        cycleStart.take();
        port->invoke(cycleStart);

        if (0 == i%10) {
            // call interrupt to emulate a clock
            Svc::InputCyclePort* port = rgDecouple_ptr->get_BackupCycleIn_InputPort(0);
            Svc::TimerVal cycleStart;
            cycleStart.take();
            port->invoke(cycleStart);
        }

        Os::Task::delay(1);
    }
}

#endif //ifndef BUILD_DSPAL
