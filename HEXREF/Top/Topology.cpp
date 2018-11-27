#include <Components.hpp>


#include <Fw/Types/Assert.hpp>
#include <HEXREF/Top/TargetInit.hpp>
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

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers

Svc::RateGroupDecouplerComponentImpl* rgDecouple_ptr = 0;
Svc::ActiveRateGroupImpl* rg_ptr = 0;
Svc::RateGroupDriverImpl* rgGncDrv_ptr = 0;
Svc::PassiveRateGroupImpl* rgAtt_ptr = 0;
Svc::PassiveRateGroupImpl* rgPos_ptr = 0;
Svc::ConsoleTextLoggerImpl* textLogger_ptr = 0;
Svc::ActiveLoggerImpl* eventLogger_ptr = 0;
LLProc::ShortLogQueueComponentImpl* logQueue_ptr = 0;
Svc::LinuxTimeImpl* linuxTime_ptr = 0;
SnapdragonFlight::KraitRouterComponentImpl* kraitRouter_ptr = 0;
Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr = 0;
Svc::FatalHandlerComponentImpl* fatalHandler_ptr = 0;
LLProc::LLCmdDispatcherImpl* cmdDisp_ptr = 0;
Gnc::LeeCtrlComponentImpl* leeCtrl_ptr = 0;
Gnc::BasicMixerComponentImpl* mixer_ptr = 0;
Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr = 0;
Gnc::ImuIntegComponentImpl* imuInteg_ptr = 0;
Drv::MPU9250ComponentImpl* mpu9250_ptr = 0;
Drv::LinuxSpiDriverComponentImpl* spiDrv_ptr = 0;
Drv::LinuxI2CDriverComponentImpl* i2cDrv_ptr = 0;
Drv::LinuxGpioDriverComponentImpl* imuDRInt_ptr = 0;
Drv::LinuxPwmDriverComponentImpl* escPwm_ptr = 0;

void allocComps() {
    rgDecouple_ptr = new Svc::RateGroupDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDECOUPLE",
#endif
                        5) // 50 dropped hardware cycles before an error
;

    NATIVE_UINT_TYPE rgContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = {
        0, // unused
        Gnc::IMUINTEG_SCHED_CONTEXT_TLM, // imuInteg
        Gnc::LCTRL_SCHED_CONTEXT_TLM, // leeCtrl
    };
    
    rg_ptr = new Svc::ActiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                        "RG",
#endif
                        rgContext,FW_NUM_ARRAY_ELEMENTS(rgContext));
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

    textLogger_ptr = new Svc::ConsoleTextLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("TLOG")
#endif
;

    eventLogger_ptr = new Svc::ActiveLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("ELOG")
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
    // Manual connections
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(0, cmdDisp_ptr->get_seqCmdBuff_InputPort(0));
    cmdDisp_ptr->set_seqCmdStatus_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(0));

    mpu9250_ptr->set_Imu_OutputPort(1, kraitRouter_ptr->get_HexPortsIn_InputPort(1));
    imuInteg_ptr->set_odomNoCov_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(2));
    
    logQueue_ptr->set_LogSend_OutputPort(0, kraitRouter_ptr->get_HexPortsIn_InputPort(4));

    kraitRouter_ptr->set_KraitPortsOut_OutputPort(1, imuInteg_ptr->get_ImuStateUpdate_InputPort(0));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(2, escPwm_ptr->get_pwmSetDuty_InputPort(1));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(3, actuatorAdapter_ptr->get_motor_InputPort(1));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(4, cmdDisp_ptr->get_seqCmdBuff_InputPort(1));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(5, leeCtrl_ptr->get_flatOutput_InputPort(0));
    kraitRouter_ptr->set_KraitPortsOut_OutputPort(6, leeCtrl_ptr->get_attRateThrust_InputPort(0));
}

void constructApp() {
    allocComps();

    localTargetInit();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgGncDrv_ptr->init();

    // Initialize the rate groups
    rg_ptr->init(10,0);
    rgDecouple_ptr->init(10, 0);
    rgAtt_ptr->init(1);
    rgPos_ptr->init(0);

    // Initialize the GNC components
    leeCtrl_ptr->init(0);
    mixer_ptr->init(0);
    actuatorAdapter_ptr->init(0);
    imuInteg_ptr->init(0);
    mpu9250_ptr->init(0);

    spiDrv_ptr->init(0);
    i2cDrv_ptr->init(0);
    imuDRInt_ptr->init(0);
    escPwm_ptr->init(0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger_ptr->init();
#endif

    eventLogger_ptr->init(10, 0);
    logQueue_ptr->init(0);

    linuxTime_ptr->init(0);

    fatalAdapter_ptr->init(0);
    fatalHandler_ptr->init(0);

    cmdDisp_ptr->init(0);

    kraitRouter_ptr->init(50, 1000);

    // Connect rate groups to rate group driver
    constructHEXREFArchitecture();

    manualConstruct();

    /* Register commands */
    cmdDisp_ptr->regCommands();
    eventLogger_ptr->regCommands();
    fatalHandler_ptr->regCommands();

    leeCtrl_ptr->regCommands();

    // Open devices

    Gnc::ActuatorAdapterComponentImpl::I2CMetadata meta;
    meta.minIn = 0.0f;
    meta.maxIn = 1000.0f;
    meta.minOut = 0;
    meta.maxOut = 800;
    
    meta.addr = 11;
    actuatorAdapter_ptr->setupI2C(0, meta);
    meta.addr = 12;
    actuatorAdapter_ptr->setupI2C(1, meta);
    meta.addr = 13;
    actuatorAdapter_ptr->setupI2C(2, meta);
    meta.addr = 14;
    actuatorAdapter_ptr->setupI2C(3, meta);
    
#ifdef BUILD_DSPAL
    // /dev/spi-1 on QuRT; connected to MPU9250
    spiDrv_ptr->open(1, 0, Drv::SPI_FREQUENCY_1MHZ);
    imuDRInt_ptr->open(65, Drv::LinuxGpioDriverComponentImpl::GPIO_INT);
    
    // J9, BLSP2
    i2cDrv_ptr->open(2, Drv::I2C_FREQUENCY_400KHZ);

    // J15, BLSP9
    // TODO(mereweth) - Spektrum UART and binding GPIO

    // J13 is already at 5V, so use for 4 of the ESCs
    NATIVE_UINT_TYPE pwmPins[4] = {27, 28, 29, 30};
    // /dev/pwm-1 on QuRT
    escPwm_ptr->open(1, pwmPins, 4, 20 * 1000);
#endif

    // Active component startup
    // start rate groups
    rg_ptr->start(0, 50, 2 * 1024);
    // NOTE(mereweth) - GNC att & pos loops run in this thread:
    rgDecouple_ptr->start(0, 90, 5*1024);
    // start telemetry
    eventLogger_ptr->start(0, 40, 2*1024);

#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}

void run1cycle(void) {
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rgDecouple_ptr->get_BackupCycleIn_InputPort(0);
    Svc::TimerVal cycleStart;
    cycleStart.take();
    port->invoke(cycleStart);

#if 0 // stress test, small amount of data
    Fw::ExternalSerializeBuffer bufObj;
    char buf[200] = {"hi"};
    bufObj.setExtBuffer((U8*) buf, 200);
    bufObj.setBuffLen(12);
    Fw::InputSerializePort* serPort = kraitRouter_ptr->get_HexPortsIn_InputPort(1);
    serPort->invokeSerial(bufObj);
#endif
}

void exitTasks(void) {
    rg_ptr->exit();
    rgDecouple_ptr->exit();
    eventLogger_ptr->exit();
    imuDRInt_ptr->exitThread();
    kraitRouter_ptr->exit();
}

volatile bool terminate = false;
volatile bool preinit = true;

int hexref_arm() {
    DEBUG_PRINT("hexref_arm");
    if (preinit) {
        DEBUG_PRINT("hexref_arm preinit - returning");
        return -1;
    }
    Drv::InputI2CConfigPort* confPort = i2cDrv_ptr->get_I2CConfig_InputPort(0);
    Drv::InputI2CReadWritePort* rwPort = i2cDrv_ptr->get_I2CReadWrite_InputPort(0);
    for (U32 i = 0; i < 35; i++) {
        DEBUG_PRINT("arm %u", i);
        for (U32 j = 11; j <= 14; j++) {
            confPort->invoke(400, j, 100);
            U8 readBuf[1] = { 0 };
            U8 writeBuf[1] = { 0 };
            Fw::Buffer writeObj = Fw::Buffer(0, 0, (U64) writeBuf, 1);
            Fw::Buffer readObj = Fw::Buffer(0, 0, (U64) readBuf, 1);
            rwPort->invoke(writeObj,
                           readObj);
            usleep(2500);
        }
    }
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

    Os::Task::delay(1000);
    preinit = false;

    return 0;
}

int hexref_run(void) {
    DEBUG_PRINT("hexref_run");
    if (preinit) {
        DEBUG_PRINT("hexref_run preinit - returning");
        return -1;
    }

    // TODO(mereweth) - interrupt for cycling - local_cycle as argument
    bool local_cycle = true;
    int cycle = 0;
#ifdef BUILD_DSPAL
    imuDRInt_ptr->startIntTask(99); // NOTE(mereweth) - priority unused on DSPAL
#endif

    while (!terminate) {
        //DEBUG_PRINT("Cycle %d\n",cycle);
        if (local_cycle) {
            run1cycle();
        }
        Os::Task::delay(10);
        cycle++;
    }

    // stop tasks
#ifdef BUILD_DSPAL
    imuDRInt_ptr->exitThread();
#endif
    exitTasks();
    // Give time for threads to exit
    DEBUG_PRINT("Waiting for threads...\n");
    Os::Task::delay(1000);

    DEBUG_PRINT("Exiting...\n");

    return 0;
}

int hexref_cycle(unsigned int cycles) {
    DEBUG_PRINT("hexref_cycle");
    if (preinit) {
        DEBUG_PRINT("hexref_cycle preinit - returning");
        return -1;
    }

    imuDRInt_ptr->startIntTask(99); // NOTE(mereweth) - priority unused on DSPAL
    for (unsigned int i = 0; i < cycles; i++) {
        //DEBUG_PRINT("Cycle %d of %d\n", i, cycles);
        if (terminate) return -1;
        run1cycle();
        Os::Task::delay(10);
    }
    imuDRInt_ptr->exitThread();
    DEBUG_PRINT("hexref_cycle returning");

    return 0;
}

int hexref_wait() {
    DEBUG_PRINT("hexref_wait\n");
    while (!terminate) {
        DEBUG_PRINT("hexref_wait loop; terminate: %d", terminate);
        Os::Task::delay(1000);

        // NOTE(mereweth) - test code for PWM with servos - DON'T USE WITH ESCs
        // Drv::InputPwmSetDutyCycleDataPort * port = escPwm.get_pwmSetDuty_InputPort(0);
        // static F32 d1 = 0.05;
        // static F32 d2 = 0.1;
        // F32 duty[4] = {d1, d2, d1, d2};
        // Drv::PwmSetDutyCycle config(duty, 4, 0x0f);
        // port->invoke(config);
        // d1 += 0.005;
        // if (d1 > 0.1) {  d1 = 0.05;  }
        // d2 -= 0.005;
        // if (d2 < 0.05) {  d2 = 0.1;  }

    }
    return 0;
}

int hexref_fini(void) {
    DEBUG_PRINT("hexref_fini called...\n");
    terminate = true;
    DEBUG_PRINT("hexref_fini done...\n");
    return 0;
}

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter_ptr->buffRead(port, buff, buffLen, bytes);
}

int hexref_rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter_ptr->portRead(buff, buffLen, bytes);
}

int hexref_rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return kraitRouter_ptr->buffWrite(port, buff, buffLen);
}

int hexref_rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    return kraitRouter_ptr->portWrite(buff, buffLen);
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

    hexref_cycle(10);
}

#endif //ifndef BUILD_DSPAL
