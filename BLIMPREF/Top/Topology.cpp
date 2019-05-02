#include <Components.hpp>


#include <Fw/Types/Assert.hpp>
#include <BLIMPREF/Top/TargetInit.hpp>
#include <Os/Task.hpp>
#include <Os/Log.hpp>
#include <Fw/Types/MallocAllocator.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#include <unistd.h>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

#define DECOUPLE_ACTUATORS

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers

Svc::RateGroupDecouplerComponentImpl* rgDecouple_ptr = 0;
Svc::QueuedDecouplerComponentImpl* passiveDataPasser_ptr = 0;
Svc::ActiveDecouplerComponentImpl* imuDecouple_ptr = 0;
Svc::ActiveDecouplerComponentImpl* actDecouple_ptr = 0;
Svc::RateGroupDriverImpl* rgDcplDrv_ptr = 0;
Svc::RateGroupDriverImpl* rgGncDrv_ptr = 0;
Svc::PassiveRateGroupImpl* rgOp_ptr = 0;
Svc::PassiveRateGroupImpl* rgTlm_ptr = 0;
Svc::PassiveRateGroupImpl* rgDev_ptr = 0;
Svc::LinuxTimeImpl* linuxTime_ptr = 0;
Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr = 0;
Svc::FatalHandlerComponentImpl* fatalHandler_ptr = 0;
Gnc::FrameTransformComponentImpl* ctrlXest_ptr = 0;
Gnc::ImuProcComponentImpl* imuProc_ptr = 0;
Gnc::FixedAxisSe3AdapterComponentImpl* axSe3Adap_ptr = 0;
Gnc::Se3CtrlComponentImpl* se3Ctrl_ptr = 0;
Gnc::WrenchMixerComponentImpl* mixer_ptr = 0;
Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr = 0;
Gnc::SigGenComponentImpl* sigGen_ptr = 0;
Gnc::AttFilterComponentImpl* attFilter_ptr = 0;
Drv::MPU9250ComponentImpl* mpu9250_ptr = 0;

SnapdragonFlight::BlspSpiDriverComponentImpl* spiDrvSnap_ptr = 0;
SnapdragonFlight::BlspI2CDriverComponentImpl* i2cDrvSnap_ptr = 0;
SnapdragonFlight::BlspGpioDriverComponentImpl* imuDRIntSnap_ptr = 0;
SnapdragonFlight::BlspGpioDriverComponentImpl* hwEnablePinSnap_ptr = 0;

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

    passiveDataPasser_ptr = new Svc::QueuedDecouplerComponentImpl(
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
        Gnc::SE3CTRL_SCHED_CONTEXT_TLM, // se3Ctrl
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

    NATIVE_INT_TYPE rgDcplDivs[] = {1, 10};

    rgDcplDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDCPLDRV",
#endif
                        rgDcplDivs,FW_NUM_ARRAY_ELEMENTS(rgDcplDivs));
 
    NATIVE_INT_TYPE rgGncDivs[] = {1, 100};

    rgGncDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDRV",
#endif
                        rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

    NATIVE_UINT_TYPE rgOpContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        0,
        Gnc::ATTFILTER_SCHED_CONTEXT_FILT, // attFilter
        Gnc::SIGGEN_SCHED_CONTEXT_OP, // sigGen
        Gnc::SE3CTRL_SCHED_CONTEXT_CTRL, // se3Ctrl
        Gnc::ACTADAP_SCHED_CONTEXT_ARM, // adapter - for arming
        0, // unused
        0, // logQueue
        0, // kraitRouter
    };

    rgOp_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGOP",
#endif
                            rgOpContext,FW_NUM_ARRAY_ELEMENTS(rgOpContext));
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
 
    axSe3Adap_ptr = new Gnc::FixedAxisSe3AdapterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("AXSE3ADAP")
#endif
;
 
    se3Ctrl_ptr = new Gnc::Se3CtrlComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SE3CTRL")
#endif
;

    mixer_ptr = new Gnc::WrenchMixerComponentImpl
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
    // Manual connections

}

void constructApp() {
    allocComps();

    localTargetInit();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgGncDrv_ptr->init();
    rgDcplDrv_ptr->init();

    // Initialize the rate groups
    rgDecouple_ptr->init(10, 0); // designed to drop if full
    passiveDataPasser_ptr->init(200, 1000); // big entries - all passive components in rgOp use this
    imuDecouple_ptr->init(100, 20); // just need to serialize cycle port
    actDecouple_ptr->init(100, 500); // big message queue entry, few entries
    rgOp_ptr->init(0);
    rgTlm_ptr->init(2);
    rgDev_ptr->init(3);

    // Initialize the GNC components
    ctrlXest_ptr->init(0);
    imuProc_ptr->init(0);
    axSe3Adap_ptr->init(0);
    se3Ctrl_ptr->init(0);
    mixer_ptr->init(0);
    actuatorAdapter_ptr->init(0);
    sigGen_ptr->init(0);
    attFilter_ptr->init(0);
    mpu9250_ptr->init(0);

    spiDrvSnap_ptr->init(0);
    i2cDrvSnap_ptr->init(0);
    hwEnablePinSnap_ptr->init(1);
    imuDRIntSnap_ptr->init(0);

    spiDrv_ptr->init(0);
    i2cDrv_ptr->init(0);
    hwEnablePin_ptr->init(1);
    imuDRInt_ptr->init(0);
    escPwm_ptr->init(0);

    linuxTime_ptr->init(0);

    fatalAdapter_ptr->init(0);
    fatalHandler_ptr->init(0);

    // Connect rate groups to rate group driver
    constructBLIMPREFArchitecture();

    manualConstruct();

    /* Register commands */
    fatalHandler_ptr->regCommands();

    ctrlXest_ptr->regCommands();
    imuProc_ptr->regCommands();
    se3Ctrl_ptr->regCommands();
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

    // TODO(mereweth) - I2C port

    // TODO(mereweth) - Spektrum UART and binding GPIO
    // TODO(mereweth) - PWM pins
#endif // SOC
#endif // BUILD_DSPAL

    // Active component startup
    imuDecouple_ptr->start(0, 91, 20*1024);
    // NOTE(mereweth) - GNC att & pos loops run in this thread:
    rgDecouple_ptr->start(0, 90, 20*1024);
    // NOTE(mereweth) - ESC I2C calls happen in this thread:
    actDecouple_ptr->start(0, 89, 20*1024);
    
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
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rgDecouple_ptr->get_BackupCycleIn_InputPort(0);
    Svc::TimerVal cycleStart;
    cycleStart.take();
    port->invoke(cycleStart);
    Os::Task::delay(10);

#ifndef BUILD_DSPAL // stress test
    for (int i = 0; i < 45; i++) {
        Fw::ExternalSerializeBuffer bufObj;
        char buf[996] = {"hi"};
        bufObj.setExtBuffer((U8*) buf, sizeof(buf));
        bufObj.setBuffLen(sizeof(buf));
        Fw::InputSerializePort* serPort = kraitRouter_ptr->get_HexPortsIn_InputPort(1);
        serPort->invokeSerial(bufObj);
    }
#endif

#ifndef BUILD_DSPAL
    U8 readBuf[4096*1000];
    int len = 0;
    hexref_rpc_relay_port_read(readBuf, sizeof(readBuf), &len);
#endif
}

void exitTasks(void) {
    rgDecouple_ptr->exit();
    imuDecouple_ptr->exit();
    actDecouple_ptr->exit();
#ifdef BUILD_DSPAL
    imuDRInt_ptr->exitThread();
#endif
    kraitRouter_ptr->exit();
}

volatile bool terminate = false;
volatile bool preinit = true;

#include <Fw/Cmd/CmdPacket.hpp>

int hexref_arm() {
    DEBUG_PRINT("hexref_arm\n");
    if (preinit) {
        DEBUG_PRINT("hexref_arm preinit - returning\n");
        return -1;
    }

    /*
    U8 buf[16] = {0};
    buf[2+4] = 0x01;
    buf[3+4] = 0xA1;

    Fw::CmdPacket cmdPkt;
    Fw::ComBuffer dat0(&buf[0], sizeof(buf));
    Fw::SerializeStatus stat = cmdPkt.deserialize(dat0);
    Fw::InputCmdPort* p2 = se3Ctrl_ptr->get_CmdDisp_InputPort(0);
    p2->invoke(0x1a1,0,cmdPkt.getArgBuffer());
    usleep(50000);
    DEBUG_PRINT("hexref_arm after sleep\n");
    */

    /*
    Fw::InputComPort* port = cmdDisp_ptr->get_seqCmdBuff_InputPort(0);
    usleep(50000);
    Fw::ComBuffer dat(buf, sizeof(buf));
    port->invoke(dat, 0);
    usleep(50000);
    DEBUG_PRINT("hexref_arm after sleep\n");
    */

    /*
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
    */

    // NOTE(mereweth) - test code for PWM with servos - DON'T USE WITH ESCs
    /*
    Drv::InputPwmSetDutyCycleDataPort * port = escPwm_ptr->get_pwmSetDuty_InputPort(0);
    static F32 d1 = 0.05;
    static F32 d2 = 0.1;
    F32 duty[4] = {d1, d2, d1, d2};
    Drv::PwmSetDutyCycle config(duty, 4, 0x0f);
    port->invoke(config);
    d1 += 0.005;
    if (d1 > 0.1) {  d1 = 0.05;  }
    d2 -= 0.005;
    if (d2 < 0.05) {  d2 = 0.1;  }
    */
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
    DEBUG_PRINT("hexref_run\n");
    if (preinit) {
        DEBUG_PRINT("hexref_run preinit - returning\n");
        return -1;
    }
    
    while (!mpu9250_ptr->isReady()) {
        Svc::InputCyclePort* port = rgDev_ptr->get_CycleIn_InputPort(0);
        Svc::TimerVal cycleStart;
        cycleStart.take();
        port->invoke(cycleStart);
        Os::Task::delay(10);
    }
    rgDecouple_ptr->setEnabled(true);
    
    int backupCycle = 0;

    while (!terminate) {
        run1backupCycle();
        backupCycle++;
    }

    // stop tasks
    rgDecouple_ptr->setEnabled(false);
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
    
    while (!mpu9250_ptr->isReady()) {
        Svc::InputCyclePort* port = rgDev_ptr->get_CycleIn_InputPort(0);
        Svc::TimerVal cycleStart;
        cycleStart.take();
        port->invoke(cycleStart);
        Os::Task::delay(10);
    }

    rgDecouple_ptr->setEnabled(true);
    for (unsigned int i = 0; i < backupCycles; i++) {
        if (terminate) break;
        run1backupCycle();
    }
    rgDecouple_ptr->setEnabled(false);
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
