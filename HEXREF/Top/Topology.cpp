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

Svc::RateGroupDecouplerComponentImpl rgDecouple(
#if FW_OBJECT_NAMES == 1
                    "RGDECOUPLE",
#endif
                    5) // 50 dropped hardware cycles before an error
;

static NATIVE_UINT_TYPE rgContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = {
    0,
    0, // TODO(mereweth) - estimator
    Gnc::LCTRL_SCHED_CONTEXT_TLM, // leeCtrl
};
Svc::ActiveRateGroupImpl rg(
#if FW_OBJECT_NAMES == 1
                    "RG",
#endif
                    rgContext,FW_NUM_ARRAY_ELEMENTS(rgContext));
;

static NATIVE_INT_TYPE rgGncDivs[] = {10, 1, 1000};
Svc::RateGroupDriverImpl rgGncDrv(
#if FW_OBJECT_NAMES == 1
                    "RGDRV",
#endif
                    rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

static NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    Drv::MPU9250_SCHED_CONTEXT_OPERATE,
    0, //TODO(mereweth) - estimator
    Gnc::LCTRL_SCHED_CONTEXT_ATT,
};
Svc::PassiveRateGroupImpl rgAtt(
#if FW_OBJECT_NAMES == 1
                    "RGATT",
#endif
                    rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

static NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    0, //TODO(mereweth) - IMU?
    0, //TODO(mereweth) - estimator?
    Gnc::LCTRL_SCHED_CONTEXT_POS,
};
Svc::PassiveRateGroupImpl rgPos(
#if FW_OBJECT_NAMES == 1
                    "RGPOS",
#endif
                    rgPosContext,FW_NUM_ARRAY_ELEMENTS(rgPosContext));
;

#if FW_ENABLE_TEXT_LOGGING
Svc::ConsoleTextLoggerImpl textLogger
#if FW_OBJECT_NAMES == 1
                    ("TLOG")
#endif
;
#endif

Svc::ActiveLoggerImpl eventLogger
#if FW_OBJECT_NAMES == 1
                    ("ELOG")
#endif
;

Svc::LinuxTimeImpl linuxTime
#if FW_OBJECT_NAMES == 1
                    ("LTIME")
#endif
;

SnapdragonFlight::KraitRouterComponentImpl kraitRouter
#if FW_OBJECT_NAMES == 1
                    ("KRAITRTR")
#endif
;

Svc::AssertFatalAdapterComponentImpl fatalAdapter
#if FW_OBJECT_NAMES == 1
("fatalAdapter")
#endif
;

Svc::FatalHandlerComponentImpl fatalHandler
#if FW_OBJECT_NAMES == 1
("fatalHandler")
#endif
;

Gnc::LeeCtrlComponentImpl leeCtrl
#if FW_OBJECT_NAMES == 1
                    ("LEECTRL")
#endif
;

Drv::MPU9250ComponentImpl mpu9250(
#if FW_OBJECT_NAMES == 1
                    "MPU9250",
#endif
                     false) // don't use magnetometer for now
;

Drv::LinuxSpiDriverComponentImpl spiDrv
#if FW_OBJECT_NAMES == 1
                    ("SPIDRV")
#endif
;

Drv::LinuxGpioDriverComponentImpl imuDRInt
#if FW_OBJECT_NAMES == 1
                    ("IMUDRINT")
#endif
;

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
    // TODO(mereweth) - multiple DSPAL components with commands?
    //kraitRouter.set_KraitPortsOut_OutputPort(0, .get_CmdDisp_InputPort(0));
    //.set_CmdStatus_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(0);

    //kraitRouter.set_KraitPortsOut_OutputPort(0, .get_CmdDisp_InputPort(0));
    //.set_CmdStatus_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(0);

    mpu9250.set_Imu_OutputPort(1, kraitRouter.get_HexPortsIn_InputPort(1));
    //mpu9250.set_FIFORaw_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(2));
}

void constructApp() {

    localTargetInit();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgGncDrv.init();

    // Initialize the rate groups
    rg.init(10,0);
    rgDecouple.init(10, 0);
    rgAtt.init(1);
    rgPos.init(0);

    // Initialize the GNC components
    leeCtrl.init(0);
    mpu9250.init(0);

    spiDrv.init(0);
    imuDRInt.init(0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger.init();
#endif

    eventLogger.init(10, 0);

    linuxTime.init(0);

    fatalAdapter.init(0);
    fatalHandler.init(0);

    kraitRouter.init(20, 512);

    // Connect rate groups to rate group driver
    constructHEXREFArchitecture();

    manualConstruct();

    /* Register commands */
    /*eventLogger.regCommands();*/

    // Open devices
    // /dev/spi-1 on QuRT
    spiDrv.open(1, 0, Drv::SPI_FREQUENCY_1MHZ);
    imuDRInt.open(65, Drv::LinuxGpioDriverComponentImpl::GPIO_INT);

    // Active component startup
    // start rate groups
    rg.start(0, 50, 2 * 1024);
    // NOTE(mereweth) - GNC att & pos loops run in this thread:
    rgDecouple.start(0, 90, 5*1024);
    // start telemetry
    eventLogger.start(0, 40, 2*1024);

#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}

void run1cycle(void) {
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rgDecouple.get_BackupCycleIn_InputPort(0);
    Svc::TimerVal cycleStart;
    cycleStart.take();
    port->invoke(cycleStart);

#if 0 // stress test, small amount of data
    Fw::ExternalSerializeBuffer bufObj;
    char buf[200] = {"hi"};
    bufObj.setExtBuffer((U8*) buf, 200);
    bufObj.setBuffLen(12);
    Fw::InputSerializePort* serPort = kraitRouter.get_HexPortsIn_InputPort(1);
    serPort->invokeSerial(bufObj);
#endif
}

void exitTasks(void) {
    rg.exit();
    rgDecouple.exit();
    eventLogger.exit();
    imuDRInt.exitThread();
    kraitRouter.exit();
}

volatile bool terminate = false;
volatile bool preinit = true;

/* TODO(mereweth)
 * use singleton pattern to only allow one instance of the topology?
 * return error if already initialized or if terminate is already true?
 *
 * split into init and run so SDREF can wait for init to be done? init would be called in
 * Topology (first thread) and would block. Then, hexref_run would be called in thread
 */
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
    imuDRInt.startIntTask(99); // NOTE(mereweth) - priority unused on DSPAL

    while (!terminate) {
        //DEBUG_PRINT("Cycle %d\n",cycle);
        if (local_cycle) {
            run1cycle();
        }
        Os::Task::delay(10);
        cycle++;
    }

    // stop tasks
    imuDRInt.exitThread();
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

    imuDRInt.startIntTask(99); // NOTE(mereweth) - priority unused on DSPAL
    for (unsigned int i = 0; i < cycles; i++) {
        //DEBUG_PRINT("Cycle %d of %d\n", i, cycles);
        if (terminate) return -1;
        run1cycle();
        Os::Task::delay(10);
    }
    imuDRInt.exitThread();
    DEBUG_PRINT("hexref_cycle returning");

    return 0;
}

int hexref_wait() {
    DEBUG_PRINT("hexref_wait\n");
    while (!terminate) {
        DEBUG_PRINT("hexref_wait loop; terminate: %d", terminate);
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

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter.buffRead(port, buff, buffLen, bytes);
}

int hexref_rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter.portRead(buff, buffLen, bytes);
}

int hexref_rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return kraitRouter.buffWrite(port, buff, buffLen);
}

int hexref_rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    return kraitRouter.portWrite(buff, buffLen);
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

    hexref_run();
}

#endif //ifndef BUILD_DSPAL
