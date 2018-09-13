#include <ros/ros.h>

#include <Components.hpp>

enum {
    CORE_NONE = -1,
    CORE_0 = 0,
    CORE_1 = 1,
    CORE_2 = 2,
    CORE_3 = 3,

    CORE_CDH = CORE_1,
    CORE_RPC = CORE_2
};

#include <Fw/Types/Assert.hpp>
#include <SDREF/Top/TargetInit.hpp>
#include <Os/Task.hpp>
#include <Os/Log.hpp>
#include <Fw/Types/MallocAllocator.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#ifdef BUILD_SDFLIGHT
#include <HEXREF/Rpc/hexref.h>
#endif

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

/*#ifdef BUILD_SDFLIGHT
#define PRM_PATH "/usr/share/data/adsp/PrmDb.dat"
#else*/
#define PRM_PATH "PrmDb.dat"
//#endif

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers
static NATIVE_INT_TYPE rgDivs[] = {1};
Svc::RateGroupDriverImpl rgDrv(
#if FW_OBJECT_NAMES == 1
                    "RGDRV",
#endif
                    rgDivs,FW_NUM_ARRAY_ELEMENTS(rgDivs));

static NATIVE_UINT_TYPE rgContext[] = {0,0,0,0,0,0,0,0,0,0};
Svc::ActiveRateGroupImpl rg(
#if FW_OBJECT_NAMES == 1
                    "RG",
#endif
                    rgContext,FW_NUM_ARRAY_ELEMENTS(rgContext));
;

// Command Components
Svc::SocketGndIfImpl sockGndIf
#if FW_OBJECT_NAMES == 1
                    ("SGIF")
#endif
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

Svc::TlmChanImpl chanTlm
#if FW_OBJECT_NAMES == 1
                    ("TLM")
#endif
;

Svc::CommandDispatcherImpl cmdDisp
#if FW_OBJECT_NAMES == 1
                    ("CMDDISP")
#endif
;

Fw::MallocAllocator seqMallocator;
Svc::CmdSequencerComponentImpl cmdSeq
#if FW_OBJECT_NAMES == 1
                    ("CMDSEQ")
#endif
;

Svc::PrmDbImpl prmDb
#if FW_OBJECT_NAMES == 1
                    ("PRM",PRM_PATH)
#else
                    (PRM_PATH)
#endif
;

SnapdragonFlight::HexRouterComponentImpl hexRouter
#if FW_OBJECT_NAMES == 1
                    ("HEXRTR")
#endif
;

SDREF::SDRosIfaceComponentImpl sdRosIface
#if FW_OBJECT_NAMES == 1
                    ("SDROSIFACE")
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

void constructApp(int port_number, char* hostname) {

    localTargetInit();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgDrv.init();

    // Initialize the rate groups
    rg.init(10,0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger.init();
#endif

    eventLogger.init(10,0);

    linuxTime.init(0);

    chanTlm.init(10,0);

    cmdDisp.init(20,0);

    cmdSeq.init(10,0);
    cmdSeq.allocateBuffer(0,seqMallocator,5*1024);

    prmDb.init(10,0);

    sockGndIf.init(0);

    fatalAdapter.init(0);
    fatalHandler.init(0);

    hexRouter.init(10, 0);
    sdRosIface.init(10);

    // Connect rate groups to rate group driver
    constructSDREFArchitecture();

    // Manual connections

    //hexRouter.set_HexPortsOut_OutputPort(1, prmDb.get_

    // Commanding - use last port to allow MagicDraw plug-in to autocount the other components
    cmdDisp.set_compCmdSend_OutputPort(Svc::CommandDispatcherImpl::NUM_CMD_PORTS-1,hexRouter.get_KraitPortsIn_InputPort(0));
    hexRouter.set_HexPortsOut_OutputPort(0, cmdDisp.get_compCmdStat_InputPort(0));

    hexRouter.set_HexPortsOut_OutputPort(1, sdRosIface.get_Imu_InputPort(0));
    hexRouter.set_HexPortsOut_OutputPort(2, sdRosIface.get_Odometry_InputPort(0));

    sdRosIface.set_ImuStateUpdate_OutputPort(0, hexRouter.get_KraitPortsIn_InputPort(1));
    for (int i = 0; i < 6; i++) {
        sdRosIface.set_Float32Data_OutputPort(i, hexRouter.get_KraitPortsIn_InputPort(2 + i));
    }

    // Proxy registration
    // TODO(mereweth) - multiple DSPAL components with commands?
    //hexCmdProxy.set_CmdReg_OutputPort(0,cmdDisp.get_compCmdReg_InputPort(Svc::CommandDispatcherImpl::NUM_CMD_PORTS-1));
    //hexCmdProxy.regCommands();

    /* Register commands */
    cmdSeq.regCommands();
    cmdDisp.regCommands();
    eventLogger.regCommands();
    prmDb.regCommands();

    // read parameters
    prmDb.readParamFile();

    // Active component startup
    // start rate groups
    rg.start(0, 95, 20*1024);
    // start dispatcher
    cmdDisp.start(0,60,20*1024);
    // start sequencer
    cmdSeq.start(0,50,20*1024);
    // start telemetry
    eventLogger.start(0,50,20*1024);
    chanTlm.start(0,60,20*1024);
    prmDb.start(0,50,20*1024);

    hexRouter.start(0, 90, 20*1024);//, CORE_RPC);

    hexRouter.startPortReadThread(90,20*1024, CORE_RPC);
    //hexRouter.startBuffReadThread(60,20*1024, CORE_RPC);

    // Initialize socket server
    sockGndIf.startSocketTask(40, port_number, hostname);

    sdRosIface.startPub();

#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}


void run1cycle(void) {
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rgDrv.get_CycleIn_InputPort(0);
    Svc::TimerVal cycleStart;
    cycleStart.take();
    port->invoke(cycleStart);
    Os::Task::delay(1000);
}

void runcycles(NATIVE_INT_TYPE cycles) {
    if (cycles == -1) {
        while (true) {
            run1cycle();
        }
    }

    for (NATIVE_INT_TYPE cycle = 0; cycle < cycles; cycle++) {
        run1cycle();
    }
}

void exitTasks(void) {
    hexRouter.quitReadThreads();
    DEBUG_PRINT("After HexRouter read thread quit\n");
    rg.exit();
    cmdDisp.exit();
    eventLogger.exit();
    chanTlm.exit();
    prmDb.exit();
    cmdSeq.exit();
    hexRouter.exit();
    DEBUG_PRINT("After HexRouter quit\n");
}

void print_usage() {
    (void) printf("Usage: ./SDREF [options]\n-p\tport_number\n-a\thostname/IP address\n-l\tFor time-based cycles\n-i\tto disable init\n-f\tto disable fini\n-o to run # cycles instead of continuously\n");
}


#include <signal.h>
#include <stdio.h>

extern "C" {
    int main(int argc, char* argv[]);
};

volatile sig_atomic_t terminate = 0;

static void sighandler(int signum) {
    terminate = 1;
    ros::shutdown();
}

void dummy() {
    while(!terminate) {
        Os::Task::delay(1000);
    }
}

int main(int argc, char* argv[]) {
    bool noInit = false;
    bool noFini = false;
    bool kraitCycle = false;
    bool hexCycle = true;
    int numKraitCycles = 0;
    U32 port_number = 0;
    I32 option = 0;
    char *hostname = NULL;
    bool local_cycle = false;

    // Removes ROS cmdline args as a side-effect
    ros::init(argc,argv,"SDREF", ros::init_options::NoSigintHandler);

    while ((option = getopt(argc, argv, "ifhlp:a:o:")) != -1){
        switch(option) {
            case 'h':
                print_usage();
                return 0;
                break;
            case 'l':
              local_cycle = true;
              break;
            case 'p':
                port_number = atoi(optarg);
                break;
            case 'a':
                hostname = optarg;
                break;
            case 'i':
                noInit = true;
                break;
            case 'f':
                noFini = true;
                break;
            case 'o':
                numKraitCycles = atoi(optarg);
                kraitCycle = true;
                hexCycle = false;
                break;
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

    if (kraitCycle && hexCycle) {
        printf("o and c both specified - use one only\n");
        return 1;
    }

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);
    signal(SIGKILL,sighandler);

    (void) printf("Hit Ctrl-C to quit\n");

    constructApp(port_number, hostname);
    //dumparch();

    ros::start();

    Os::Task task;
    Os::Task waiter;
    Os::TaskString waiter_task_name("WAITER");
#ifdef BUILD_SDFLIGHT
    // TODO(mereweth) - test that calling other functions before init has no effect
    //hexref_rpc_relay_buff_allocate(10);
    if (!noInit) {
        hexref_init();
    }
    if (hexCycle) {
        Os::TaskString task_name("HEXRPC");
        DEBUG_PRINT("Starting cycler on hexagon\n");
        task.start(task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) hexref_run, NULL);
    }
    waiter.start(waiter_task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) hexref_wait, NULL);
#else
    if (hexCycle) {
        Os::TaskString task_name("DUMMY");
        task.start(task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) dummy, NULL);
    }
    waiter.start(waiter_task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) dummy, NULL);
#endif //BUILD_SDFLIGHT

#ifdef BUILD_SDFLIGHT
    if (kraitCycle) {
        DEBUG_PRINT("Cycling from Krait\n");
        hexref_cycle(numKraitCycles);
    }
#endif //BUILD_SDFLIGHT

    int cycle = 0;

    while (!terminate) {
      DEBUG_PRINT("Cycle %d\n",cycle);
      if (local_cycle) {
        runcycles(1);
      } else {
        Os::Task::delay(1000);
      }
      cycle++;
    }

#ifdef BUILD_SDFLIGHT
    if (!noFini) {
        DEBUG_PRINT("Calling exit function for SDFLIGHT\n");
        hexref_fini();
    }
#endif //BUILD_SDFLIGHT

    // stop tasks
    DEBUG_PRINT("Stopping tasks\n");
    ros::shutdown();
    exitTasks();

    if (hexCycle) {
        DEBUG_PRINT("Waiting for the runner to return\n");
        FW_ASSERT(task.join(NULL) == Os::Task::TASK_OK);
    }

    DEBUG_PRINT("Waiting for the Hexagon code to be unloaded - prevents hanging the board\n");
    FW_ASSERT(waiter.join(NULL) == Os::Task::TASK_OK);

    // Give time for threads to exit
    (void) printf("Waiting for threads...\n");
    Os::Task::delay(1000);

    (void) printf("Exiting...\n");

    return 0;
}
