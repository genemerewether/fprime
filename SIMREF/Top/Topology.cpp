#include <ros/ros.h>

#include <Components.hpp>

#include <Fw/Types/Assert.hpp>
#include <SIMREF/Top/TargetInit.hpp>
#include <Os/Task.hpp>
#include <Os/Log.hpp>
#include <Fw/Types/MallocAllocator.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

#define PRM_PATH "PrmDb.dat"

enum {
    DOWNLINK_PACKET_SIZE = 500,
    DOWNLINK_BUFFER_STORE_SIZE = 2500,
    DOWNLINK_BUFFER_QUEUE_SIZE = 5,
    UPLINK_BUFFER_STORE_SIZE = 3000,
    UPLINK_BUFFER_QUEUE_SIZE = 30
};

// List of context IDs
enum {
    ACTIVE_COMP_1HZ_RG,
    ACTIVE_COMP_GNC_RG,
    ACTIVE_COMP_CMD_DISP,
    ACTIVE_COMP_CMD_SEQ,
    ACTIVE_COMP_LOGGER,
    ACTIVE_COMP_TLM,
    ACTIVE_COMP_PRMDB,
    ACTIVE_COMP_FILE_DOWNLINK,
    ACTIVE_COMP_FILE_UPLINK,

    CYCLER_TASK,
    NUM_ACTIVE_COMPS
};

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers

static NATIVE_UINT_TYPE rgContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = {0};
Svc::ActiveRateGroupImpl rg(
#if FW_OBJECT_NAMES == 1
                    "RG",
#endif
                    rgContext,FW_NUM_ARRAY_ELEMENTS(rgContext));
;

static NATIVE_UINT_TYPE rgGncContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = {0};
Svc::ActiveRateGroupImpl rgGnc(
#if FW_OBJECT_NAMES == 1
                    "RGGNC",
#endif
                    rgGncContext,FW_NUM_ARRAY_ELEMENTS(rgGncContext));
;

static NATIVE_INT_TYPE rgGncDivs[] = {1, 10, 1000};
Svc::RateGroupDriverImpl rgGncDrv(
#if FW_OBJECT_NAMES == 1
                    "RGGNCDRV",
#endif
                    rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

//NOTE(mereweth) - change this in sync with RosCycle timeDivMS

// This table needs to be updated whenever
// the connections in the topology are
// updated.

static NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {0};
Svc::PassiveRateGroupImpl rgAtt(
#if FW_OBJECT_NAMES == 1
                    "RGATT",
#endif
                    rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

static NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {0};
Svc::PassiveRateGroupImpl rgPos(
#if FW_OBJECT_NAMES == 1
                    "RGPOS",
#endif
                    rgPosContext,FW_NUM_ARRAY_ELEMENTS(rgPosContext));
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

ROS::RosCycleComponentImpl rosCycle
#if FW_OBJECT_NAMES == 1
                    ("ROSCYCLE",
#else
                    (
#endif
                    1)
;

SIMREF::RotorSDrvComponentImpl rotorSDrv
#if FW_OBJECT_NAMES == 1
                    ("ROTORSDRV")
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

Svc::FileUplink fileUp ("fileUp");
Svc::FileDownlink fileDown ("fileDown", DOWNLINK_PACKET_SIZE);
Svc::BufferManager fileDownBufMgr("fileDownBufMgr", DOWNLINK_BUFFER_STORE_SIZE, DOWNLINK_BUFFER_QUEUE_SIZE);
Svc::BufferManager fileUpBufMgr("fileUpBufMgr", UPLINK_BUFFER_STORE_SIZE, UPLINK_BUFFER_QUEUE_SIZE);
Svc::HealthImpl health("health");

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
    rgGncDrv.init();

    // Initialize the rate groups
    rg.init(10,0);
    rgGnc.init(10,0);
    rgAtt.init(0);
    rgPos.init(0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger.init();
#endif

    eventLogger.init(10,0);

    rosCycle.init(0);

    rotorSDrv.init(0);

    linuxTime.init(0);

    chanTlm.init(10,0);

    cmdDisp.init(20,0);

    cmdSeq.init(10,0);
    cmdSeq.allocateBuffer(0,seqMallocator,5*1024);

    prmDb.init(10,0);

    sockGndIf.init(0);

    fileUp.init(30, 0);
    fileDown.init(30, 0);
    fileUpBufMgr.init(0);
    fileDownBufMgr.init(1);

    fatalAdapter.init(0);
    fatalHandler.init(0);
    health.init(25,0);

    // Connect rate groups to rate group driver
    constructSIMREFArchitecture();

    // Manual connections

    /* Register commands */
    cmdSeq.regCommands();
    cmdDisp.regCommands();
    eventLogger.regCommands();
    prmDb.regCommands();
    fileDown.regCommands();
    health.regCommands();

    // read parameters
    prmDb.readParamFile();

    // set health ping entries

    Svc::HealthImpl::PingEntry pingEntries[] = {
        {5,10,chanTlm.getObjName()}, // 0
        {5,10,cmdDisp.getObjName()}, // 1
        {5,10,cmdSeq.getObjName()}, // 2
        {5,10,eventLogger.getObjName()}, // 3
        {5,10,fileDown.getObjName()}, // 4
        {5,10,fileUp.getObjName()}, // 5
        {5,10,prmDb.getObjName()}, // 6

        {5,10,rgGnc.getObjName()}, // 7
        {5,10,rg.getObjName()}, // 8
        {5,10,rosCycle.getObjName()}, // 9
    };

    // Active component startup
    // start rate groups
    rg.start(ACTIVE_COMP_1HZ_RG, 50, 20*1024);
    rgGnc.start(ACTIVE_COMP_GNC_RG, 90, 20*1024);
    // start dispatcher
    cmdDisp.start(ACTIVE_COMP_CMD_DISP,60,20*1024);
    // start sequencer
    cmdSeq.start(ACTIVE_COMP_CMD_SEQ,50,20*1024);
    // start telemetry
    eventLogger.start(ACTIVE_COMP_LOGGER,50,20*1024);
    chanTlm.start(ACTIVE_COMP_TLM,60,20*1024);
    prmDb.start(ACTIVE_COMP_PRMDB,50,20*1024);

    fileDown.start(ACTIVE_COMP_FILE_DOWNLINK, 40, 20*1024);
    fileUp.start(ACTIVE_COMP_FILE_UPLINK, 40, 20*1024);

    // Initialize socket server
    sockGndIf.startSocketTask(40, port_number, hostname);

    Os::Task::TaskStatus stat = rosCycle.startIntTask(90, 20*1024);
    FW_ASSERT(Os::Task::TASK_OK == stat, stat);

    rotorSDrv.startPub();
    stat = rotorSDrv.startIntTask(70, 20*1024);
    FW_ASSERT(Os::Task::TASK_OK == stat, stat);

    // register ping table
    //health.setPingEntries(pingEntries,FW_NUM_ARRAY_ELEMENTS(pingEntries),0x123);

#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}

void run1cycle(void) {
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rg.get_CycleIn_InputPort(0);
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
    rg.exit();
    rgGnc.exit();
    cmdDisp.exit();
    eventLogger.exit();
    chanTlm.exit();
    prmDb.exit();
    fileUp.exit();
    fileDown.exit();
    cmdSeq.exit();
}

void print_usage() {
    (void) printf("Usage: ./SIMREF [options]\n-p\tport_number\n-a\thostname/IP address\n-l\tFor time-based cycles\n");
}


#include <signal.h>
#include <stdio.h>

extern "C" {
    int main(int argc, char* argv[]);
};

volatile sig_atomic_t terminate = 0;

static void sighandler(int signum) {
    terminate = 1;
}

int main(int argc, char* argv[]) {
    U32 port_number = 0;
    I32 option = 0;
    char *hostname = NULL;
    bool local_cycle = false;

    // Removes ROS cmdline args as a side-effect
    ros::init(argc,argv,"SIMREF", ros::init_options::NoSigintHandler);

    while ((option = getopt(argc, argv, "hlp:a:")) != -1){
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
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

    (void) printf("Hit Ctrl-C to quit\n");

    ros::start();

    constructApp(port_number, hostname);

    if (!local_cycle) {
        rgGncDrv.set_CycleOut_OutputPort(2, rg.get_CycleIn_InputPort(0));
    }
    //dumparch();

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

    int cycle = 0;

    while (!terminate) {
      //DEBUG_PRINT("Cycle %d\n",cycle);
      if (local_cycle) {
        runcycles(1);
      } else {
        Os::Task::delay(1000);
      }
      cycle++;
    }

    // stop tasks
    exitTasks();

    // Give time for threads to exit
    (void) printf("Waiting for threads...\n");
    Os::Task::delay(1000);

    (void) printf("Exiting...\n");

    ros::shutdown();

    return 0;
}
