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
        ACTIVE_COMP_HEXROUTER,
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

    fileUp.init(30, 0);
    fileDown.init(30, 0);
    fileUpBufMgr.init(0);
    fileDownBufMgr.init(1);

    fatalAdapter.init(0);
    fatalHandler.init(0);
    health.init(25,0);

    hexRouter.init(10, 0);

    // Connect rate groups to rate group driver
    constructSDREFArchitecture();

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
        {3,5,rg.getObjName()}, // 0
        {3,5,cmdDisp.getObjName()}, // 1
        {3,5,eventLogger.getObjName()}, // 2
        {3,5,cmdSeq.getObjName()}, // 3
        {3,5,chanTlm.getObjName()}, // 4
        {3,5,fileUp.getObjName()}, // 5
        {3,5,fileDown.getObjName()}, // 6
    };

    // register ping table
    health.setPingEntries(pingEntries,FW_NUM_ARRAY_ELEMENTS(pingEntries),0x123);

    // Active component startup
    // start rate groups
    rg.start(ACTIVE_COMP_1HZ_RG, 95, 20*1024);
    // start dispatcher
    cmdDisp.start(ACTIVE_COMP_CMD_DISP,60,20*1024);
    // start sequencer
    cmdSeq.start(ACTIVE_COMP_CMD_SEQ,50,20*1024);
    // start telemetry
    eventLogger.start(ACTIVE_COMP_LOGGER,50,20*1024);
    chanTlm.start(ACTIVE_COMP_TLM,60,20*1024);
    prmDb.start(ACTIVE_COMP_PRMDB,50,20*1024);

    hexRouter.start(ACTIVE_COMP_HEXROUTER, 90, 20*1024);//, CORE_RPC);

    hexRouter.startPortReadThread(90,20*1024, CORE_RPC);
    hexRouter.startBuffReadThread(60,20*1024, CORE_RPC);

    fileDown.start(ACTIVE_COMP_FILE_DOWNLINK, 40, 20*1024);
    fileUp.start(ACTIVE_COMP_FILE_UPLINK, 40, 20*1024);

    // Initialize socket server
    sockGndIf.startSocketTask(40, port_number, hostname);

#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}

//void run1cycle(void) {
//    // get timer to call rate group driver
//    Svc::TimerVal timer;
//    timer.take();
//    rateGroupDriverComp.get_CycleIn_InputPort(0)->invoke(timer);
//    Os::Task::TaskStatus delayStat = Os::Task::delay(1000);
//    FW_ASSERT(Os::Task::TASK_OK == delayStat,delayStat);
//}


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
    rg.exit();
    cmdDisp.exit();
    eventLogger.exit();
    chanTlm.exit();
    prmDb.exit();
    fileUp.exit();
    fileDown.exit();
    cmdSeq.exit();
    hexRouter.exit();
}

void print_usage() {
	(void) printf("Usage: ./SDREF [options]\n-p\tport_number\n-a\thostname/IP address\n-l\tFor time-based cycles\n");
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

#ifdef BUILD_SDFLIGHT
    hexref_init();
    Os::Task task;
    Os::TaskString task_name("HEXRPC");
    task.start(task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) hexref_run, NULL);
#endif //BUILD_SDFLIGHT
    
    constructApp(port_number, hostname);
    //dumparch();

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

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
     
    // stop tasks
    exitTasks();

#ifdef BUILD_SDFLIGHT
    DEBUG_PRINT("Calling exit function for SDFLIGHT\n");
    hexref_fini();
#endif //BUILD_SDFLIGHT
    
    // Give time for threads to exit
    (void) printf("Waiting for threads...\n");
    Os::Task::delay(1000);

    (void) printf("Exiting...\n");

    return 0;
}
