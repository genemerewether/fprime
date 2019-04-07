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

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers

static NATIVE_UINT_TYPE rgContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = {
    // TODO(mereweth) - add sched contexts here - keep in sync with MD model
    0, // cmdSeq
    0, // chanTlm
    0, // rosCycle
    SIMREF::RSDRV_SCHED_CONTEXT_TLM, // rotorSDrv
    Gnc::ATTFILTER_SCHED_CONTEXT_TLM, // imuInteg
    Gnc::SIGGEN_SCHED_CONTEXT_TLM, // sigGen
    Gnc::LCTRL_SCHED_CONTEXT_TLM, // leeCtrl
    0, // mixer
    0 // activeFileLogger
};
Svc::ActiveRateGroupImpl rg(
#if FW_OBJECT_NAMES == 1
                    "RG",
#endif
                    rgContext,FW_NUM_ARRAY_ELEMENTS(rgContext));
;

Svc::RateGroupDecouplerComponentImpl rgDecouple
#if FW_OBJECT_NAMES == 1
                    ("RGDECOUPLE",
                     100) // 100 dropped cycles before error
#endif
;

//NOTE(mereweth) - change this in sync with RosCycle timeDivMS
static NATIVE_INT_TYPE rgGncDivs[] = {10, 1, 100};
Svc::RateGroupDriverImpl rgGncDrv(
#if FW_OBJECT_NAMES == 1
                    "RGGNCDRV",
#endif
                    rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

static NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    // TODO(mereweth) - add sched contexts here - keep in sync with MD model
    SIMREF::RSDRV_SCHED_CONTEXT_ATT,
    Gnc::ATTFILTER_SCHED_CONTEXT_ATT,
    Gnc::SIGGEN_SCHED_CONTEXT_ATT,
    Gnc::LCTRL_SCHED_CONTEXT_ATT,
};
Svc::PassiveRateGroupImpl rgAtt(
#if FW_OBJECT_NAMES == 1
                    "RGATT",
#endif
                    rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

static NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    // TODO(mereweth) - add sched contexts here - keep in sync with MD model
    SIMREF::RSDRV_SCHED_CONTEXT_POS,
    Gnc::ATTFILTER_SCHED_CONTEXT_POS,
    Gnc::SIGGEN_SCHED_CONTEXT_POS,
    Gnc::LCTRL_SCHED_CONTEXT_POS,
};
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

Svc::UdpReceiverComponentImpl udpReceiver
#if FW_OBJECT_NAMES == 1
                        ("UDPRECV")
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

Svc::ActiveFileLoggerImpl fileLogger
#if FW_OBJECT_NAMES == 1
                    ("FLOG")
#endif
;

ROS::RosTimeImpl rosTime
#if FW_OBJECT_NAMES == 1
                    ("ROSTIME")
#endif
;

ROS::RosCycleComponentImpl rosCycle
#if FW_OBJECT_NAMES == 1
                    ("ROSCYCLE",
#else
                    (
#endif
                    10) // 10 milliseconds per division
;

SIMREF::RotorSDrvComponentImpl rotorSDrv
#if FW_OBJECT_NAMES == 1
                    ("ROTORSDRV")
#endif
;

SIMREF::GazeboManipIfComponentImpl gzManipIf
#if FW_OBJECT_NAMES == 1
                    ("GZMANIPIF")
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

Gnc::AttFilterComponentImpl attFilter
#if FW_OBJECT_NAMES == 1
                    ("ATTFILTER")
#endif
;

Gnc::SigGenComponentImpl sigGen
#if FW_OBJECT_NAMES == 1
                    ("SIGGEN")
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

void constructApp(int port_number, char* udp_string, char* hostname) {

    localTargetInit();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgGncDrv.init();

    // Initialize the rate groups
    rg.init(10,0);
    rgDecouple.init(10,0);
    rgAtt.init(0);
    rgPos.init(0);

    // Initialize the GNC components
    leeCtrl.init(0);
    mixer.init(0);
    attFilter.init(0);
    sigGen.init(0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger.init();
#endif

    eventLogger.init(10,0);
    fileLogger.init(10);

    rosCycle.init(0);

    rotorSDrv.init(0);
    gzManipIf.init(0);

    linuxTime.init(0);

    chanTlm.init(10,0);

    cmdDisp.init(20,0);

    cmdSeq.init(10,0);
    cmdSeq.allocateBuffer(0,seqMallocator,5*1024);

    prmDb.init(10,0);

    sockGndIf.init(0);

    fatalAdapter.init(0);
    fatalHandler.init(0);
    udpReceiver.init(0);

    // Connect rate groups to rate group driver
    constructSIMREFArchitecture();
    
    udpReceiver.set_PortsOut_OutputPort(0, leeCtrl.get_attRateThrust_InputPort(0));

    /* Register commands */
    cmdSeq.regCommands();
    cmdDisp.regCommands();
    eventLogger.regCommands();
    fileLogger.regCommands();
    prmDb.regCommands();
    fatalHandler.regCommands();

    leeCtrl.regCommands();
    attFilter.regCommands();
    mixer.regCommands();
    sigGen.regCommands();

    gzManipIf.regCommands();

    // initialize file logs
    fileLogger.initLog("./log/");
    
    // read parameters
    prmDb.readParamFile();
    leeCtrl.loadParameters();
    attFilter.loadParameters();
    mixer.loadParameters();
    sigGen.loadParameters();

    // Active component startup
    // start rate groups
    rg.start(0, 50, 20*1024);
    rgDecouple.start(0, 90, 20*1024);
    // start dispatcher
    cmdDisp.start(0,60,20*1024);
    // start sequencer
    cmdSeq.start(0,50,20*1024);
    // start telemetry
    eventLogger.start(0,50,20*1024);
    chanTlm.start(0,60,20*1024);
    prmDb.start(0,50,20*1024);

    fileLogger.start(0,50,20*1024);
    
    // Initialize socket server
    sockGndIf.startSocketTask(40, 20*1024, port_number, hostname);

    if (udp_string) {
        udpReceiver.open(udp_string);
        udpReceiver.startThread(85,20*1024);
    }

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
    rgDecouple.exit();
    cmdDisp.exit();
    eventLogger.exit();
    fileLogger.exit();
    chanTlm.exit();
    prmDb.exit();
    cmdSeq.exit();
}

void print_usage() {
    (void) printf("Usage: ./SIMREF [options]\n-p\tport_number\n-u\tUDP port number\n-a\thostname/IP address\n-l\tFor time-based cycles\n");
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
    char* udp_string = 0;
    bool local_cycle = false;

    // Removes ROS cmdline args as a side-effect
    ros::init(argc,argv,"SIMREF", ros::init_options::NoSigintHandler);

    while ((option = getopt(argc, argv, "hlp:a:u:")) != -1){
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
            case 'u':
                udp_string = optarg;
                break;
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

    (void) printf("Hit Ctrl-C to quit\n");

    // needs to be before constructApp so RosTime is ready
    ros::start();

    constructApp(port_number, udp_string, hostname);

    if (!local_cycle) {
        rgGncDrv.set_CycleOut_OutputPort(2, rg.get_CycleIn_InputPort(0));
    }
    //dumparch();

    Os::Task::TaskStatus stat = rosCycle.startIntTask(90, 20*1024);
    FW_ASSERT(Os::Task::TASK_OK == stat, stat);

    rotorSDrv.startPub();
    stat = rotorSDrv.startIntTask(70, 5*1000*1024);
    FW_ASSERT(Os::Task::TASK_OK == stat, stat);
    gzManipIf.startPub();
    stat = gzManipIf.startIntTask(70, 5*1000*1024);
    FW_ASSERT(Os::Task::TASK_OK == stat, stat);

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
