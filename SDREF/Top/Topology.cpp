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

//#define LLROUTER_DEVICES

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

Fw::MallocAllocator seqMallocator;

Svc::RateGroupDriverImpl* rgDrv_ptr = 0;
Svc::ActiveRateGroupImpl* rgTlm_ptr = 0;
Svc::ActiveRateGroupImpl* rgXfer_ptr = 0;

Svc::SocketGndIfImpl* sockGndIf_ptr = 0;
Svc::SocketGndIfImpl* sockGndIfLL_ptr = 0;
Svc::ConsoleTextLoggerImpl* textLogger_ptr = 0;
Svc::ActiveLoggerImpl* eventLogger_ptr = 0;
Svc::ActiveLoggerImpl* eventLoggerLL_ptr = 0;
Svc::ActiveFileLoggerImpl* fileLogger_ptr = 0;
Svc::SerLoggerComponentImpl* serLogger_ptr = 0;
Svc::LinuxTimeImpl* linuxTime_ptr = 0;
Svc::TlmChanImpl* chanTlm_ptr = 0;
Svc::CommandDispatcherImpl* cmdDisp_ptr = 0;
Svc::CmdSequencerComponentImpl* cmdSeq_ptr = 0;
Svc::CmdSequencerComponentImpl* cmdSeqLL_ptr = 0;
Svc::PrmDbImpl* prmDb_ptr = 0;
Svc::SerialTextConverterComponentImpl* serialTextConv_ptr = 0;
Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr = 0;
Svc::FatalHandlerComponentImpl* fatalHandler_ptr = 0;

SnapdragonFlight::HexRouterComponentImpl* hexRouter_ptr = 0;
HLProc::LLRouterComponentImpl* llRouter_ptr = 0;
HLProc::HLRosIfaceComponentImpl* sdRosIface_ptr = 0;
HLProc::EventExpanderComponentImpl* eventExp_ptr = 0;
Svc::UdpReceiverComponentImpl* udpReceiver_ptr = 0;

Drv::LinuxSerialDriverComponentImpl* serialDriverLL_ptr = 0;
Drv::LinuxSerialDriverComponentImpl* serialDriverDebug_ptr = 0;

void allocComps() {
    // Component instance pointers
    NATIVE_INT_TYPE rgDivs[] = {10, 1};
    rgDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDRV",
#endif
                        rgDivs,FW_NUM_ARRAY_ELEMENTS(rgDivs));

    NATIVE_UINT_TYPE rgTlmContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = { 0 };
    rgTlm_ptr = new Svc::ActiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                        "RGTLM",
#endif
                        rgTlmContext,FW_NUM_ARRAY_ELEMENTS(rgTlmContext));

    NATIVE_UINT_TYPE rgXferContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = { 0 };
    rgXfer_ptr = new Svc::ActiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                        "RGXFER",
#endif
                        rgXferContext,FW_NUM_ARRAY_ELEMENTS(rgXferContext));

    sockGndIf_ptr = new Svc::SocketGndIfImpl
#if FW_OBJECT_NAMES == 1
                        ("SGIF")
#endif
;

    sockGndIfLL_ptr = new Svc::SocketGndIfImpl
#if FW_OBJECT_NAMES == 1
                        ("SGIFLL")
#endif
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

    eventLoggerLL_ptr = new Svc::ActiveLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("ELOGLL")
#endif
;

    fileLogger_ptr = new Svc::ActiveFileLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("FLOG")
#endif
;

    serLogger_ptr = new Svc::SerLoggerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SERLOG")
#endif
;
    
    linuxTime_ptr = new Svc::LinuxTimeImpl
#if FW_OBJECT_NAMES == 1
                        ("LTIME")
#endif
;

    chanTlm_ptr = new Svc::TlmChanImpl
#if FW_OBJECT_NAMES == 1
                        ("TLM")
#endif
;

    cmdDisp_ptr = new Svc::CommandDispatcherImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDDISP")
#endif
;

    cmdSeq_ptr = new Svc::CmdSequencerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDSEQ")
#endif
;

    cmdSeqLL_ptr = new Svc::CmdSequencerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDSEQLL")
#endif
;

    prmDb_ptr = new Svc::PrmDbImpl
#if FW_OBJECT_NAMES == 1
                        ("PRM",PRM_PATH)
#else
                        (PRM_PATH)
#endif
;

    hexRouter_ptr = new SnapdragonFlight::HexRouterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("HEXRTR")
#endif
;

    llRouter_ptr = new HLProc::LLRouterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("LLROUTER")
#endif
;

    serialDriverLL_ptr = new Drv::LinuxSerialDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SERIALDRVLL")
#endif
;

    serialDriverDebug_ptr = new Drv::LinuxSerialDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SERIALDRVDBUG")
#endif
;

    serialTextConv_ptr = new Svc::SerialTextConverterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("STCONVERTER")
#endif
;

    sdRosIface_ptr = new HLProc::HLRosIfaceComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SDROSIFACE")
#endif
;

    eventExp_ptr = new HLProc::EventExpanderComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("EVEXP")
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

    udpReceiver_ptr = new Svc::UdpReceiverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("UDPRECV")
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

void manualConstruct() {
    serLogger_ptr->set_LogOut_OutputPort(0, fileLogger_ptr->get_LogQueue_InputPort(0));
  
#ifndef LLROUTER_DEVICES
    // Sequence Com buffer and cmd response
    cmdSeq_ptr->set_comCmdOut_OutputPort(1, hexRouter_ptr->get_KraitPortsIn_InputPort(0));
    hexRouter_ptr->set_HexPortsOut_OutputPort(0, cmdSeq_ptr->get_cmdResponseIn_InputPort(1));

    hexRouter_ptr->set_HexPortsOut_OutputPort(1, sdRosIface_ptr->get_Imu_InputPort(0));
    hexRouter_ptr->set_HexPortsOut_OutputPort(2, sdRosIface_ptr->get_Odometry_InputPort(0));
    hexRouter_ptr->set_HexPortsOut_OutputPort(3, sdRosIface_ptr->get_AccelCommand_InputPort(0));
    hexRouter_ptr->set_HexPortsOut_OutputPort(4, eventExp_ptr->get_LogRecv_InputPort(0));
    hexRouter_ptr->set_HexPortsOut_OutputPort(5, sockGndIfLL_ptr->get_downlinkPort_InputPort(0));
    hexRouter_ptr->set_HexPortsOut_OutputPort(6, serLogger_ptr->get_SerPortIn_InputPort(0));
    
    rgXfer_ptr->set_RateGroupMemberOut_OutputPort(2, hexRouter_ptr->get_Sched_InputPort(0));
    
    sdRosIface_ptr->set_ImuStateUpdate_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(1));
    sdRosIface_ptr->set_ActuatorsData_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(2));
    sdRosIface_ptr->set_ActuatorsData_OutputPort(1, hexRouter_ptr->get_KraitPortsIn_InputPort(3));
    sockGndIfLL_ptr->set_uplinkPort_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(4));
    sdRosIface_ptr->set_flatOutput_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(5));
    sdRosIface_ptr->set_attRateThrust_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(6));
    udpReceiver_ptr->set_PortsOut_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(7));
#else
    // Sequence Com buffer and cmd response
    cmdSeq_ptr->set_comCmdOut_OutputPort(1, llRouter_ptr->get_HLPortsIn_InputPort(0));
    llRouter_ptr->set_LLPortsOut_OutputPort(0, cmdSeq_ptr->get_cmdResponseIn_InputPort(1));

    llRouter_ptr->set_LLPortsOut_OutputPort(1, sdRosIface_ptr->get_Imu_InputPort(0));
    llRouter_ptr->set_LLPortsOut_OutputPort(2, sdRosIface_ptr->get_Odometry_InputPort(0));
    llRouter_ptr->set_LLPortsOut_OutputPort(3, sdRosIface_ptr->get_AccelCommand_InputPort(0));
    llRouter_ptr->set_LLPortsOut_OutputPort(4, eventExp_ptr->get_LogRecv_InputPort(0));
    llRouter_ptr->set_LLPortsOut_OutputPort(5, sockGndIfLL_ptr->get_downlinkPort_InputPort(0));
    llRouter_ptr->set_LLPortsOut_OutputPort(6, serLogger_ptr->get_SerPortIn_InputPort(0));
    
    sdRosIface_ptr->set_ImuStateUpdate_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(1));
    sdRosIface_ptr->set_ActuatorsData_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(2));
    sdRosIface_ptr->set_ActuatorsData_OutputPort(1, llRouter_ptr->get_HLPortsIn_InputPort(3));
    sockGndIfLL_ptr->set_uplinkPort_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(4));
    sdRosIface_ptr->set_flatOutput_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(5));
    sdRosIface_ptr->set_attRateThrust_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(6));
    udpReceiver_ptr->set_PortsOut_OutputPort(0, llRouter_ptr->get_HLPortsIn__InputPort(7));
#endif
}

void constructApp(int port_number, int ll_port_number, char* udp_string, char* hostname) {
    allocComps();
  
    localTargetInit();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgDrv_ptr->init();

    // Initialize the rate groups
    rgTlm_ptr->init(10,0);
    rgXfer_ptr->init(10,0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger_ptr->init();
#endif

    eventLogger_ptr->init(10,0);
    eventLoggerLL_ptr->init(10,0);
    fileLogger_ptr->init(10);
    serLogger_ptr->init(0);
    serLogger_ptr->setStreamId(AFL_ACTADAP_ESC);

    linuxTime_ptr->init(0);

    chanTlm_ptr->init(10,0);

    cmdDisp_ptr->init(20,0);

    cmdSeq_ptr->init(10,0);
    cmdSeq_ptr->allocateBuffer(0,seqMallocator,5*1024);

    cmdSeqLL_ptr->init(10,0);
    cmdSeqLL_ptr->allocateBuffer(0,seqMallocator,5*1024);

    prmDb_ptr->init(10,0);

    sockGndIf_ptr->init(0);
    sockGndIfLL_ptr->init(0);

    eventExp_ptr->init(0);

    fatalAdapter_ptr->init(0);
    fatalHandler_ptr->init(0);

    hexRouter_ptr->init(10, 1000); // message size
    sdRosIface_ptr->init(0);
    
    serialTextConv_ptr->init(20,0);
    llRouter_ptr->init(10,SERIAL_BUFFER_SIZE,0);
    serialDriverLL_ptr->init();
    serialDriverDebug_ptr->init();

    udpReceiver_ptr->init(0);
    
    // Connect rate groups to rate group driver
    constructSDREFArchitecture();

    manualConstruct();

    const U32 tempPortNum[2] = {0, 1};
    const FwOpcodeType tempMinOpcode[2] = {0, 10000};
    const FwOpcodeType tempMaxOpcode[2] = {9999, 19999};
    cmdSeq_ptr->setOpCodeRanges(2,
                                tempPortNum,
                                tempMinOpcode,
                                tempMaxOpcode);
    /*cmdSeq1_ptr->setOpCodeRanges(2,
                                  tempPortNum,
                                  tempMinOpcode,
                                  tempMaxOpcode);*/

    /* Register commands */
    cmdSeq_ptr->regCommands();
    cmdSeqLL_ptr->regCommands();
    cmdDisp_ptr->regCommands();
    eventLogger_ptr->regCommands();
    eventLoggerLL_ptr->regCommands();
    fileLogger_ptr->regCommands();
    prmDb_ptr->regCommands();

    llRouter_ptr->regCommands();
    serialTextConv_ptr->regCommands();
    
    // initialize file logs
    fileLogger_ptr->initLog("/log/");

    // read parameters
    prmDb_ptr->readParamFile();

    char logFileName[256];
    snprintf(logFileName, sizeof(logFileName), "/eng/STC_%u.txt", 0); //boot_count % 10);
    serialTextConv_ptr->set_log_file(logFileName, 100*1024, 0);
    
    // Active component startup
    // start rate groups
    rgTlm_ptr->start(0, 50, 20*1024);
    rgXfer_ptr->start(0, 95, 20*1024);
    // start dispatcher
    cmdDisp_ptr->start(0,60,20*1024);
    // start sequencer
    cmdSeq_ptr->start(0,50,20*1024);
    cmdSeqLL_ptr->start(0,50,20*1024);
    // start telemetry
    eventLogger_ptr->start(0,50,20*1024);
    eventLoggerLL_ptr->start(0,50,20*1024);
    chanTlm_ptr->start(0,60,20*1024);
    prmDb_ptr->start(0,50,20*1024);

    hexRouter_ptr->start(0, 90, 20*1024);//, CORE_RPC);

    llRouter_ptr->start(0, 85, 20*1024);
    serialTextConv_ptr->start(0,79,20*1024);

    fileLogger_ptr->start(0,50,20*1024);
    
    hexRouter_ptr->startPortReadThread(90,20*1024); //, CORE_RPC);
    //hexRouter_ptr->startBuffReadThread(60,20*1024, CORE_RPC);

#ifdef LLROUTER_DEVICES
    // Must start serial drivers after tasks that setup the buffers for the driver:
    serialDriverLL_ptr->open(
#ifdef BUILD_SDFLIGHT
                        "/dev/ttyHS3",
#elif defined BUILD_LINUX
                        "/dev/ttyUSB0",
#else
                        "/dev/ttyUSB0",
#endif
                        Drv::LinuxSerialDriverComponentImpl::BAUD_921K,
                        Drv::LinuxSerialDriverComponentImpl::NO_FLOW,
                        Drv::LinuxSerialDriverComponentImpl::PARITY_NONE,
                        true);
    
    serialDriverDebug_ptr->open(
#ifdef BUILD_SDFLIGHT
                           "/dev/ttyHS2",
#elif defined BUILD_LINUX
                           "/dev/ttyUSB1",
#else
                           "/dev/ttyUSB1",
#endif
                           Drv::LinuxSerialDriverComponentImpl::BAUD_921K,
                           Drv::LinuxSerialDriverComponentImpl::NO_FLOW,
                           Drv::LinuxSerialDriverComponentImpl::PARITY_NONE,
                           true);
    
    /* ---------- Done opening devices, now start device threads ---------- */
    serialDriverLL_ptr->startReadThread(98, 20*1024);
    serialDriverDebug_ptr->startReadThread(40, 20*1024);
#endif
    
    // Initialize socket server
    if (port_number && hostname) {
        sockGndIf_ptr->startSocketTask(40, 20*1024, port_number, hostname);
    }
    if (ll_port_number && hostname) {
        sockGndIfLL_ptr->startSocketTask(40, 20*1024, ll_port_number, hostname);
    }

    if (udp_string) {
        udpReceiver_ptr->open(udp_string);
        udpReceiver_ptr->startThread(85,20*1024);
    }
    
#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}


void run1cycle(void) {
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rgDrv_ptr->get_CycleIn_InputPort(0);
    Svc::TimerVal cycleStart;
    cycleStart.take();
    port->invoke(cycleStart);
    Os::Task::delay(100);
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
    hexRouter_ptr->quitReadThreads();
    
#ifdef LLROUTER_DEVICES
    serialDriverLL_ptr->quitReadThread();
    serialDriverDebug_ptr->quitReadThread();
#endif
    
    llRouter_ptr->exit();
    serialTextConv_ptr->exit();
    
    DEBUG_PRINT("After HexRouter read thread quit\n");
    rgTlm_ptr->exit();
    rgXfer_ptr->exit();
    cmdDisp_ptr->exit();
    eventLogger_ptr->exit();
    eventLoggerLL_ptr->exit();
    chanTlm_ptr->exit();
    prmDb_ptr->exit();
    fileLogger_ptr->exit();
    cmdSeq_ptr->exit();
    cmdSeqLL_ptr->exit();
    hexRouter_ptr->exit();
    DEBUG_PRINT("After HexRouter quit\n");
}

void print_usage() {
    (void) printf("Usage: ./SDREF [options]\n-p\tport_number\n-x\tll_port_number\n-u\tUDP port number\n-a\thostname/IP address\n-l\tFor time-based cycles\n-i\tto disable init\n-f\tto disable fini\n-o to run # cycles instead of continuously\n");
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
    U32 ll_port_number = 0;
    I32 option = 0;
    char *hostname = NULL;
    char* udp_string = 0;
    bool local_cycle = true;

    // Removes ROS cmdline args as a side-effect
    ros::init(argc,argv,"SDREF", ros::init_options::NoSigintHandler);

    while ((option = getopt(argc, argv, "ifhlp:x:a:u:o:")) != -1){
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
            case 'x':
                ll_port_number = atoi(optarg);
                break;
            case 'a':
                hostname = optarg;
                break;
            case 'u':
                udp_string = optarg;
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

#ifdef BUILD_SDFLIGHT
    if (!noInit) {
        hexref_init();
    }
#endif
    
    constructApp(port_number, ll_port_number, udp_string, hostname);
    //dumparch();

    ros::start();

    sdRosIface_ptr->startPub();
    sdRosIface_ptr->startIntTask(30, 20*1024);

    Os::Task task;
    Os::Task waiter;
    Os::TaskString waiter_task_name("WAITER");
#ifdef BUILD_SDFLIGHT
    // TODO(mereweth) - test that calling other functions before init has no effect
    //hexref_rpc_relay_buff_allocate(10);
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
      //DEBUG_PRINT("Cycle %d\n",cycle);
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
