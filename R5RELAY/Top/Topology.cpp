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

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers

static NATIVE_UINT_TYPE rgContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
    0, // llRouter
    0, // llRouterDone
};

Svc::PassiveRateGroupImpl rg(
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

HLProc::LLRouterComponentImpl llRouter
#if FW_OBJECT_NAMES == 1
                    ("LLROUTER")
#endif
;

Drv::LinuxSerialDriverComponentImpl serialDrv
#if FW_OBJECT_NAMES == 1
                    ("SERIALDRV")
#endif
;

HLProc::EventExpanderComponentImpl eventExpander
#if FW_OBJECT_NAMES == 1
                    ("EVREXP")
#endif
;

Svc::LinuxTimeImpl linuxTime
#if FW_OBJECT_NAMES == 1
                    ("TIME")
#endif
;

Svc::ActiveLoggerImpl activeLogger
#if FW_OBJECT_NAMES == 1
                    ("ACTIVELOGGER")
#endif
;

//Svc::AssertFatalAdapterComponentImpl fatalAdapter
//#if FW_OBJECT_NAMES == 1
//("fatalAdapter")
//#endif
//;

//Svc::FatalHandlerComponentImpl fatalHandler
//#if FW_OBJECT_NAMES == 1
//("fatalHandler")
//#endif
//;


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

    llRouter.set_LLPortsOut_OutputPort(4, eventExpander.get_LogRecv_InputPort(0));
    llRouter.set_LLPortsOut_OutputPort(5, sockGndIf.get_downlinkPort_InputPort(0));

    sockGndIf.set_uplinkPort_OutputPort(0, llRouter.get_HLPortsIn_InputPort(4));
}

void constructApp(int port_number, char* udp_string, char* hostname, char* serial_port) {

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif


    // Initialize the rate groups
    rg.init(0);
    serialDrv.init(0);
    sockGndIf.init(0);
    llRouter.init(100, 1000, 0);
    linuxTime.init(0);
    eventExpander.init(0);
    activeLogger.init(100, 0);


    // Connect rate groups to rate group driver
    constructR5RELAYArchitecture();

    manualConstruct();
    
    // Active component startup

    // Initialize socket server
    sockGndIf.startSocketTask(40, 20*1024, port_number, hostname);

    llRouter.start(0, 40, 8192);
    activeLogger.start(0, 40, 8192);

    serialDrv.open(serial_port,
                   Drv::LinuxSerialDriverComponentImpl::BAUD_115K,
                   Drv::LinuxSerialDriverComponentImpl::NO_FLOW,
                   Drv::LinuxSerialDriverComponentImpl::PARITY_NONE,
                   true);

    serialDrv.startReadThread(40, 8192);


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
    char* serial_port = NULL;
    bool local_cycle = false;

    while ((option = getopt(argc, argv, "hlp:a:u:s:")) != -1){
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
            case 's':
                serial_port = optarg;
                break;
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

    (void) printf("Hit Ctrl-C to quit\n");

    constructApp(port_number, udp_string, hostname, serial_port);

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

    return 0;
}
