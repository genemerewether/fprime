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
#endif

#ifdef BUILD_DSPAL
#define DEBUG_PRINT(x,...) FARF(ALWAYS,x,##__VA_ARGS__);
#else
#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
#endif

//#undef DEBUG_PRINT
//#define DEBUG_PRINT(x,...)

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

#if FW_ENABLE_TEXT_LOGGING
Svc::ConsoleTextLoggerImpl textLogger
#if FW_OBJECT_NAMES == 1
                    ("TLOG")
#endif
;
#endif

Svc::LinuxTimeImpl linuxTime
#if FW_OBJECT_NAMES == 1
                    ("LTIME")
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

void constructApp() {

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
    
    linuxTime.init(0);

    fatalAdapter.init(0);
    fatalHandler.init(0);
    
    // Connect rate groups to rate group driver
    constructHEXREFArchitecture();

    // Active component startup
    // start rate groups
    rg.start(ACTIVE_COMP_1HZ_RG, 120,10 * 1024);

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
    rg.exit();
}

// DSPAL binary is launched by FastRPC call
#ifdef BUILD_DSPAL
int hexref_init(void) {
  bool local_cycle = true;

  DEBUG_PRINT("Before constructing app\n");
  constructApp();
  DEBUG_PRINT("After constructing app\n");
  
  Os::Task::delay(1000);

  //dumparch();

  //signal(SIGINT,sighandler);
  //signal(SIGTERM,sighandler);

  int cycle = 0;

  while (true) { //!terminate) {
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
  // Give time for threads to exit
  DEBUG_PRINT("Waiting for threads...\n");
  Os::Task::delay(1000);
  
  DEBUG_PRINT("Exiting...\n");
  
  return 0; 
}

#else // For testing

void print_usage() {
	(void) DEBUG_PRINT("Usage: ./HEXREF [options]\n-l\tFor time-based cycles\n");
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
	I32 option = 0;
        bool local_cycle = false;

	while ((option = getopt(argc, argv, "hl")) != -1){
		switch(option) {
			case 'h':
				print_usage();
				return 0;
				break;
                        case 'l':
                          local_cycle = true;
                          break;
			case '?':
				return 1;
			default:
				print_usage();
				return 1;
		}
	}

	(void) DEBUG_PRINT("Hit Ctrl-C to quit\n");

    constructApp();
    //dumparch();

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

    int cycle = 0;

    while (!terminate) {
      (void) DEBUG_PRINT("Cycle %d\n",cycle);
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
    (void) DEBUG_PRINT("Waiting for threads...\n");
    Os::Task::delay(1000);

    (void) DEBUG_PRINT("Exiting...\n");

    return 0;
}

#endif //ifdef BUILD_DSPAL
