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
        ACTIVE_COMP_ATT_RG,
        ACTIVE_COMP_POS_RG,
        ACTIVE_COMP_LOGGER,

        CYCLER_TASK,
        NUM_ACTIVE_COMPS
};

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

// Component instance pointers

// rg, rgAtt, rgPos
static NATIVE_INT_TYPE rgDivs[] = {1000, 1, 5};
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

static NATIVE_UINT_TYPE rgAttContext[] = {0,0,0,0,0,0,0,0,0,0};
Svc::ActiveRateGroupImpl rgAtt(
#if FW_OBJECT_NAMES == 1
                    "RGATT",
#endif
                    rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

static NATIVE_UINT_TYPE rgPosContext[] = {0,0,0,0,0,0,0,0,0,0};
Svc::ActiveRateGroupImpl rgPos(
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

void constructApp() {

    localTargetInit();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif    

    // Initialize rate group driver
    rgDrv.init();

    // Initialize the rate groups
    rg.init(10,0);
    rgAtt.init(10,0);
    rgPos.init(10,0);

#if FW_ENABLE_TEXT_LOGGING
    textLogger.init();
#endif

    eventLogger.init(10,0);

    linuxTime.init(0);

    fatalAdapter.init(0);
    fatalHandler.init(0);
    health.init(25,0);

    // TODO(mereweth) - update if queued or active
    kraitRouter.init(0);

    // Connect rate groups to rate group driver
    constructHEXREFArchitecture();

    // Manual connections
    // TODO(mereweth) - multiple DSPAL components with commands?
    //kraitRouter.set_KraitPortsOut_OutputPort(0, .get_CmdDisp_InputPort(0));
    //.set_CmdStatus_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(0);

    //kraitRouter.set_KraitPortsOut_OutputPort(0, .get_CmdDisp_InputPort(0));
    //.set_CmdStatus_OutputPort(0, kraitRouter.get_HexPortsIn_InputPort(0);
    
    /* Register commands */
    /*eventLogger.regCommands();
    health.regCommands();*/

    // set health ping entries

    Svc::HealthImpl::PingEntry pingEntries[] = {
        {3,5,rg.getObjName()}, // 0
        {3,5,eventLogger.getObjName()}, // 1
    };

    // register ping table
    //health.setPingEntries(pingEntries,FW_NUM_ARRAY_ELEMENTS(pingEntries),0x123);

    // Active component startup
    // start rate groups
    rg.start(ACTIVE_COMP_1HZ_RG, 50,10 * 1024);
    rgAtt.start(ACTIVE_COMP_ATT_RG, 90,10 * 1024);
    rgPos.start(ACTIVE_COMP_POS_RG, 80,10 * 1024);
    // start telemetry
    eventLogger.start(ACTIVE_COMP_LOGGER,40,10*1024);
    
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
    Os::Task::delay(1);
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
    rgAtt.exit();
    rgPos.exit();
    eventLogger.exit();
}

volatile bool terminate = false;

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

  //dumparch();
  
  Os::Task::delay(1000);
  
  return 0;
}

int hexref_run(void) {
  bool local_cycle = true;  
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
  DEBUG_PRINT("Waiting for threads...\n");
  Os::Task::delay(1000);
  
  DEBUG_PRINT("Exiting...\n");
  
  return 0; 
}

int hexref_fini(void) {
  DEBUG_PRINT("hexref_fini called...\n");
  terminate = true;
  kraitRouter.m_quit = true;
  return 0;
}

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
  return kraitRouter.buffRead(port, buff, buffLen, bytes);
}

int hexref_rpc_relay_port_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
  return kraitRouter.portRead(port, buff, buffLen, bytes);
}

int hexref_rpc_relay_write(unsigned int port, const unsigned char* buff, int buffLen) {
  return kraitRouter.write(port, buff, buffLen);
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
    
  hexref_run();
}

#endif //ifndef BUILD_DSPAL
