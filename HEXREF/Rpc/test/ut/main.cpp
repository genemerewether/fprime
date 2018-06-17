#ifdef BUILD_SDFLIGHT
#include <HEXREF/Rpc/hexref.h>
#include <SnapdragonFlight/HexRouter/HexRouterComponentImpl.hpp>
#endif //BUILD_SDFLIGHT

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#include <signal.h>
#include <stdio.h>
#include <Os/Task.hpp>

#include <Fw/Types/Assert.hpp>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

extern "C" {
    int main(int argc, char* argv[]);
};

volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t fini = 1;

static void sighandler(int signum) {
    terminate = 1;
    if (signum == SIGHUP) {
        fini = 1;
    }
}

void dummy() {
    while(!terminate) {
        Os::Task::delay(1000);
    }
}

void print_usage() {
    (void) printf("Usage: -i to disable init; -f to disable fini\n-o to run # cycles; -c to run continuously\n-r to use HexRouter\n");
}

int main(int argc, char* argv[]) {
    bool noInit = false;
    bool kraitCycle = false;
    bool hexCycle = false;
    bool hexRtr = false;
    int numKraitCycles = 0;
    int option = 0;
    while ((option = getopt(argc, argv, "rifho:c")) != -1) {
        switch(option) {
            case 'h':
                print_usage();
                return 0;
                break;
            case 'i':
                noInit = true;
                break;
            case 'f':
                fini = false; // overridden in case of SIGHUP
                break;
            case 'o':
                numKraitCycles = atoi(optarg);
                kraitCycle = true;
                break;
            case 'c':
                hexCycle = true;
                break;
            case 'r':
                hexRtr = true;
                break;
            case '?':
                print_usage();
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
    signal(SIGHUP,sighandler);

    Os::Task task;
    Os::Task waiter;
    Os::TaskString waiter_task_name("WAITER");
#ifdef BUILD_SDFLIGHT
    SnapdragonFlight::HexRouterComponentImpl hexRouter("HEXRTR");
    // TODO(mereweth) - test that calling other functions before init has no effect
    //hexref_rpc_relay_buff_allocate(10);
    if (!noInit) {
        hexref_init();
    }
    if (hexRtr) {
        DEBUG_PRINT("Starting HexRouter\n");
        hexRouter.init(10, 0);
        hexRouter.startPortReadThread(90, 20*1024, 0);
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

    if (hexCycle) {
        while (!terminate) {
            DEBUG_PRINT("Waiting on Krait\n");
            Os::Task::delay(1000);
        }
        DEBUG_PRINT("Terminate is true\n");
    }

#ifdef BUILD_SDFLIGHT
    if (fini) {
        DEBUG_PRINT("Calling exit function for SDFLIGHT\n");
        hexref_fini();
    }
#endif //BUILD_SDFLIGHT

    if (hexRtr) {
        DEBUG_PRINT("Quitting hexrouter read threads\n");
        hexRouter.quitReadThreads();
        //hexRouter.exit();
    }

    if (hexCycle) {
        DEBUG_PRINT("Waiting for the runner to return\n");
        FW_ASSERT(task.join(NULL) == Os::Task::TASK_OK);
    }

    DEBUG_PRINT("Waiting for the Hexagon code to be unloaded - prevents hanging the board\n");
    FW_ASSERT(waiter.join(NULL) == Os::Task::TASK_OK);

    return 0;
}
