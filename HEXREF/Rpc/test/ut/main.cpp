#ifdef BUILD_SDFLIGHT
#include <HEXREF/Rpc/hexref.h>
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

static void sighandler(int signum) {
    terminate = 1;
}

void dummy() {
    while(!terminate) {
        Os::Task::delay(1000);
    }
}

void print_usage() {
    (void) printf("Usage: -i to disable init; -f to disable fini\n-o to run 1 cycle; -c to run continuously\n");
}

int main(int argc, char* argv[]) {
    bool noInit = false;
    bool noFini = false;
    bool kraitCycle = false;
    bool hexCycle = false;
    int numKraitCycles = 0;
    int option = 0;
    while ((option = getopt(argc, argv, "ifho:c")) != -1) {
        switch(option) {
            case 'h':
                print_usage();
                return 0;
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
                break;
            case 'c':
                hexCycle = true;
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

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);


#ifdef BUILD_SDFLIGHT
    if (kraitCycle) {
        hexref_cycle(numKraitCycles);
    }
#endif //BUILD_SDFLIGHT

    if (hexCycle) {
        while (!terminate) {
            Os::Task::delay(1000);
        }
    }

#ifdef BUILD_SDFLIGHT
    if (!noFini) {
        DEBUG_PRINT("Calling exit function for SDFLIGHT\n");
        hexref_fini();
    }
#endif //BUILD_SDFLIGHT

    if (hexCycle) {
        DEBUG_PRINT("Waiting for the runner to return\n");
        FW_ASSERT(task.join(NULL) == Os::Task::TASK_OK);
    }

    DEBUG_PRINT("Waiting for the Hexagon code to be unloaded - prevents hanging the board\n");
    FW_ASSERT(waiter.join(NULL) == Os::Task::TASK_OK);

    return 0;
}
