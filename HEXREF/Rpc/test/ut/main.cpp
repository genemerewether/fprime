#ifdef BUILD_SDFLIGHT
#include <HEXREF/Rpc/hexref.h>
#endif //BUILD_SDFLIGHT

#include <getopt.h>

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
  (void) printf("Usage: -i to disable init; -f to disable fini\n");
}

int main(int argc, char* argv[]) {
  bool noInit = false;
  bool noFini = false;
  int option = 0;
  while ((option = getopt(argc, argv, "ifh")) != -1){
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
    case '?':
      return 1;
    default:
      print_usage();
      return 1;
    }
  }

Os::Task task;
#ifdef BUILD_SDFLIGHT
  // TODO(mereweth) - test that calling other functions before init has no effect
  //hexref_rpc_relay_buff_allocate(10);
  if (!noInit) {
    hexref_init();
    Os::TaskString task_name("HEXRPC");
    task.start(task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) hexref_run, NULL);
  }
#else
  if (!noInit) {
    Os::TaskString task_name("DUMMY");
    task.start(task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) dummy, NULL);
  }
#endif //BUILD_SDFLIGHT

  signal(SIGINT,sighandler);
  signal(SIGTERM,sighandler);
  
  while (!terminate) {
    Os::Task::delay(1000);
  }
  
#ifdef BUILD_SDFLIGHT
  if (!noFini) {
    DEBUG_PRINT("Calling exit function for SDFLIGHT\n");
    hexref_fini();
  }
#endif //BUILD_SDFLIGHT

  if (!noInit) {
    DEBUG_PRINT("Waiting for the runner to return\n");
    FW_ASSERT(task.join(NULL) == Os::Task::TASK_OK);
  }
  
  return 0;
}
