#ifdef BUILD_SDFLIGHT
#include <HEXREF/Rpc/hexref.h>
#endif //BUILD_SDFLIGHT

#include <signal.h>
#include <stdio.h>
#include <Os/Task.hpp>

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

extern "C" {
  int main(int argc, char* argv[]);
};

volatile sig_atomic_t terminate = 0;

static void sighandler(int signum) {
  terminate = 1;
}

int main(int argc, char* argv[]) {
  signal(SIGINT,sighandler);
  signal(SIGTERM,sighandler);

  Os::Task task;
  
#ifdef BUILD_SDFLIGHT
  // TODO(mereweth) - test that calling other functions before init has no effect
  //hexref_rpc_relay_buff_allocate(10);
  hexref_init();
  Os::TaskString task_name("HEXRPC");
  task.start(task_name, 0, 10, 20*1024, (Os::Task::taskRoutine) hexref_run, NULL);
#endif //BUILD_SDFLIGHT

  while (!terminate) {
    Os::Task::delay(1000);
  }
  
#ifdef BUILD_SDFLIGHT
  DEBUG_PRINT("Calling exit function for SDFLIGHT\n");
  hexref_fini();
#endif //BUILD_SDFLIGHT
  
  return 0;
}
