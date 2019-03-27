/*
 * main.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: tim
 */

#include <fprime-zmq/zmq-sub/test/ut/Tester.hpp>
#include <Os/Task.hpp>
#include <string.h>

volatile bool quit = false;

static void sighandler(int signum) {
    quit = true;
}

void usage(const char* prog) {
    printf("Usage %s <server> <port>\n",prog);
}

int main(int argc, char* argv[]) {

    if (argc != 3) {
        usage(argv[0]);
        return -1;
    }

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

    Zmq::ZmqSubComponentImpl adapter("sub");

    adapter.init(0);
    adapter.open(argv[1],argv[2],90,20*1024,0);

    // spin waiting for termination
    while (not quit) {
        Os::Task::delay(1000);
    }

    printf("Quitting\n");

    return 0;
}

