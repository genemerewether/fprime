/*
 * main.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: tim
 */

#include <fprime-zmq/zmq-adapter/test/ut/Tester.hpp>
#include <Os/Task.hpp>
#include <string.h>

volatile bool quit = false;

static void sighandler(int signum) {
    quit = true;
}

void usage(char* prog) {
    printf("Usage: %s [server|client]\n",prog);
}

int main(int argc, char* argv[]) {

    if (argc != 2) {
        usage(argv[0]);
        return -1;
    }

    // check to see if server or client
    bool server;
    if (strcmp(argv[1],"server") == 0) {
        server = true;
        printf("Running as server.\n");
    } else if (strcmp(argv[1],"client") == 0) {
        server = false;
        printf("Running as client.\n");
    } else {
        usage(argv[0]);
        return -1;
    }

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

    Zmq::ZmqAdapterComponentImpl adapter("adapter");

    adapter.init(10,1024,0);
    adapter.open(server,"127.0.0.1","50000",100,20*1024,0);
    adapter.start(0, 100, 20*1024);

    if (server) {
        // make server a loopback
        adapter.set_PortsOut_OutputPort(0,adapter.get_PortsIn_InputPort(0));
        while (not quit) {
            Os::Task::delay(1000);
        }
    } else {
        // client will feed packets to server
        while (not quit) {
            Os::Task::delay(1000);
            Fw::InputSerializePort* port = adapter.get_PortsIn_InputPort(0);
            Zmq::ZmqAdapterComponentImpl::ZmqSerialBuffer buff;
            buff.serialize((U32)10);
            port->invokeSerial(buff);
        }
    }

    printf("Quitting\n");

    adapter.exit();

    return 0;
}

