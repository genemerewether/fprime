/*
 * main.cpp
 *
 *  Created on: Jul 4, 2017
 *      Author: tim
 */

#include <fprime-zmq/zmq-pub/test/ut/Tester.hpp>
#include <Os/Task.hpp>
#include <string.h>

volatile bool quit = false;

static void sighandler(int signum) {
    quit = true;
}

void usage(const char* prog) {
    printf("Usage %s <port>\n",prog);
}

int main(int argc, char* argv[]) {

    signal(SIGINT,sighandler);
    signal(SIGTERM,sighandler);

    Zmq::ZmqPubComponentImpl adapter("pub");

    if (argc != 2) {
        usage(argv[0]);
        return -1;
    }

    adapter.init(10,100,0);
    adapter.open(argv[1]);
    adapter.start(0, 90, 20*1024);

    // feed packets to publisher
    NATIVE_UINT_TYPE portNum = 0;
    while (not quit) {
        Os::Task::delay(1000);
        Fw::InputSerializePort* port = adapter.get_PortsIn_InputPort(portNum);
        Zmq::ZmqPubComponentImpl::ZmqSerialBuffer buff;
        buff.serialize((U32)portNum);
        port->invokeSerial(buff);
        portNum = (portNum+1)%adapter.getNum_PortsIn_InputPorts();
    }

    printf("Quitting\n");

    adapter.exit();

    return 0;
}

