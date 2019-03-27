# fprime-zmq

This repository is meant to be used as a sub-repository under an F' derived repository. It adds ZeroMQ:

http://zeromq.org

The `zmq` directory has the ported ZMQ source code and license files.

## Components

The following components are provided:

|Component|Description|Status|
|---|---|---|
|`zmq-router`|A generic router with serialize input/ouput ports for sending port invocations to other nodes via ZMQ.|In progress, doesn't work yet|
|`zmq-pub`|A component with serialize input ports for publishing port calls.|Not started|
|`zmq-sub`|A component with serialize output ports for subscribing to port calls.|Not started|
|`zmq-radio`|A component with uplink/downlink ports that communicate via ZMQ with a ground system|Delivered|

See the component SDD files in their respective `docs` subdirectories.

## Modifications to your repo:

The following modifications should be made to the build configuration files to allow the code to compile:

`<root>/mk/configs/compiler/include_common.mk`

Add:

`-I$(BUILD_ROOT)/fprime-zmq/zmq/include`

`<root>/mk/configs/compiler/defines_common.mk`

Add:

`-DZMQ_BUILD_DRAFT_API` if you wish to use the ZMQ draft API. (Needed for `zmq_router`)


## Modification to repo for zmq-ref and zmq-radio to build properly

git submodule add https://github.jpl.nasa.gov/reder/fprime-zmq.git

Add the following to `<root>/mk/configs/module/modules.mk`

````
ZMQ-REF_MODULES := \
        fprime-zmq/zmq-ref/Top \
        Ref/RecvBuffApp \
        Ref/SendBuffApp \
        Ref/SignalGen \
        Ref/PingReceiver \
        fprime-zmq/zmq-radio \
        fprime-zmq/zmq


zmq-ref_MODULES := \
        $(ZMQ-REF_MODULES) \
        $(SVC_MODULES) \
        $(DRV_MODULES) \
        $(FW_MODULES) \
        $(OS_MODULES) \
        $(CFDP_MODULES) \
        $(UTILS_MODULES)
````

Change this to:

````
# Other modules to build, but not to link with deployment binaries
OTHER_MODULES := \
        gtest \
        Os/Stubs \
        Fw/Test \
        fprime-zmq/zmq-radio \
        fprime-zmq/zmq-pub \
        fprime-zmq/zmq-sub \
        $(FW_GTEST_MODULES)
````

Change this to:

````

# List deployments

DEPLOYMENTS := Ref acdev zmq-ref

````

Next build the submodule

````
cd `<root>/fprime-zmq/zmq-ref`
make rebuild
````

To start a simple flight and ground configuration running in seperate windows do these commands

````
cd ../Gse/bin
python run_zmq_server.py 50000
````

````
python gse.py -d ../generated/Ref -N ground -p 50000
````

````
zmq-ref -p 50000 -a localhost -n flight
````

Then configure using the run_zmq_server_config.py script.








