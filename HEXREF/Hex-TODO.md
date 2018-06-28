## Time conversion
- adapt SnapdragonTime?
- use with dsp_offset kernel module (read during sched call?)

## topology
- factory init - globals but only init once

## Utils
- test HAP process list
    - Qualcomm/Hexagon_SDK/3.0/incs/HAP_ps.h

## Unit tests

### KraitRouter
- MD model
- Exit after some number of cycles in port/buffRead wait loop
- Make a fake Topology.cpp as the unit test
- Manually loop back ports of different sizes
    - Push data in
    - invoke one cycle
    - wait some short amount of time
    - check data out

## Autogenerating dependency on idl file

Generating Makefiles in /quest/mk/makefiles
**Error: Invalid Extension .idl on file HEXREFRpc/hexref.idl**
Command exited with non-zero status 255
