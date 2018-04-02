## topology
- factory init - globals but only init once

## SPI Driver
- add port for setting SPI bus speed to MD model

## MPU9250 driver
- add port for setting SPI bus speed to MD model

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
