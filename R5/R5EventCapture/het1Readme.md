
= Adding an Event

== Finding a Pin

Find a pin on the TMS570 that is connected to the HET1 peripheral. There are two HETs inside the TMS570 and the EventCapture is setup to only work with HET1.

Many of the input pins on the TMS570 are multiplexed. Be sure that the HET1 input for the selected pin is enabled in HalCoGen-\>PINMUX-\>Input Pin Muxing.

== Editing HET Code 
If you are reading this you have the unfortunate task of editing the HET assembly.
HET asm can be updated using the Windows HET IDE from TI. There is a exiting project that can be modified in the EventHET.

To add an event, add a new WCAP instruction. Make sure to update the "next" and "cond\_addr" flags of the previous WCAP to point to your new instruction. You will also need to modify the "pin" flag to reflect the pin you want to capture and the "reqnum" to a new value. The "reqnum" must be unique in order for anything in EventCapture to work properly. Finally, point the "next" and "cond\_addr" flags to the location of the BR instruction.

After compiling the code, copy the generated het1\_proc.c/h to the R5EventCapture directory.

== Updating R5EventCapture.cpp

1. Update the ECNumEventPins parameter in R5EventCaptureCfg.hpp.
2. Add an entry in the ECConfigs array for the configuration. 
  * DCP should be identical to the "reqnum" of the new HET instruction
  * Instruction should be the instruction index in the HET assembly
3. Add a new entry in the ECPorts array, setting config the the new ECConfig structure created in step 2. The index in the ECPorts array of this entry will determine the F' port to use when receiving event times.
4. Increase the "max\_number" of input ports to R5EventCaptureComponentAi.xml if necessary to support the new port
5. Add a connection to R5EventCapture in the R5REF Topology. Be sure to connect to the correct port in R5EventCapture


